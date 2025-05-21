import time
import logging
import math

from ..utils import config
# We expect MotorController to be passed in, but type hinting is good.
# from .motor import MotorController # This would be a circular import if NavigationController is imported by main.py

logger = logging.getLogger(__name__)

class PIDController:
    """A simple PID controller class."""
    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = 0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()
        self.max_integral = 500 # Cap integral to prevent windup
        self.min_integral = -500

    def set_gains(self, kp: float, ki: float, kd: float):
        """Dynamically update PID gains."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        # Optionally, could reset integral and previous_error here, 
        # or leave them to adapt to new gains.
        # self.reset() # Consider if resetting state on gain change is desired.
        logger.info(f"PID gains updated: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")

    def compute(self, current_value: float) -> float:
        current_time = time.time()
        dt = current_time - self.last_time
        if dt == 0: # Avoid division by zero if called too rapidly
            dt = 1e-6

        error = self.setpoint - current_value
        
        self.integral += error * dt
        # Anti-windup for integral
        self.integral = max(self.min_integral, min(self.max_integral, self.integral))
        
        derivative = (error - self.previous_error) / dt
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        self.previous_error = error
        self.last_time = current_time
        
        return output

    def reset(self):
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()

class NavigationController:
    def __init__(self, motor_controller):
        self.motor_controller = motor_controller
        
        # Initialize PID controllers
        # For turning, the setpoint is 0 (we want the marker's angle to be 0)
        self.turning_pid = PIDController(
            kp=config.NAV_TURNING_PID_KP,
            ki=config.NAV_TURNING_PID_KI,
            kd=config.NAV_TURNING_PID_KD,
            setpoint=0 # Setpoint is 0 radians
        )
        
        # For distance, the setpoint is the target approach distance
        self.distance_pid = PIDController(
            kp=config.NAV_DISTANCE_PID_KP,
            ki=config.NAV_DISTANCE_PID_KI,
            kd=config.NAV_DISTANCE_PID_KD,
            setpoint=config.NAV_TARGET_APPROACH_DISTANCE_M
        )
        
        self._initial_turning_pid_gains = (config.NAV_TURNING_PID_KP, config.NAV_TURNING_PID_KI, config.NAV_TURNING_PID_KD)
        self._initial_distance_pid_gains = (config.NAV_DISTANCE_PID_KP, config.NAV_DISTANCE_PID_KI, config.NAV_DISTANCE_PID_KD)

        self.target_approach_distance_m = config.NAV_TARGET_APPROACH_DISTANCE_M
        self.distance_threshold_m = config.NAV_DISTANCE_THRESHOLD_M
        # self.x_threshold_m = config.NAV_X_THRESHOLD_M # Replaced by angular threshold
        self.angular_threshold_rad = config.NAV_X_THRESHOLD_RAD

        self.max_turning_speed = config.NAV_MAX_TURNING_SPEED
        self.max_forward_speed = config.NAV_MAX_FORWARD_SPEED
        self.min_effective_speed = config.NAV_MIN_EFFECTIVE_SPEED

        self.current_target_pose = None

    def set_target(self, tvec, rvec):
        self.current_target_pose = (tvec, rvec)
        self.turning_pid.reset()
        self.distance_pid.reset()
        # Log the raw tvec, calculations will happen in navigate()
        logger.info(f"NavigationController: New target set. tvec: {tvec.flatten().round(3)}")

    def clear_target(self):
        # Target is lost or reached, or being explicitly cleared
        logger.info("NavigationController.clear_target(): Called. Stopping motors and resetting PIDs.") # More specific
        self.current_target_pose = None
        self.motor_controller.stop()
        # Reset PIDs when target is cleared so they don't carry over stale values
        self.turning_pid.reset()
        self.distance_pid.reset()
        # logger.info("NavigationController: Target cleared, PIDs reset.") # Redundant with above

    def navigate(self) -> bool: # Returns True if actively navigating, False if at target or no target
        if not self.current_target_pose:
            # This case should ideally be preempted by clear_target() if pose becomes None by external factors.
            # logger.debug("Navigate called with no current target pose.") # Can be noisy
            return False

        tvec, rvec = self.current_target_pose
        
        x_cam = tvec[0][0]
        z_cam = tvec[0][2]

        # Avoid math domain error if z_cam is zero or negative (marker directly at or behind camera)
        if z_cam <= 0:
            logger.warning(f"Navigate: Marker z_cam is {z_cam:.3f}, too close or behind. Stopping.")
            self.motor_controller.stop()
            return False
            
        angle_to_marker_rad = math.atan2(x_cam, z_cam)
        current_planar_distance_m = math.sqrt(x_cam**2 + z_cam**2) # True planar distance

        if self.is_at_target(angle_to_marker_rad, current_planar_distance_m):
            logger.info("NavigationController: At target position.")
            self.motor_controller.stop()
            return False

        turning_effort = self.turning_pid.compute(current_value=angle_to_marker_rad)
        distance_effort = self.distance_pid.compute(current_value=current_planar_distance_m)

        # Clamp PID outputs - these factors might need tuning based on typical effort values
        # For angle, effort might be small (e.g., +/- PI/2 radians), for distance, effort could be larger.
        # Consider the typical range of 'error' for each PID.
        # Angle error range: e.g. -pi to +pi. Distance error: e.g. -1m to +1m.
        # The division factor helps normalize this before applying max_speed.
        clamped_turning_effort = max(-1.0, min(1.0, turning_effort / (math.pi/2) )) # Normalize by ~max expected angle error
        clamped_distance_effort = max(-1.0, min(1.0, distance_effort / 1.0)) # Normalize by ~1m max expected distance error

        turn_speed = clamped_turning_effort * self.max_turning_speed
        forward_speed = -clamped_distance_effort * self.max_forward_speed

        if 0 < abs(forward_speed) < self.min_effective_speed and not self.is_at_distance(current_planar_distance_m):
            forward_speed = self.min_effective_speed * (1 if forward_speed > 0 else -1)
        if 0 < abs(turn_speed) < self.min_effective_speed and not self.is_centered(angle_to_marker_rad):
             turn_speed = self.min_effective_speed * (1 if turn_speed > 0 else -1)

        if not self.is_centered(angle_to_marker_rad):
            logger.debug(f"Nav: Turning. Angle_err: {angle_to_marker_rad:.3f} rad, TurnEffortRaw: {turning_effort:.2f}, TurnSpeed: {turn_speed:.2f}")
            self.motor_controller.stop()
            if turn_speed > 0: 
                self.motor_controller.turn_right(abs(turn_speed))
            else: 
                self.motor_controller.turn_left(abs(turn_speed))
        elif not self.is_at_distance(current_planar_distance_m):
            logger.debug(f"Nav: Driving. Dist_err: {(current_planar_distance_m - self.target_approach_distance_m):.3f} m, DistEffortRaw: {distance_effort:.2f}, FwdSpeed: {forward_speed:.2f}")
            if forward_speed > 0:
                self.motor_controller.drive_forward(abs(forward_speed))
            else:
                self.motor_controller.drive_backward(abs(forward_speed))
        else:
            logger.debug("Nav: Minor adjustments or at target (in navigate), stopping.")
            self.motor_controller.stop()
            return False
            
        return True

    def is_centered(self, angle_to_marker_rad: float) -> bool:
        return abs(angle_to_marker_rad) < self.angular_threshold_rad

    def is_at_distance(self, current_planar_distance_m: float) -> bool:
        return abs(current_planar_distance_m - self.target_approach_distance_m) < self.distance_threshold_m

    def is_at_target(self, angle_to_marker_rad: float, current_planar_distance_m: float) -> bool:
        centered = self.is_centered(angle_to_marker_rad)
        at_distance = self.is_at_distance(current_planar_distance_m)
        if centered and at_distance:
            logger.debug("Navigation: Reached target (centered and at distance).")
            return True
        return False

    def stop_navigation(self):
        self.clear_target()
        logger.info("NavigationController: Navigation explicitly stopped.")

    def update_turning_pid_gains(self, kp: float, ki: float, kd: float):
        self.turning_pid.set_gains(kp, ki, kd)
        logger.info(f"NavigationController: Turning PID gains updated to Kp={kp}, Ki={ki}, Kd={kd}")

    def update_distance_pid_gains(self, kp: float, ki: float, kd: float):
        self.distance_pid.set_gains(kp, ki, kd)
        logger.info(f"NavigationController: Distance PID gains updated to Kp={kp}, Ki={ki}, Kd={kd}") 