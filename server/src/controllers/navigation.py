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

        # PID control for turning
        if not self.is_centered(angle_to_marker_rad):
            turn_effort_raw = self.turning_pid.compute(angle_to_marker_rad)
            
            # Scale and clamp turn_effort (example: if PID output is +/- 1, scale to max_turning_speed)
            # Assuming PID output is already somewhat scaled or its Kp relates to desired PWM range.
            # Here, we directly use it but clamp to max_turning_speed and ensure min_effective_speed.
            direction = 1 if turn_effort_raw > 0 else -1 # Positive error means need to turn left (e.g. target is to the left)
            
            # Let's assume turn_effort_raw is a value that needs to be clamped by max_turning_speed
            # And also needs to respect min_effective_speed if it's non-zero
            abs_turn_effort = abs(turn_effort_raw) 
            
            # Clamp the magnitude of the turn speed
            clamped_effort = min(abs_turn_effort, self.max_turning_speed)
            
            # Ensure minimum effective speed if there's any effort, otherwise speed is 0
            effective_effort = 0
            if abs_turn_effort > 1e-3: # Only apply min speed if there's some error
                effective_effort = max(self.min_effective_speed, clamped_effort)
            
            final_turn_speed = direction * effective_effort
            
            logger.debug(f"Nav: Turning. Angle_err: {angle_to_marker_rad:.3f} rad, TurnPID_Raw: {turn_effort_raw:.2f}, FinalTurnSpeed: {final_turn_speed:.2f}")
            if final_turn_speed > 0: # Positive speed for turn_left in this convention
                logger.debug(f"Nav: Calling motor_controller.turn_left({abs(final_turn_speed):.2f})")
                self.motor_controller.turn_left(abs(final_turn_speed))
                logger.debug(f"Nav: Returned from motor_controller.turn_left({abs(final_turn_speed):.2f})")
            elif final_turn_speed < 0: # Negative speed for turn_right
                logger.debug(f"Nav: Calling motor_controller.turn_right({abs(final_turn_speed):.2f})")
                self.motor_controller.turn_right(abs(final_turn_speed))
                logger.debug(f"Nav: Returned from motor_controller.turn_right({abs(final_turn_speed):.2f})")
            else:
                # If final_turn_speed is 0 (e.g. error is tiny, PID output is zero, and min_effective_speed is also 0 or not applied)
                # We might still be 'navigating' if distance is not okay.
                # Or, if distance is also okay, the 'target reached' logic above should handle it.
                # If we want to explicitly stop turning if speed is zero:
                # self.motor_controller.stop() # This might be too aggressive, let distance PID handle it or rely on target_reached
                logger.debug(f"Nav: Turning. FinalTurnSpeed is 0. No motor turn command issued.")
                pass

        # PID control for distance (only if angle is okay)
        elif not self.is_at_distance(current_planar_distance_m):
            forward_effort_raw = self.distance_pid.compute(current_planar_distance_m)
            
            direction = 1 if forward_effort_raw < 0 else -1 # Negative error (current > target) means move forward
                                                            # Positive error (current < target) means move backward (or pid output needs inversion)
                                                            # Let's adjust: PID compute error as (setpoint - current).
                                                            # If current_planar_distance_m > self.target_approach_distance_m (too far), error is positive. Need to move forward.
                                                            # If current_planar_distance_m < self.target_approach_distance_m (too close), error is negative. Need to move backward.
                                                            # So, positive PID output should mean drive_forward.
            
            # Re-evaluating direction based on PID output meaning "correction"
            # If PID output (forward_effort_raw) is positive, it means "increase speed forward"
            # If PID output is negative, it means "increase speed backward" (or decrease speed forward)
            
            abs_forward_effort = abs(forward_effort_raw)
            clamped_effort = min(abs_forward_effort, self.max_forward_speed)
            
            effective_effort = 0
            if abs_forward_effort > 1e-3: # Only apply min speed if there's some error
                effective_effort = max(self.min_effective_speed, clamped_effort)

            final_forward_speed = 0
            if forward_effort_raw > 1e-3: # Drive forward
                final_forward_speed = effective_effort
            elif forward_effort_raw < -1e-3: # Drive backward
                final_forward_speed = -effective_effort
            
            logger.debug(f"Nav: Driving. Dist_err: {(current_planar_distance_m - self.target_approach_distance_m):.3f} m, DistPID_Raw: {forward_effort_raw:.2f}, FinalFwdSpeed: {final_forward_speed:.2f}")
            if final_forward_speed > 0: # Moving towards target (positive speed = forward)
                logger.debug(f"Nav: Calling motor_controller.drive_forward({abs(final_forward_speed):.2f})")
                self.motor_controller.drive_forward(abs(final_forward_speed))
                logger.debug(f"Nav: Returned from motor_controller.drive_forward({abs(final_forward_speed):.2f})")
            elif final_forward_speed < 0: # Moving away from target (negative speed = backward)
                logger.debug(f"Nav: Calling motor_controller.drive_backward({abs(final_forward_speed):.2f})")
                self.motor_controller.drive_backward(abs(final_forward_speed))
                logger.debug(f"Nav: Returned from motor_controller.drive_backward({abs(final_forward_speed):.2f})")
            else:
                logger.debug(f"Nav: Driving. FinalFwdSpeed is 0. No motor drive command issued.")
                pass
        
        return True # Actively navigating

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