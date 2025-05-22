import time
import logging
import math
import numpy as np # For normalize_angle
from typing import Optional, Tuple

from ..utils import config
# We expect MotorController to be passed in, but type hinting is good.
# from .motor import MotorController # This would be a circular import if NavigationController is imported by main.py

logger = logging.getLogger(__name__)

def normalize_angle(angle_rad: float) -> float:
    """Normalize an angle to the range [-pi, pi]."""
    # Simpler way to normalize to [-pi, pi]
    while angle_rad > np.pi:
        angle_rad -= 2 * np.pi
    while angle_rad < -np.pi:
        angle_rad += 2 * np.pi
    return angle_rad

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
        # Cap integral to prevent windup. Max output is usually clamped to max speed.
        self.max_integral = config.NAV_MAX_FORWARD_SPEED # Example cap, might need tuning or a separate config
        self.min_integral = -config.NAV_MAX_FORWARD_SPEED

    def set_gains(self, kp: float, ki: float, kd: float):
        """Dynamically update PID gains."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        logger.info(f"PID gains updated: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")

    def compute(self, current_value: float) -> float:
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0: 
            dt = 1e-6 

        error = self.setpoint - current_value
        
        self.integral += error * dt
        self.integral = max(self.min_integral, min(self.max_integral, self.integral))
        
        derivative = (error - self.previous_error) / dt if dt > 1e-9 else 0 # Avoid division by zero with tiny dt
        
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
        
        # PID for turning to face the target XY point
        self.point_turning_pid = PIDController(
            kp=config.OVERHEAD_NAV_TURNING_PID_KP,
            ki=config.OVERHEAD_NAV_TURNING_PID_KI,
            kd=config.OVERHEAD_NAV_TURNING_PID_KD,
            setpoint=0 
        )
        
        # PID for driving towards the target XY point
        self.xy_distance_pid = PIDController(
            kp=config.OVERHEAD_NAV_DISTANCE_PID_KP,
            ki=config.OVERHEAD_NAV_DISTANCE_PID_KI,
            kd=config.OVERHEAD_NAV_DISTANCE_PID_KD,
            setpoint=0 
        )
        
        # PID for aligning to the final target orientation
        self.final_orientation_pid = PIDController(
            kp=config.OVERHEAD_NAV_FINAL_ORIENTATION_PID_KP,
            ki=config.OVERHEAD_NAV_FINAL_ORIENTATION_PID_KI,
            kd=config.OVERHEAD_NAV_FINAL_ORIENTATION_PID_KD,
            setpoint=0 
        )
        
        # Store thresholds and parameters from config
        self.target_approach_distance_pixels = config.OVERHEAD_NAV_TARGET_APPROACH_DISTANCE_PIXELS
        self.distance_threshold_pixels = config.OVERHEAD_NAV_DISTANCE_THRESHOLD_PIXELS
        self.orientation_to_point_threshold_rad = config.OVERHEAD_NAV_ORIENTATION_THRESHOLD_RAD
        self.final_orientation_threshold_rad = config.OVERHEAD_NAV_FINAL_ANGULAR_THRESHOLD_RAD

        # Speed limits (these are absolute PWM values 0-100)
        self.max_turning_speed = config.NAV_MAX_TURNING_SPEED 
        self.max_forward_speed = config.NAV_MAX_FORWARD_SPEED 
        self.min_effective_speed = config.NAV_MIN_EFFECTIVE_SPEED # Lower bound of 75-100 range

        self.current_target_pixel_pose: Optional[Tuple[float, float, float]] = None
        
        # Remove old metric attributes
        # self._initial_turning_pid_gains = (config.NAV_TURNING_PID_KP, config.NAV_TURNING_PID_KI, config.NAV_TURNING_PID_KD)
        # self._initial_distance_pid_gains = (config.NAV_DISTANCE_PID_KP, config.NAV_DISTANCE_PID_KI, config.NAV_DISTANCE_PID_KD)
        # self.target_approach_distance_m = config.NAV_TARGET_APPROACH_DISTANCE_M
        # self.distance_threshold_m = config.NAV_DISTANCE_THRESHOLD_M
        # self.angular_threshold_rad = config.NAV_X_THRESHOLD_RAD

    def set_target(self, target_pixel_pose: Tuple[float, float, float]):
        if target_pixel_pose is None:
            logger.warning("NavigationController: set_target called with None. Clearing target.")
            self.clear_target()
            return

        self.current_target_pixel_pose = target_pixel_pose
        self.point_turning_pid.reset()
        self.xy_distance_pid.reset()
        self.final_orientation_pid.reset()
        
        target_x, target_y, target_theta_rad = target_pixel_pose
        logger.info(f"NavigationController: New target set - X:{target_x:.0f}px, Y:{target_y:.0f}px, Theta:{math.degrees(target_theta_rad):.1f}°")

    def clear_target(self):
        logger.info("NavigationController.clear_target(): Called. Stopping motors and resetting PIDs.")
        self.current_target_pixel_pose = None
        self.motor_controller.stop()
        self.point_turning_pid.reset()
        self.xy_distance_pid.reset()
        self.final_orientation_pid.reset()

    def _calculate_scaled_speed(self, effort: float, max_speed_limit: int, min_speed_limit: int) -> int:
        """Scales PID effort to motor speed, clamping and ensuring minimum effective speed.
        
        Args:
            effort: Raw output from PID controller. Can be positive or negative.
            max_speed_limit: The absolute maximum speed (e.g., 100).
            min_speed_limit: The minimum speed at which motors should run effectively (e.g., 75).
        Returns:
            Scaled motor command speed, including direction (positive/negative).
        """
        
        # Determine direction from effort
        direction = np.sign(effort)
        abs_effort = abs(effort)

        # At this point, Kp for PIDs need to be tuned such that `abs_effort` is somewhat
        # in a range that can be mapped to speeds. Let's assume Kp are tuned so `abs_effort` 
        # could be, for example, 0-100 or more.
        
        # Clamp the magnitude of the effort to the max_speed_limit (e.g. 0-100)
        # This acts as a primary governor on the PID output if Kp values are large.
        scaled_speed_magnitude = min(abs_effort, float(max_speed_limit))

        if scaled_speed_magnitude < 0.1: # If scaled effort is too small, consider it zero speed
            return 0
        
        # If there is some effort, ensure the speed is at least min_speed_limit,
        # but not exceeding max_speed_limit.
        # This maps the PID output to the [min_speed_limit, max_speed_limit] operational range.
        # Example: if min_speed_limit=75, max_speed_limit=100:
        #   A small PID output (but >0.1) will result in 75.
        #   A large PID output will result in 100.
        #   An intermediate PID output (e.g. if scaled_speed_magnitude was 80) would remain 80.
        
        # This logic ensures that if any movement is intended, it's at least min_speed_limit.
        # And it also respects the max_speed_limit.
        final_speed_magnitude = max(float(min_speed_limit), scaled_speed_magnitude)
        final_speed_magnitude = min(final_speed_magnitude, float(max_speed_limit)) # Redundant if scaled_speed_magnitude was already capped, but safe.
            
        return int(direction * final_speed_magnitude)

    def navigate(self, current_pixel_pose: Tuple[float, float, float]) -> bool:
        if not self.current_target_pixel_pose or current_pixel_pose is None:
            # self.motor_controller.stop() # Ensure motors are stopped if no valid state, handled by caller or periodic check
            return False # Cannot navigate without target or current pose

        current_x_px, current_y_px, current_theta_rad = current_pixel_pose
        target_x_px, target_y_px, target_theta_rad = self.current_target_pixel_pose

        # --- Calculate errors for XY navigation ---
        dx = target_x_px - current_x_px
        dy = target_y_px - current_y_px
        distance_to_target_xy_pixels = math.sqrt(dx*dx + dy*dy)
        angle_to_target_point_rad = math.atan2(dy, dx) 
        angle_to_point_error_rad = normalize_angle(angle_to_target_point_rad - current_theta_rad)

        # logger.debug(f"Nav Info: Curr(X:{current_x_px:.0f},Y:{current_y_px:.0f},T:{math.degrees(current_theta_rad):.1f}), Target(X:{target_x_px:.0f},Y:{target_y_px:.0f},T:{math.degrees(target_theta_rad):.1f})")
        # logger.debug(f"Nav Calc: DistToXY:{distance_to_target_xy_pixels:.1f}px, AngleToTargetPt:{math.degrees(angle_to_target_point_rad):.1f}°, AngleToPtError:{math.degrees(angle_to_point_error_rad):.1f}°")

        # --- State 1: Align to Target Point XY ---
        if abs(angle_to_point_error_rad) > self.orientation_to_point_threshold_rad:
            # PID error: setpoint (0) - current_error. If angle_to_point_error_rad is positive (target is CCW from robot heading),
            # then -angle_to_point_error_rad is negative. PID output should be negative for CCW turn (convention in motor controller turn_left/right).
            # Let's adjust PID interpretation: Positive PID output for turn_left (CCW), Negative for turn_right (CW).
            # So, if error is positive (target is CCW), we want positive PID output.
            turn_effort = self.point_turning_pid.compute(angle_to_point_error_rad) # Pass error directly, PID handles setpoint=0 internally
            
            turn_speed = self._calculate_scaled_speed(turn_effort, self.max_turning_speed, self.min_effective_speed)
            
            logger.debug(f"Nav State 1 (AlignToPoint): AngleToPtError:{math.degrees(angle_to_point_error_rad):.1f}°, Effort:{turn_effort:.2f}, TurnSpeed:{turn_speed}")
            if turn_speed == 0 and abs(angle_to_point_error_rad) > self.orientation_to_point_threshold_rad:
                 # If error still exists but PID output is zero (e.g. stuck in deadband or Kp too small), force minimal turn
                logger.debug("Nav State 1: Forcing minimal turn due to zero speed with existing error.")
                turn_speed = int(np.sign(angle_to_point_error_rad) * self.min_effective_speed)

            if turn_speed > 0:
                self.motor_controller.turn_left(abs(turn_speed))
            elif turn_speed < 0:
                self.motor_controller.turn_right(abs(turn_speed))
            else:
                self.motor_controller.stop() # No turn needed or PID output too small
            return True # Still navigating (turning)

        # --- State 2: Drive to Target Point XY ---
        # Error for distance PID: current_distance_pixels - target_approach_distance_pixels.
        # We want this error to be 0.
        # If current > approach (too far), error is positive. PID (setpoint 0) gets negative input, should output positive (forward).
        distance_error_for_pid = distance_to_target_xy_pixels - self.target_approach_distance_pixels

        # Only drive if we are further than the *arrival* threshold (distance_threshold_pixels)
        # The PID uses target_approach_distance_pixels as its conceptual setpoint via the error term.
        if distance_to_target_xy_pixels > self.distance_threshold_pixels: # Check if we are outside the acceptable ARRIVAL distance threshold
            # If distance_error_for_pid is positive (too far), self.xy_distance_pid.compute will use (0 - positive_error) = negative value.
            # We want positive PID output to drive forward. So, pass negative of the error.
            drive_effort = self.xy_distance_pid.compute(-distance_error_for_pid) 
            
            drive_speed = self._calculate_scaled_speed(drive_effort, self.max_forward_speed, self.min_effective_speed)

            logger.debug(f"Nav State 2 (DriveToXY): DistToXY:{distance_to_target_xy_pixels:.1f}px, ApproachDist:{self.target_approach_distance_pixels:.1f}px, DistErrorPID:{distance_error_for_pid:.1f}px, Effort:{drive_effort:.2f}, DriveSpeed:{drive_speed}")
            
            if drive_speed > 0:
                self.motor_controller.drive_forward(abs(drive_speed))
            elif drive_speed < 0: # Overshot, or PID tuned to allow reversal
                self.motor_controller.drive_backward(abs(drive_speed))
            else:
                self.motor_controller.stop()
            return True # Still navigating (driving)

        # --- State 3: Align to Final Target Orientation ---
        final_orientation_error_rad = normalize_angle(target_theta_rad - current_theta_rad)
        # logger.debug(f"Nav State 3 Check (FinalOrient): FinalOrientError:{math.degrees(final_orientation_error_rad):.1f}°")

        if abs(final_orientation_error_rad) > self.final_orientation_threshold_rad:
            # Similar to point turning: Positive error (target_theta is CCW from current_theta) means turn left (positive PID output)
            final_turn_effort = self.final_orientation_pid.compute(final_orientation_error_rad)
            final_turn_speed = self._calculate_scaled_speed(final_turn_effort, self.max_turning_speed, self.min_effective_speed)
            
            logger.debug(f"Nav State 3 (FinalOrient): Error:{math.degrees(final_orientation_error_rad):.1f}°, Effort:{final_turn_effort:.2f}, Speed:{final_turn_speed}")
            if final_turn_speed == 0 and abs(final_orientation_error_rad) > self.final_orientation_threshold_rad:
                logger.debug("Nav State 3: Forcing minimal turn for final orientation.")
                final_turn_speed = int(np.sign(final_orientation_error_rad) * self.min_effective_speed)

            if final_turn_speed > 0:
                self.motor_controller.turn_left(abs(final_turn_speed))
            elif final_turn_speed < 0:
                self.motor_controller.turn_right(abs(final_turn_speed))
            else:
                self.motor_controller.stop()
            return True # Still navigating (final turning)
        
        # --- Target Reached (All conditions met) --- 
        logger.info(f"NavigationController: Target Reached! XY_Dist:{distance_to_target_xy_pixels:.1f}px (Threshold:{self.distance_threshold_pixels:.1f}px), FinalAngleError:{math.degrees(final_orientation_error_rad):.1f}° (Threshold:{math.degrees(self.final_orientation_threshold_rad):.1f}°)")
        self.motor_controller.stop()
        return False # Navigation complete

    def stop_navigation(self):
        self.clear_target()
        logger.info("NavigationController: Navigation explicitly stopped by stop_navigation().")

    def update_pid_gains(self, pid_name: str, kp: float, ki: float, kd: float):
        """Updates gains for a specified PID controller."""
        pid_to_update = None
        if pid_name == "point_turning":
            pid_to_update = self.point_turning_pid
        elif pid_name == "xy_distance":
            pid_to_update = self.xy_distance_pid
        elif pid_name == "final_orientation":
            pid_to_update = self.final_orientation_pid
        
        if pid_to_update:
            pid_to_update.set_gains(kp, ki, kd)
            # logger.info(f"NavigationController: {pid_name} PID gains updated to Kp={kp}, Ki={ki}, Kd={kd}") # PIDController.set_gains already logs
        else:
            logger.warning(f"NavigationController: Unknown PID name '{pid_name}' for updating gains.")

    # Remove old PID gain setters or mark as deprecated if client compatibility is briefly needed
    # def update_turning_pid_gains(self, kp: float, ki: float, kd: float):
    #     logger.warning("update_turning_pid_gains is deprecated. Use update_pid_gains('point_turning', ...)")
    #     self.update_pid_gains("point_turning", kp, ki, kd)

    # def update_distance_pid_gains(self, kp: float, ki: float, kd: float):
    #     logger.warning("update_distance_pid_gains is deprecated. Use update_pid_gains('xy_distance', ...)")
    #     self.update_pid_gains("xy_distance", kp, ki, kd) 