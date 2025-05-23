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
        self.pid_output_limits = (-100, 100) # Standard PID output limits, actual speed uses min/max config

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

    def _calculate_scaled_speed(self, effort: float, max_speed_limit: int, min_speed_limit: int, context: str = "-") -> int:
        """Scales PID effort to a motor speed, applying min/max limits."""
        # Detailed logging for debugging PID scaling
        # if context == "turn_to_point" or context == "turn_final_orient": # Log only for turning for now to reduce noise
        logger.debug(f"_calc_speed ({context}): InEffort={effort:.2f}, MaxLim={max_speed_limit}, MinLim={min_speed_limit}")

        direction = np.sign(effort)
        abs_effort = abs(effort)

        # Clamp the magnitude of the effort to the max_speed_limit (e.g. 0-100)
        scaled_speed_magnitude = min(abs_effort, float(max_speed_limit))
        # if context == "turn_to_point" or context == "turn_final_orient":
        logger.debug(f"_calc_speed ({context}): AbsEff={abs_effort:.2f}, ScaledMag={scaled_speed_magnitude:.2f} (after min(abs_effort, max_limit))")

        if scaled_speed_magnitude < 0.1: # If scaled effort is too small, consider it zero speed
            # if context == "turn_to_point" or context == "turn_final_orient":
            logger.debug(f"_calc_speed ({context}): ScaledMag < 0.1, returning 0")
            return 0
        
        # If there is some effort, ensure the speed is at least min_speed_limit,
        # but not exceeding max_speed_limit.
        final_speed_magnitude = max(float(min_speed_limit), scaled_speed_magnitude)
        # if context == "turn_to_point" or context == "turn_final_orient":
        logger.debug(f"_calc_speed ({context}): FinalMag={final_speed_magnitude:.2f} (after max(min_limit, scaled_mag))")

        # Redundant if scaled_speed_magnitude was already capped by max_speed_limit, but safe.
        final_speed_magnitude = min(final_speed_magnitude, float(max_speed_limit)) 
        # if context == "turn_to_point" or context == "turn_final_orient":
        logger.debug(f"_calc_speed ({context}): FinalMag={final_speed_magnitude:.2f} (after min(final_mag, max_limit))")
            
        result_speed = int(direction * final_speed_magnitude)
        # if context == "turn_to_point" or context == "turn_final_orient":
        logger.debug(f"_calc_speed ({context}): Returning speed: {result_speed}")
        return result_speed

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
            # PID error: setpoint (0) - current_error. If angle_to_point_error_rad is positive (robot is CCW, needs to turn CW), effort will be negative.
            self.point_turning_pid.setpoint = 0 
            turn_effort = self.point_turning_pid.compute(angle_to_point_error_rad) 
            turn_speed = self._calculate_scaled_speed(-turn_effort, self.max_turning_speed, self.min_effective_speed, context="turn_to_point")
            self.motor_controller.move(forward_component=0, turn_component=turn_speed)
            logger.info(f"State: ALIGNING_TO_POINT. ErrRad:{angle_to_point_error_rad:.2f} (Deg:{math.degrees(angle_to_point_error_rad):.1f}), Effort(raw):{turn_effort:.2f}, ScaledSpeed:{turn_speed}")
            return False # Still aligning

        # --- State 2: Drive to Target Point XY ---
        # This state is reached if the robot is generally facing the target point.
        if distance_to_target_xy_pixels > self.distance_threshold_pixels:
            # PID for forward movement (error is positive if target is further away)
            forward_effort = self.xy_distance_pid.compute(distance_to_target_xy_pixels) # PID setpoint is 0, error = 0 - dist = -dist
                                                                                        # To move forward, effort should be positive if distance is positive.
                                                                                        # Let's make PID input -distance so error = 0 - (-dist) = +dist
            
            # Let's adjust xy_distance_pid input: setpoint is target_approach_distance_pixels, current value is distance_to_target_xy_pixels
            # Error = setpoint - current_value. If current > setpoint, error is negative (need to move backward, or reduce speed if overshot)
            # This PID should output positive to go forward if current_value < setpoint.
            # For simplicity, let's keep setpoint = 0 for distance PID.
            # error for distance PID = 0 - distance_to_target_xy_pixels.
            # If distance is positive, error is negative. PID output (forward_effort) is negative.
            # To move forward, motor_controller.move() needs positive forward_component. So, negate forward_effort.
            
            # Re-thinking distance PID:
            # Setpoint is effectively self.target_approach_distance_pixels (or 0 if we want to reach exactly the point).
            # Let's use current distance_to_target_xy_pixels as the value to reduce to 0 (or to target_approach_distance_pixels).
            # If self.xy_distance_pid.setpoint = 0:
            #   error = 0 - distance_to_target_xy_pixels. If distance is 100, error = -100. Output will be negative.
            #   We need positive forward speed. So, forward_speed = _calculate_scaled_speed(-output, ...)
            forward_speed = self._calculate_scaled_speed(-forward_effort, self.max_forward_speed, self.min_effective_speed)

            # Corrective turning to face the target point (if needed while moving forward)
            self.point_turning_pid.setpoint = 0
            correction_turn_effort = self.point_turning_pid.compute(angle_to_point_error_rad)
            # Use a slightly lower max speed for corrective turning, and potentially higher min if it's too sluggish.
            # For now, use same limits. Negate effort for motor controller.
            correction_turn_speed = self._calculate_scaled_speed(-correction_turn_effort, self.max_turning_speed, self.min_effective_speed, context="turn_corrective")

            self.motor_controller.move(forward_component=forward_speed, turn_component=correction_turn_speed)
            logger.info(f"State: DRIVING_TO_XY. Dist:{distance_to_target_xy_pixels:.1f}px, FwdEffort:{forward_effort:.2f}, FwdSpeed:{forward_speed}. AngleErr:{math.degrees(angle_to_point_error_rad):.1f}°, TurnEffort:{correction_turn_effort:.2f}, TurnSpeed:{correction_turn_speed}")
            return False

        # --- State 3: Align to Final Target Orientation ---
        # This state is reached if XY position is close enough.
        final_angle_error_rad = normalize_angle(target_theta_rad - current_theta_rad)
        if abs(final_angle_error_rad) > self.final_orientation_threshold_rad:
            # PID error: setpoint (0) - current_error.
            self.final_orientation_pid.setpoint = 0
            final_turn_effort = self.final_orientation_pid.compute(final_angle_error_rad)
            final_turn_speed = self._calculate_scaled_speed(-final_turn_effort, self.max_turning_speed, self.min_effective_speed, context="turn_final_orient")
            self.motor_controller.move(forward_component=0, turn_component=final_turn_speed)
            logger.info(f"State: FINAL_ALIGNMENT. TargetAngle:{math.degrees(target_theta_rad):.1f}°, CurrentAngle:{math.degrees(current_theta_rad):.1f}°, FinalAngleError:{math.degrees(final_angle_error_rad):.1f}°, Effort:{final_turn_effort:.2f}, ScaledSpeed:{final_turn_speed}")
            return False # Still aligning
        
        # --- Target Reached (All conditions met) --- 
        logger.info(f"NavigationController: Target Reached! XY_Dist:{distance_to_target_xy_pixels:.1f}px (Threshold:{self.distance_threshold_pixels:.1f}px), FinalAngleError:{math.degrees(final_angle_error_rad):.1f}° (Threshold:{math.degrees(self.final_orientation_threshold_rad):.1f}°)")
        self.motor_controller.stop()
        return True # Navigation complete

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