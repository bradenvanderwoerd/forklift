import time
import logging
import math
import numpy as np # For normalize_angle
from typing import Optional, Tuple

from ..utils import config
# Import MotorController for type hinting, assuming it won't cause circular import if main.py structure is respected.
# from .motor import MotorController # Keep commented if direct import causes issues.

logger = logging.getLogger(__name__)

def normalize_angle(angle_rad: float) -> float:
    """Normalizes an angle in radians to the range [-pi, pi].

    Args:
        angle_rad: The angle in radians to normalize.

    Returns:
        The normalized angle in radians within the range [-pi, pi].
    """
    while angle_rad > np.pi:
        angle_rad -= 2 * np.pi
    while angle_rad < -np.pi:
        angle_rad += 2 * np.pi
    return angle_rad

class PIDController:
    """A Proportional-Integral-Derivative (PID) controller.

    This controller computes an output value designed to drive a system towards a
    desired setpoint by minimizing the error between the setpoint and a current value.
    It includes integral windup protection by capping the integral term.
    """
    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = 0):
        """Initializes the PIDController.

        Args:
            kp: The Proportional gain. This determines the influence of the current error.
            ki: The Integral gain. This determines the influence of the accumulated past errors.
            kd: The Derivative gain. This determines the influence of the rate of change of the error.
            setpoint: The target value the controller aims to achieve.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        
        self.previous_error: float = 0.0
        self.integral: float = 0.0
        self.last_time: float = time.time()
        
        # Cap integral term to prevent integral windup.
        # Uses NAV_MAX_FORWARD_SPEED as a proxy for max PID output, might need specific tuning.
        self.max_integral: float = float(config.NAV_MAX_FORWARD_SPEED) 
        self.min_integral: float = -float(config.NAV_MAX_FORWARD_SPEED)

    def set_gains(self, kp: float, ki: float, kd: float):
        """Dynamically updates the PID gains.

        Args:
            kp: New Proportional gain.
            ki: New Integral gain.
            kd: New Derivative gain.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        logger.info(f"PID gains updated: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}")

    def compute(self, current_value: float) -> float:
        """Computes the PID output based on the current value and setpoint.

        Args:
            current_value: The current measured value of the system.

        Returns:
            The computed PID output value.
        """
        current_time = time.time()
        dt = current_time - self.last_time
        # Avoid division by zero or excessively small dt if time hasn't changed significantly.
        if dt <= 1e-6: 
            dt = 1e-6 

        error = self.setpoint - current_value
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(self.min_integral, min(self.max_integral, self.integral))
        
        # Derivative term (handles dt near zero)
        derivative = (error - self.previous_error) / dt if dt > 1e-9 else 0.0
        
        # PID output formula
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        self.previous_error = error
        self.last_time = current_time
        
        return output

    def reset(self):
        """Resets the PID controller's internal state (integral and previous error).
        Useful when starting a new control sequence or after a significant change.
        """
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

class NavigationController:
    """Manages autonomous navigation of the robot to a target pose in pixel space.

    This controller uses a state machine and three PID controllers to guide the robot:
    1.  Align to Target Point (XY): Turns the robot to face the target (x, y) coordinates.
    2.  Drive to Target Point (XY): Drives the robot towards the (x, y) coordinates,
        making corrective turns as needed.
    3.  Align to Final Orientation: Once at the (x, y) coordinates, turns the robot
        to match the final target orientation (theta).

    It relies on an external MotorController for actual motor commands and current pose
    updates from an overhead localization system.
    """
    def __init__(self, motor_controller):
        """Initializes the NavigationController.

        Args:
            motor_controller: An instance of MotorController for executing movement commands.
        """
        self.motor_controller = motor_controller # Instance of MotorController
        
        # PID for turning to face the target XY point (State 1).
        # Error is the angular difference between robot's heading and direction to target XY.
        self.point_turning_pid = PIDController(
            kp=config.OVERHEAD_NAV_TURNING_PID_KP,
            ki=config.OVERHEAD_NAV_TURNING_PID_KI,
            kd=config.OVERHEAD_NAV_TURNING_PID_KD,
            setpoint=0 # Target error is 0 degrees/radians.
        )
        
        # PID for driving towards the target XY point (State 2).
        # Error is the remaining distance to the target XY.
        self.xy_distance_pid = PIDController(
            kp=config.OVERHEAD_NAV_DISTANCE_PID_KP,
            ki=config.OVERHEAD_NAV_DISTANCE_PID_KI,
            kd=config.OVERHEAD_NAV_DISTANCE_PID_KD,
            setpoint=0 # Target error is 0 pixels (or target_approach_distance_pixels indirectly).
        )
        
        # PID for aligning to the final target orientation (theta) (State 3).
        # Error is the angular difference between robot's heading and target_theta.
        self.final_orientation_pid = PIDController(
            kp=config.OVERHEAD_NAV_FINAL_ORIENTATION_PID_KP,
            ki=config.OVERHEAD_NAV_FINAL_ORIENTATION_PID_KI,
            kd=config.OVERHEAD_NAV_FINAL_ORIENTATION_PID_KD,
            setpoint=0 # Target error is 0 degrees/radians.
        )
        
        # Navigation thresholds and parameters loaded from config.py
        self.target_approach_distance_pixels = config.OVERHEAD_NAV_TARGET_APPROACH_DISTANCE_PIXELS
        self.distance_threshold_pixels = config.OVERHEAD_NAV_DISTANCE_THRESHOLD_PIXELS
        self.orientation_to_point_threshold_rad = config.OVERHEAD_NAV_ORIENTATION_THRESHOLD_RAD
        self.final_orientation_threshold_rad = config.OVERHEAD_NAV_FINAL_ANGULAR_THRESHOLD_RAD
        
        # Speed limits for motor commands (0-100 scale)
        self.max_turning_speed = config.NAV_MAX_TURNING_SPEED 
        self.max_forward_speed = config.NAV_MAX_FORWARD_SPEED 
        self.min_effective_speed = config.NAV_MIN_EFFECTIVE_SPEED

        self.current_target_pixel_pose: Optional[Tuple[float, float, float]] = None
        
        # Obsolete metric navigation attributes (commented out in config, removed here for clarity)
        # self._initial_turning_pid_gains = ... (etc.)

    def set_target(self, target_pixel_pose: Tuple[float, float, float]):
        """Sets a new navigation target and resets PID controllers.

        Args:
            target_pixel_pose: A tuple (x, y, theta_rad) representing the target pose
                               in the overhead camera's pixel coordinate system.
                               theta_rad is the desired final orientation in radians.
        """
        if target_pixel_pose is None:
            logger.warning("NavigationController: set_target called with None. Clearing current target.")
            self.clear_target()
            return

        self.current_target_pixel_pose = target_pixel_pose
        # Reset PIDs for the new target.
        self.point_turning_pid.reset()
        self.xy_distance_pid.reset()
        self.final_orientation_pid.reset()
        
        target_x, target_y, target_theta_rad = target_pixel_pose
        logger.info(f"NavigationController: New target set - X:{target_x:.0f}px, Y:{target_y:.0f}px, ThetaRad:{target_theta_rad:.2f} (Deg:{math.degrees(target_theta_rad):.1f}°)")

    def clear_target(self):
        """Clears the current navigation target, stops motors, and resets PID controllers."""
        logger.info("NavigationController.clear_target(): Called. Stopping motors and resetting PIDs.")
        self.current_target_pixel_pose = None
        self.motor_controller.stop()
        self.point_turning_pid.reset()
        self.xy_distance_pid.reset()
        self.final_orientation_pid.reset()

    def _calculate_scaled_speed(self, effort: float, max_speed_limit: int, min_speed_limit: int, context: str = "-") -> int:
        """Scales PID effort to a motor speed command (-100 to 100).

        Ensures that if any movement is intended (effort is non-zero), the speed magnitude
        is at least `min_speed_limit` to overcome motor stiction, but not exceeding
        `max_speed_limit`.

        Args:
            effort: Raw output from a PID controller. Positive or negative.
            max_speed_limit: The absolute maximum speed for this action (e.g., 100).
            min_speed_limit: The minimum effective speed (e.g., 75 or 80).
            context: Optional string for logging to identify the call site.

        Returns:
            A scaled motor speed command, including direction (positive/negative).
        """
        # Detailed logging for debugging PID scaling (can be enabled by uncommenting lines below)
        logger.debug(f"_calc_speed ({context}): InEffort={effort:.2f}, MaxLim={max_speed_limit}, MinLim={min_speed_limit}")

        direction = np.sign(effort)
        abs_effort = abs(effort)

        # Clamp the magnitude of the effort to the max_speed_limit.
        scaled_speed_magnitude = min(abs_effort, float(max_speed_limit))
        logger.debug(f"_calc_speed ({context}): AbsEff={abs_effort:.2f}, ScaledMag={scaled_speed_magnitude:.2f} (after min(abs_effort, max_limit))")

        if scaled_speed_magnitude < 0.1: # If scaled effort is too small, consider it zero speed.
            logger.debug(f"_calc_speed ({context}): ScaledMag < 0.1, returning 0")
            return 0
        
        # Ensure speed is at least min_speed_limit, but not exceeding max_speed_limit.
        final_speed_magnitude = max(float(min_speed_limit), scaled_speed_magnitude)
        logger.debug(f"_calc_speed ({context}): FinalMag={final_speed_magnitude:.2f} (after max(min_limit, scaled_mag))")

        final_speed_magnitude = min(final_speed_magnitude, float(max_speed_limit)) # Re-clamp to max (mostly redundant but safe).
        logger.debug(f"_calc_speed ({context}): FinalMag={final_speed_magnitude:.2f} (after min(final_mag, max_limit))")
            
        result_speed = int(direction * final_speed_magnitude)
        logger.debug(f"_calc_speed ({context}): Returning speed: {result_speed}")
        return result_speed

    def navigate(self, current_pixel_pose: Tuple[float, float, float]) -> bool:
        """Executes one step of the navigation state machine.

        Calculates errors based on the current pose and target, computes PID efforts,
        and commands the motors. This method should be called repeatedly in a loop.

        Args:
            current_pixel_pose: The robot's current pose (x_px, y_px, theta_rad)
                                from the overhead localization system.

        Returns:
            True if the navigation target has been reached (all states complete).
            False if navigation is still in progress.
        """
        if not self.current_target_pixel_pose or current_pixel_pose is None:
            # logger.warning("NavigationController.navigate(): Called without target or current pose.") # Can be noisy
            # self.motor_controller.stop() # Stop motors if state is invalid. Caller should handle this.
            return False # Cannot navigate.

        current_x_px, current_y_px, current_theta_rad = current_pixel_pose
        target_x_px, target_y_px, target_theta_rad = self.current_target_pixel_pose

        # --- Calculate current errors relative to the target XY point ---
        dx = target_x_px - current_x_px  # Pixel difference in X
        dy = target_y_px - current_y_px  # Pixel difference in Y
        distance_to_target_xy_pixels = math.sqrt(dx*dx + dy*dy)
        
        # Angle from robot's current X-axis to the target XY point.
        angle_to_target_point_rad = math.atan2(dy, dx) 
        # Error in robot's current heading vs the direct line to the target XY point.
        # Positive error means robot needs to turn Clockwise (to the right) to face the point.
        # Negative error means robot needs to turn Counter-Clockwise (to the left).
        angle_to_point_error_rad = normalize_angle(angle_to_target_point_rad - current_theta_rad)

        # --- Navigation State Machine ---

        # STATE 1: Align robot's heading towards the target XY point.
        if abs(angle_to_point_error_rad) > self.orientation_to_point_threshold_rad:
            # PID input: current angular error to the point.
            # PID setpoint: 0 (we want zero angular error to the point).
            # PID output (turn_effort): Positive if robot needs to turn CCW (left), negative for CW (right).
            # MotorController.move() turn_component: Positive for CCW (left), negative for CW (right).
            # So, -turn_effort is NOT needed here if PID output convention matches motor_controller.move convention.
            # Let's re-verify PID output: error = setpoint(0) - current_value(angle_to_point_error_rad).
            # If angle_to_point_error_rad is positive (robot is CCW of target line, needs to turn CW/right),
            # then error is negative. PID output (turn_effort) is negative. This matches motor_controller.move for CW/right.
            self.point_turning_pid.setpoint = 0 
            turn_effort = self.point_turning_pid.compute(angle_to_point_error_rad)
            # Correction: The comment above was slightly off. If error = setpoint - current,
            # and current = angle_to_point_error_rad.
            # If angle_to_point_error_rad is POSITIVE (e.g. robot is at 0 deg, target line is at +30 deg, error_to_pt is +30 deg), 
            # this means robot needs to turn CCW (left) to reduce this positive error.
            # PID error = 0 - (+30) = -30. PID output (turn_effort) will be negative (e.g. -Kp*30).
            # MotorController.move() expects POSITIVE turn_component for CCW (left).
            # Therefore, we MUST negate the PID output: `_calculate_scaled_speed(-turn_effort, ...)`
            turn_speed = self._calculate_scaled_speed(-turn_effort, self.max_turning_speed, self.min_effective_speed, context="turn_to_point")
            self.motor_controller.move(forward_component=0, turn_component=turn_speed)
            logger.info(f"State: ALIGNING_TO_POINT. ErrRad:{angle_to_point_error_rad:.2f} (Deg:{math.degrees(angle_to_point_error_rad):.1f}), Effort(raw):{turn_effort:.2f}, ScaledSpeed:{turn_speed}")
            return False # Still aligning

        # STATE 2: Drive towards the target XY point.
        # This state is reached if the robot is generally facing the target point.
        if distance_to_target_xy_pixels > self.distance_threshold_pixels:
            # PID input: current distance to target XY.
            # PID setpoint: 0 (we want to minimize distance to the target XY).
            # PID output (forward_effort): If distance is positive, error = 0 - distance = negative.
            # So, forward_effort will be negative if further away.
            # MotorController.move() forward_component: Positive for forward.
            # Therefore, we MUST negate the PID output: `_calculate_scaled_speed(-forward_effort, ...)`
            self.xy_distance_pid.setpoint = 0 # Target is to reduce distance to 0 (or approach_distance).
            forward_effort = self.xy_distance_pid.compute(distance_to_target_xy_pixels)
            forward_speed = self._calculate_scaled_speed(-forward_effort, self.max_forward_speed, self.min_effective_speed, context="drive_fwd")

            # Corrective turning while driving forward (uses the same point_turning_pid).
            # Error and effort logic is the same as in State 1 for turning.
            self.point_turning_pid.setpoint = 0
            correction_turn_effort = self.point_turning_pid.compute(angle_to_point_error_rad)
            correction_turn_speed = self._calculate_scaled_speed(-correction_turn_effort, self.max_turning_speed, self.min_effective_speed, context="turn_corrective")

            self.motor_controller.move(forward_component=forward_speed, turn_component=correction_turn_speed)
            logger.info(f"State: DRIVING_TO_XY. Dist:{distance_to_target_xy_pixels:.1f}px, FwdEffort:{forward_effort:.2f}, FwdSpeed:{forward_speed}. AngleErr:{math.degrees(angle_to_point_error_rad):.1f}°, TurnEffort:{correction_turn_effort:.2f}, TurnSpeed:{correction_turn_speed}")
            return False # Still driving

        # STATE 3: Align to the final target orientation (theta).
        # This state is reached if the robot is at the target XY coordinates.
        final_angle_error_rad = normalize_angle(target_theta_rad - current_theta_rad)
        if abs(final_angle_error_rad) > self.final_orientation_threshold_rad:
            # PID input: current final angular error.
            # PID setpoint: 0 (we want zero final angular error).
            # Effort and negation logic is the same as State 1 for turning.
            self.final_orientation_pid.setpoint = 0
            final_turn_effort = self.final_orientation_pid.compute(final_angle_error_rad)
            final_turn_speed = self._calculate_scaled_speed(-final_turn_effort, self.max_turning_speed, self.min_effective_speed, context="turn_final_orient")
            self.motor_controller.move(forward_component=0, turn_component=final_turn_speed)
            logger.info(f"State: FINAL_ALIGNMENT. TargetAngle:{math.degrees(target_theta_rad):.1f}°, CurrAngle:{math.degrees(current_theta_rad):.1f}°, FinalAngleError:{math.degrees(final_angle_error_rad):.1f}°, Effort:{final_turn_effort:.2f}, ScaledSpeed:{final_turn_speed}")
            return False # Still aligning to final orientation
        
        # ALL STATES COMPLETE: Target Reached.
        logger.info(f"NavigationController: Target Reached! XY_Dist:{distance_to_target_xy_pixels:.1f}px (Thresh:{self.distance_threshold_pixels:.1f}px), FinalAngleError:{math.degrees(final_angle_error_rad):.1f}° (Thresh:{math.degrees(self.final_orientation_threshold_rad):.1f}°)")
        self.motor_controller.stop()
        return True # Navigation complete.

    def stop_navigation(self):
        """Explicitly stops any ongoing navigation and clears the target."""
        self.clear_target() # This also stops motors and resets PIDs.
        logger.info("NavigationController: Navigation explicitly stopped by stop_navigation().")

    def update_pid_gains(self, pid_name: str, kp: float, ki: float, kd: float):
        """Updates gains for a specified PID controller.

        Args:
            pid_name: The name of the PID controller to update.
                      Valid names: "point_turning", "xy_distance", "final_orientation".
            kp: New Proportional gain.
            ki: New Integral gain.
            kd: New Derivative gain.
        """
        pid_to_update: Optional[PIDController] = None
        if pid_name == "point_turning":
            pid_to_update = self.point_turning_pid
        elif pid_name == "xy_distance":
            pid_to_update = self.xy_distance_pid
        elif pid_name == "final_orientation":
            pid_to_update = self.final_orientation_pid
        
        if pid_to_update:
            pid_to_update.set_gains(kp, ki, kd)
            # PIDController.set_gains() already logs the update.
        else:
            logger.warning(f"NavigationController: Unknown PID name '{pid_name}' for updating gains. Valid names are 'point_turning', 'xy_distance', 'final_orientation'.")

    # Deprecated PID gain setters have been removed.
    # Use `update_pid_gains(pid_name, kp, ki, kd)` instead.

    # Remove old PID gain setters or mark as deprecated if client compatibility is briefly needed
    # def update_turning_pid_gains(self, kp: float, ki: float, kd: float):
    #     logger.warning("update_turning_pid_gains is deprecated. Use update_pid_gains('point_turning', ...)")
    #     self.update_pid_gains("point_turning", kp, ki, kd)

    # def update_distance_pid_gains(self, kp: float, ki: float, kd: float):
    #     logger.warning("update_distance_pid_gains is deprecated. Use update_pid_gains('xy_distance', ...)")
    #     self.update_pid_gains("xy_distance", kp, ki, kd) 