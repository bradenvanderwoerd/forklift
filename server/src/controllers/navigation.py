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
        # For turning, the setpoint is 0 (we want the marker's x-coordinate to be 0)
        self.turning_pid = PIDController(
            kp=config.NAV_TURNING_PID_KP,
            ki=config.NAV_TURNING_PID_KI,
            kd=config.NAV_TURNING_PID_KD,
            setpoint=0
        )
        
        # For distance, the setpoint is the target approach distance
        # The PID will compute an output to make current_distance approach setpoint
        self.distance_pid = PIDController(
            kp=config.NAV_DISTANCE_PID_KP,
            ki=config.NAV_DISTANCE_PID_KI,
            kd=config.NAV_DISTANCE_PID_KD,
            setpoint=config.NAV_TARGET_APPROACH_DISTANCE_M
        )
        
        # Store initial config values for reference or potential revert
        self._initial_turning_pid_gains = (config.NAV_TURNING_PID_KP, config.NAV_TURNING_PID_KI, config.NAV_TURNING_PID_KD)
        self._initial_distance_pid_gains = (config.NAV_DISTANCE_PID_KP, config.NAV_DISTANCE_PID_KI, config.NAV_DISTANCE_PID_KD)

        self.target_approach_distance_m = config.NAV_TARGET_APPROACH_DISTANCE_M
        self.distance_threshold_m = config.NAV_DISTANCE_THRESHOLD_M
        self.x_threshold_m = config.NAV_X_THRESHOLD_M

        self.max_turning_speed = config.NAV_MAX_TURNING_SPEED
        self.max_forward_speed = config.NAV_MAX_FORWARD_SPEED
        self.min_effective_speed = config.NAV_MIN_EFFECTIVE_SPEED

        self.current_target_pose = None # To store tvec, rvec of the current target

    def set_target(self, tvec, rvec):
        """Set the current navigation target pose and reset PIDs."""
        self.current_target_pose = (tvec, rvec)
        self.turning_pid.reset()
        self.distance_pid.reset()
        logger.info(f"NavigationController: New target set. tvec: {tvec.flatten().round(3)}")

    def clear_target(self):
        self.current_target_pose = None
        self.motor_controller.stop()
        logger.info("NavigationController: Target cleared, stopping motors.")

    def navigate(self) -> bool:
        """ 
        Computes and applies motor commands to navigate towards the current_target_pose.
        Returns True if navigation logic is actively trying to move, False if no target or at target.
        """
        if self.current_target_pose is None:
            # self.motor_controller.stop() # Ensure motors are stopped if no target
            return False

        tvec, rvec = self.current_target_pose
        
        # Target x-coordinate in camera frame (error for turning PID)
        # tvec from estimatePoseSingleMarkers (for a single marker in the tvecs array) is shape (1,3), i.e., [[x, y, z]]
        target_x_cam = tvec[0][0]
        # Current distance to marker (along Z-axis in camera frame)
        current_distance_cam = tvec[0][2]

        # Check if we are at the target position
        if self.is_at_target(target_x_cam, current_distance_cam):
            logger.info("NavigationController: At target position.")
            self.motor_controller.stop()
            return False # Reached target

        # --- PID Computations ---
        # Turning PID: input is current x-offset, setpoint is 0
        # Output is a turning speed adjustment
        turning_effort = self.turning_pid.compute(current_value=target_x_cam)
        
        # Distance PID: input is current distance, setpoint is NAV_TARGET_APPROACH_DISTANCE_M
        # Output is a forward speed adjustment. A positive output means we need to move forward.
        # A negative output means we need to move backward (overshot or starting too close).
        # To make PID output positive for "move forward", we can use (setpoint - current_value)
        # or invert the PID output if compute is (current_value - setpoint)
        # My PID compute is (setpoint - current_value), so positive output means current_value < setpoint (need to increase current_value, i.e., move away if setpoint is distance)
        # Let's adjust: setpoint for distance PID is target_approach_distance. Input is current_distance.
        # Error = target_approach_distance - current_distance.
        # Positive error means too far, need to move forward. Negative error = too close, move backward.
        # The PID output will try to correct this error.
        # For our PID: output = Kp*error + ...
        # So, if current_distance < NAV_TARGET_APPROACH_DISTANCE_M (too close), error is positive, output is positive.
        # This means the PID output should drive the robot BACKWARDS if positive.
        # Let's re-evaluate: PID setpoint is the value we want to reach.
        # distance_pid.setpoint = config.NAV_TARGET_APPROACH_DISTANCE_M
        # distance_pid.compute(current_value = current_distance_cam)
        # if current_distance_cam > setpoint (too far), error = setpoint - current = negative. Output negative -> drive forward.
        # if current_distance_cam < setpoint (too close), error = setpoint - current = positive. Output positive -> drive backward.
        # So, a negative PID output for distance means move forward.
        distance_effort = self.distance_pid.compute(current_value=current_distance_cam)

        # --- Convert PID efforts to motor commands ---
        # Clamp PID outputs to a manageable range before scaling to motor speeds
        # These clamps are somewhat arbitrary and might need tuning based on typical PID output values
        # clamped_turning_effort = max(-1.0, min(1.0, turning_effort / 100.0)) # Normalize somewhat
        clamped_distance_effort = max(-1.0, min(1.0, distance_effort / 100.0)) # Normalize somewhat

        # Determine motor speeds
        # TEMP: Direct scaling for turn_speed to test kp effect
        raw_turn_speed = turning_effort * 0.1 # Example scaling, adjust as needed, can be Kp dependent too
        turn_speed = max(-self.max_turning_speed, min(self.max_turning_speed, raw_turn_speed))
        # turn_speed = clamped_turning_effort * self.max_turning_speed # Original line
        
        forward_speed = -clamped_distance_effort * self.max_forward_speed # Negative because negative effort means move forward
        
        # Apply a minimum effective speed if the computed speed is too low but there's error
        if 0 < abs(forward_speed) < self.min_effective_speed and not self.is_at_distance(current_distance_cam):
            forward_speed = self.min_effective_speed * (1 if forward_speed > 0 else -1)
        # TEMP: Comment out min_effective_speed for turning to isolate kp
        # if 0 < abs(turn_speed) < self.min_effective_speed and not self.is_centered(target_x_cam):
        #      turn_speed = self.min_effective_speed * (1 if turn_speed > 0 else -1)

        # Simple control logic: Prioritize turning if not centered, then drive.
        # This can be made more sophisticated (e.g., combined turning and driving).
        if not self.is_centered(target_x_cam):
            logger.debug(f"Nav: Turning. X_err: {target_x_cam:.3f}, TurnEffort: {turning_effort:.2f}, TurnSpeed: {turn_speed:.2f}")
            self.motor_controller.stop() # Stop forward movement while turning sharply
            if turn_speed > 0: # Positive turn_speed (marker to left, PID error positive), but turn_left() in MC goes right
                self.motor_controller.turn_right(abs(turn_speed)) # Call turn_right to achieve physical left turn
            else: # Negative turn_speed (marker to right, PID error negative), but turn_right() in MC goes left
                self.motor_controller.turn_left(abs(turn_speed))  # Call turn_left to achieve physical right turn
        elif not self.is_at_distance(current_distance_cam):
            logger.debug(f"Nav: Driving. Z_err: {(current_distance_cam - self.target_approach_distance_m):.3f}, DistEffort: {distance_effort:.2f}, FwdSpeed: {forward_speed:.2f}")
            if forward_speed > 0: # Positive forward_speed means move forward
                self.motor_controller.drive_forward(abs(forward_speed))
            else: # Negative forward_speed means move backward
                self.motor_controller.drive_backward(abs(forward_speed))
        else:
            # Should have been caught by is_at_target, but as a fallback:
            logger.debug("Nav: Minor adjustments or at target, stopping.")
            self.motor_controller.stop()
            return False
            
        return True # Actively navigating

    def is_centered(self, target_x_cam: float) -> bool:
        return abs(target_x_cam) < self.x_threshold_m

    def is_at_distance(self, current_distance_cam: float) -> bool:
        return abs(current_distance_cam - self.target_approach_distance_m) < self.distance_threshold_m

    def is_at_target(self, target_x_cam: float, current_distance_cam: float) -> bool:
        """Check if the robot is at the target position (centered and at approach distance)."""
        centered = self.is_centered(target_x_cam)
        at_distance = self.is_at_distance(current_distance_cam)
        # Could add orientation check here using rvec if needed
        # For now, just position based on tvec
        if centered and at_distance:
            logger.debug("Navigation: Reached target (centered and at distance).")
            return True
        return False

    def stop_navigation(self):
        """Stops any active navigation and resets the motor controller."""
        self.clear_target()
        logger.info("NavigationController: Navigation explicitly stopped.")

    def update_turning_pid_gains(self, kp: float, ki: float, kd: float):
        """Updates the gains for the turning PID controller."""
        self.turning_pid.set_gains(kp, ki, kd)
        logger.info(f"NavigationController: Turning PID gains updated to Kp={kp}, Ki={ki}, Kd={kd}")

    def update_distance_pid_gains(self, kp: float, ki: float, kd: float):
        """Updates the gains for the distance PID controller."""
        self.distance_pid.set_gains(kp, ki, kd)
        logger.info(f"NavigationController: Distance PID gains updated to Kp={kp}, Ki={ki}, Kd={kd}") 