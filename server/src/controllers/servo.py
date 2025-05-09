from ..utils.config import SERVO_PWM_PIN, FORK_DOWN_POSITION, FORK_UP_POSITION, SERVO_STEP_DEGREES, SERVO_STEP_DELAY_SECONDS
import RPi.GPIO as GPIO # Uncomment
import time
from typing import Optional
import logging

logger = logging.getLogger(__name__)

class ServoController:
    def __init__(self):
        GPIO.setup(SERVO_PWM_PIN, GPIO.OUT) # Uncomment
        self.pin = SERVO_PWM_PIN
        self.current_position = 0 # Assume starts at 0, or read actual initial position if possible
        # self.target_position = 0 # Not strictly needed if set_position manages targets
        
        # Servo movement parameters from config
        self.step_degrees = SERVO_STEP_DEGREES
        self.step_delay_seconds = SERVO_STEP_DELAY_SECONDS
        
        self.min_angle = 0 # Physical min angle of servo/forklift
        self.max_angle = 80  # Physical max angle of servo/forklift (example)
        
        # PWM setup
        self.pwm = GPIO.PWM(self.pin, 50)  # Uncomment 
        # Initialize to starting position, then stop PWM to prevent initial jitter
        initial_duty_cycle = self._angle_to_duty_cycle(self.current_position)
        self.pwm.start(initial_duty_cycle) 
        time.sleep(0.5) # Allow time for servo to reach initial position
        self.pwm.stop() 
        logger.info(f"ServoController initialized. Position set to: {self.current_position:.2f} degrees. PWM stopped.")

    def _angle_to_duty_cycle(self, angle: float) -> float:
        """Converts an angle (0-180 nominally) to a PWM duty cycle (2.5-12.5 for SG90 type servos)."""
        # Standard SG90 servo: 0 deg = 2.5% duty, 180 deg = 12.5% duty.
        # Ensure angle is within a reasonable servo operating range if necessary, though clamping happens in set_position.
        return 2.5 + (angle / 180.0) * 10.0
    
    def set_position(self, target_angle: float, blocking: bool = True):
        """Set servo to target_angle, moving smoothly in steps."""
        target_angle = max(self.min_angle, min(self.max_angle, target_angle))
        logger.info(f"Servo: Moving from {self.current_position:.2f} to {target_angle:.2f} degrees.") # Revert log

        # Start PWM at current known position. This ensures it's active for ChangeDutyCycle.
        # If we are already at target, we'll stop it shortly.
        self.pwm.start(self._angle_to_duty_cycle(self.current_position))

        if abs(self.current_position - target_angle) < self.step_degrees / 2.0:
            logger.debug("Servo: Already at target angle or very close.") # Revert log
            if self.current_position != target_angle:
                 # Briefly ensure it's at the precise target_angle if slightly off
                 self.pwm.ChangeDutyCycle(self._angle_to_duty_cycle(target_angle))
                 self.current_position = target_angle
                 if blocking: # Allow this micro-adjustment to complete
                     time.sleep(max(self.step_delay_seconds, 0.05)) 
            if blocking: # Add a small settle time even if no significant move
                time.sleep(max(self.step_delay_seconds, 0.05))
            self.pwm.stop() # Stop PWM
            logger.info(f"Servo: At target {self.current_position:.2f}. PWM stopped.")
            return

        # Determine direction and number of steps
        if target_angle > self.current_position:
            direction = 1
        else:
            direction = -1
        
        num_steps = int(abs(target_angle - self.current_position) / self.step_degrees)
        if num_steps == 0 and self.current_position != target_angle: # Ensure at least one step if not perfectly aligned
            num_steps = 1

        for i in range(num_steps):
            next_step_angle = self.current_position + (direction * self.step_degrees)
            # Ensure we don't overshoot the final target_angle with the last step
            if (direction == 1 and next_step_angle > target_angle) or \
               (direction == -1 and next_step_angle < target_angle):
                next_step_angle = target_angle
            
            self.pwm.ChangeDutyCycle(self._angle_to_duty_cycle(next_step_angle)) # Uncomment
            self.current_position = next_step_angle
            # logger.debug(f"Servo step: {self.current_position:.2f}") # Optional: log each step
            if blocking:
                time.sleep(self.step_delay_seconds)
        
        # Ensure final position is precisely set
        if self.current_position != target_angle:
             self.pwm.ChangeDutyCycle(self._angle_to_duty_cycle(target_angle)) # Uncomment
             self.current_position = target_angle

        # Allow servo to reach the position and then stop PWM signal to prevent jitter
        # A small delay for the servo to physically complete the last move.
        # self.step_delay_seconds is the delay between steps. A final settle time might be similar
        # or slightly longer. If blocking, we assume the user wants to wait.
        if blocking:
            # Use a slightly longer delay for final settling if step_delay is very small,
            # otherwise, the step_delay itself should be sufficient.
            final_settle_delay = max(self.step_delay_seconds, 0.3) # Ensure at least 0.3s (was 0.1s)
            time.sleep(final_settle_delay)
        
        self.pwm.stop() # Stop PWM signal pulses
        logger.info(f"Servo: Reached target position {self.current_position:.2f} degrees. PWM stopped.") # Revert log
    
    def go_to_down_position(self, blocking: bool = True):
        """Move servo to the predefined FORK_DOWN_POSITION."""
        logger.info("Servo: Moving to FORK_DOWN_POSITION.") # Revert log
        self.set_position(FORK_DOWN_POSITION, blocking=blocking)

    def go_to_up_position(self, blocking: bool = True):
        """Move servo to the predefined FORK_UP_POSITION."""
        logger.info("Servo: Moving to FORK_UP_POSITION.") # Revert log
        self.set_position(FORK_UP_POSITION, blocking=blocking)

    def get_position(self) -> float:
        """Get current servo position
        
        Returns:
            Current position in degrees
        """
        return self.current_position
    
    def cleanup(self):
        """Clean up GPIO resources"""
        # Ensure PWM is stopped before cleaning up GPIO
        try:
            self.pwm.stop()
        except Exception: # pwm might not be initialized if __init__ failed
            pass 
        GPIO.cleanup([self.pin]) # Uncomment
        logger.info("Servo: Cleanup called. PWM stopped and GPIO cleaned.") # Revert log 