from ..utils.config import SERVO_PWM_PIN, FORK_DOWN_POSITION, FORK_UP_POSITION, SERVO_STEP_DEGREES, SERVO_STEP_DELAY_SECONDS
import RPi.GPIO as GPIO
import time
from typing import Optional
import logging

logger = logging.getLogger(__name__)

class ServoController:
    def __init__(self):
        GPIO.setup(SERVO_PWM_PIN, GPIO.OUT)
        self.pin = SERVO_PWM_PIN
        self.current_position = 0 # Assume starts at 0, or read actual initial position if possible
        # self.target_position = 0 # Not strictly needed if set_position manages targets
        
        # Servo movement parameters from config
        self.step_degrees = SERVO_STEP_DEGREES
        self.step_delay_seconds = SERVO_STEP_DELAY_SECONDS
        
        self.min_angle = 0 # Physical min angle of servo/forklift
        self.max_angle = 80  # Physical max angle of servo/forklift (example)
        
        # PWM setup
        self.pwm = GPIO.PWM(self.pin, 50)  # 50 Hz PWM frequency
        self.pwm.start(self._angle_to_duty_cycle(self.current_position)) # Start PWM at current/initial position
        logger.info(f"ServoController initialized. Current position assumed/set to: {self.current_position:.2f} degrees.")

    def _angle_to_duty_cycle(self, angle: float) -> float:
        """Converts an angle (0-180 nominally) to a PWM duty cycle (2.5-12.5 for SG90 type servos)."""
        # Standard SG90 servo: 0 deg = 2.5% duty, 180 deg = 12.5% duty.
        # Ensure angle is within a reasonable servo operating range if necessary, though clamping happens in set_position.
        return 2.5 + (angle / 180.0) * 10.0
    
    def set_position(self, target_angle: float, blocking: bool = True):
        """Set servo to target_angle, moving smoothly in steps."""
        target_angle = max(self.min_angle, min(self.max_angle, target_angle))
        logger.info(f"Servo: Moving from {self.current_position:.2f} to {target_angle:.2f} degrees.")

        if abs(self.current_position - target_angle) < self.step_degrees / 2.0:
            logger.debug("Servo: Already at target angle or very close.")
            # Ensure final position is set accurately if slightly off but within step threshold
            if self.current_position != target_angle:
                 self.pwm.ChangeDutyCycle(self._angle_to_duty_cycle(target_angle))
                 self.current_position = target_angle
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
            
            self.pwm.ChangeDutyCycle(self._angle_to_duty_cycle(next_step_angle))
            self.current_position = next_step_angle
            # logger.debug(f"Servo step: {self.current_position:.2f}") # Optional: log each step
            if blocking:
                time.sleep(self.step_delay_seconds)
            else:
                # For non-blocking, we might need a different approach to manage updates
                # For now, set_position will be effectively blocking due to the loop
                pass 
        
        # Ensure final position is precisely set
        if self.current_position != target_angle:
             self.pwm.ChangeDutyCycle(self._angle_to_duty_cycle(target_angle))
             self.current_position = target_angle

        logger.info(f"Servo: Reached target position {self.current_position:.2f} degrees.")
    
    def go_to_down_position(self, blocking: bool = True):
        """Move servo to the predefined FORK_DOWN_POSITION."""
        logger.info("Servo: Moving to FORK_DOWN_POSITION.")
        self.set_position(FORK_DOWN_POSITION, blocking=blocking)

    def go_to_up_position(self, blocking: bool = True):
        """Move servo to the predefined FORK_UP_POSITION."""
        logger.info("Servo: Moving to FORK_UP_POSITION.")
        self.set_position(FORK_UP_POSITION, blocking=blocking)

    def get_position(self) -> float:
        """Get current servo position
        
        Returns:
            Current position in degrees
        """
        return self.current_position
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.pwm.stop()
        GPIO.cleanup([self.pin]) 