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
        self.current_position = FORK_DOWN_POSITION 
        
        self.step_degrees = SERVO_STEP_DEGREES
        self.step_delay_seconds = SERVO_STEP_DELAY_SECONDS
        
        self.min_angle = FORK_DOWN_POSITION 
        self.max_angle = FORK_UP_POSITION  
        
        self.pwm = GPIO.PWM(self.pin, 60) # Changed frequency from 50Hz to 60Hz for testing
        initial_duty_cycle = self._angle_to_duty_cycle(self.current_position)
        self.pwm.start(initial_duty_cycle) # Start PWM at initial position
        # PWM is now left running. No sleep, stop, or pin LOW here.
        logger.info(f"ServoController initialized. Position: {self.current_position:.2f} deg. PWM active.")

    def _angle_to_duty_cycle(self, angle: float) -> float:
        """Converts an angle (0-180 nominally) to a PWM duty cycle (2.5-12.5 for SG90 type servos)."""
        # Standard SG90 servo: 0 deg = 2.5% duty, 180 deg = 12.5% duty.
        # Ensure angle is within a reasonable servo operating range if necessary, though clamping happens in set_position.
        return 2.5 + (angle / 180.0) * 10.0
    
    def set_position(self, target_angle: float, blocking: bool = True):
        # Remove CRITICAL log for now, can be re-added if needed
        # logger.critical(f"!!!! SERVO set_position CALLED with target_angle: {target_angle}, current_position: {self.current_position} !!!!")
        
        target_angle = max(self.min_angle, min(self.max_angle, target_angle))
        logger.info(f"Servo: Moving from {self.current_position:.2f} to {target_angle:.2f} degrees.")

        if abs(self.current_position - target_angle) < self.step_degrees / 2.0:
            logger.debug("Servo: Already at target angle or very close.")
            # Ensure final position is accurately set if slightly off
            if self.current_position != target_angle:
                 self.pwm.ChangeDutyCycle(self._angle_to_duty_cycle(target_angle))
                 self.current_position = target_angle
            # No sleep or pwm.stop() needed, PWM stays active to hold position
            return

        if target_angle > self.current_position:
            direction = 1
        else:
            direction = -1
        
        num_steps = int(abs(target_angle - self.current_position) / self.step_degrees)
        if num_steps == 0 and self.current_position != target_angle: 
            num_steps = 1

        for i in range(num_steps):
            next_step_angle = self.current_position + (direction * self.step_degrees)
            if (direction == 1 and next_step_angle > target_angle) or \
               (direction == -1 and next_step_angle < target_angle):
                next_step_angle = target_angle
            
            self.pwm.ChangeDutyCycle(self._angle_to_duty_cycle(next_step_angle))
            self.current_position = next_step_angle
            if blocking:
                time.sleep(self.step_delay_seconds)
        
        # Final precise position set by the loop or initial check
        # Ensure the duty cycle reflects the final target_angle if loop didn't hit it exactly
        # (though it should if step_degrees is small)
        if self.current_position != target_angle: # Should be rare with small steps
            self.pwm.ChangeDutyCycle(self._angle_to_duty_cycle(target_angle))
            self.current_position = target_angle
        
        # PWM remains active at the target_angle duty cycle to hold position
        logger.info(f"Servo: Reached target position {self.current_position:.2f} degrees. PWM holding.")
    
    def go_to_down_position(self, blocking: bool = True):
        logger.info("Servo: Moving to FORK_DOWN_POSITION.")
        self.set_position(FORK_DOWN_POSITION, blocking=blocking)

    def go_to_up_position(self, blocking: bool = True):
        logger.info("Servo: Moving to FORK_UP_POSITION.")
        self.set_position(FORK_UP_POSITION, blocking=blocking)

    def get_position(self) -> float:
        """Get current servo position
        
        Returns:
            Current position in degrees
        """
        return self.current_position
    
    def cleanup(self):
        logger.info("Servo: Cleanup called. Stopping PWM and cleaning GPIO.")
        try:
            self.pwm.stop()
        except Exception as e:
            logger.debug(f"Servo: Exception during pwm.stop() in cleanup: {e}")
        GPIO.cleanup([self.pin]) 