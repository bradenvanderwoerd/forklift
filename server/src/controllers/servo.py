from config import SERVO_PIN
import RPi.GPIO as GPIO
import time
from typing import Optional
import logging

logger = logging.getLogger(__name__)

class ServoController:
    def __init__(self):
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.pin = SERVO_PIN
        self.current_position = 0
        self.target_position = 0
        self.step_size = 2  # Changed from 5 to 2 degrees per step
        self.min_angle = 0
        self.max_angle = 80  # Changed from 90 to 80 degrees
        
        # PWM setup
        self.pwm = GPIO.PWM(self.pin, 50)  # 50 Hz
        self.pwm.start(0)
    
    def set_position(self, angle: float):
        """Set servo position
        
        Args:
            angle: Target angle (0-80 degrees)
        """
        # Clamp angle to valid range
        angle = max(self.min_angle, min(self.max_angle, angle))
        self.target_position = angle
        
        # Calculate duty cycle (2.5% to 12.5% for 0-180 degrees)
        # We use a smaller range for our 0-45 degree movement
        duty_cycle = 2.5 + (angle / 180.0) * 10.0
        
        # Apply position
        self.pwm.ChangeDutyCycle(duty_cycle)
        self.current_position = angle
        logger.info(f"Servo position set to {self.current_position:.2f} degrees (Duty Cycle: {duty_cycle:.2f}%)")
        
        # Wait for servo to reach position
        time.sleep(0.1)
    
    def step_up(self):
        """Move servo up by one step"""
        new_position = min(self.max_angle, self.current_position + self.step_size)
        self.set_position(new_position)
    
    def step_down(self):
        """Move servo down by one step"""
        new_position = max(self.min_angle, self.current_position - self.step_size)
        self.set_position(new_position)
    
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