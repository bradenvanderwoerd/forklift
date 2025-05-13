import pigpio
from config import SERVO_PIN
import time
from typing import Optional
import logging

logger = logging.getLogger(__name__)

class ServoController:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon")
            
        self.pin = SERVO_PIN
        self.current_position = 0
        self.target_position = 0
        self.step_size = 2  # 2 degrees per step
        self.min_angle = 0
        self.max_angle = 80  # Maximum angle in degrees
        
        # Servo pulse width range (in microseconds)
        self.min_pulse = 500   # 0 degrees
        self.max_pulse = 2500  # 180 degrees
        
        # Set initial position
        self.set_position(0)
    
    def set_position(self, angle: float):
        """Set servo position
        
        Args:
            angle: Target angle (0-80 degrees)
        """
        # Clamp angle to valid range
        angle = max(self.min_angle, min(self.max_angle, angle))
        self.target_position = angle
        
        # Convert angle to pulse width
        pulse_width = self._angle_to_pulse(angle)
        
        # Apply position using hardware PWM
        self.pi.set_servo_pulsewidth(self.pin, pulse_width)
        self.current_position = angle
        logger.info(f"Servo position set to {self.current_position:.2f} degrees (Pulse Width: {pulse_width}μs)")
        
        # Wait for servo to reach position
        time.sleep(0.1)
    
    def _angle_to_pulse(self, angle: float) -> int:
        """Convert angle to pulse width in microseconds
        
        Args:
            angle: Angle in degrees (0-80)
            
        Returns:
            Pulse width in microseconds
        """
        # Scale angle to full servo range (0-180)
        scaled_angle = (angle / self.max_angle) * 180.0
        
        # Convert to pulse width
        pulse_width = self.min_pulse + (scaled_angle / 180.0) * (self.max_pulse - self.min_pulse)
        return int(pulse_width)
    
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
        """Clean up pigpio resources"""
        self.pi.set_servo_pulsewidth(self.pin, 0)  # Stop servo
        self.pi.stop()  # Disconnect from pigpio daemon 