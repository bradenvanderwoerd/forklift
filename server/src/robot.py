import pigpio
import logging
import time
from typing import Dict, Any
import config

logger = logging.getLogger(__name__)

class Robot:
    def __init__(self):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon")
            
        # Initialize motor pins
        for pin in config.MOTOR_PINS.values():
            self.pi.set_mode(pin, pigpio.OUTPUT)
            
        # Initialize PWM pins
        self.pi.set_mode(config.MOTOR1_PWM, pigpio.OUTPUT)
        self.pi.set_mode(config.MOTOR2_PWM, pigpio.OUTPUT)
        
        # Initialize servo
        self.pi.set_mode(config.SERVO_PIN, pigpio.OUTPUT)
        self.current_servo_angle = 0
        self.servo_step_size = 5  # Step size in degrees
        
    def cleanup(self):
        """Clean up GPIO resources"""
        # Stop motors
        self.stop()
        
        # Stop servo
        self.pi.set_servo_pulsewidth(config.SERVO_PIN, 0)
        
        # Disconnect from pigpio
        self.pi.stop()
        
    def stop(self):
        """Stop all motors"""
        for pin in config.MOTOR_PINS.values():
            self.pi.write(pin, 0)
        self.pi.set_PWM_dutycycle(config.MOTOR1_PWM, 0)
        self.pi.set_PWM_dutycycle(config.MOTOR2_PWM, 0)
        
    def drive(self, direction: str, speed: int):
        """Drive the robot in the specified direction at the given speed"""
        # Stop all motors first
        self.stop()
        
        # Clamp speed to valid range
        speed = max(0, min(speed, config.MAX_SPEED))
        
        # Convert speed percentage to PWM value (0-255)
        pwm_value = int((speed / 100.0) * 255)
        
        if direction == "FORWARD":
            self.pi.write(config.MOTOR_PINS['left_forward'], 1)
            self.pi.write(config.MOTOR_PINS['right_forward'], 1)
        elif direction == "BACKWARD":
            self.pi.write(config.MOTOR_PINS['left_backward'], 1)
            self.pi.write(config.MOTOR_PINS['right_backward'], 1)
        elif direction == "LEFT":
            self.pi.write(config.MOTOR_PINS['right_forward'], 1)
            self.pi.write(config.MOTOR_PINS['left_backward'], 1)
        elif direction == "RIGHT":
            self.pi.write(config.MOTOR_PINS['left_forward'], 1)
            self.pi.write(config.MOTOR_PINS['right_backward'], 1)
            
        # Set motor speeds
        self.pi.set_PWM_dutycycle(config.MOTOR1_PWM, pwm_value)
        self.pi.set_PWM_dutycycle(config.MOTOR2_PWM, pwm_value)
        
    def _angle_to_pulse_width(self, angle: float) -> int:
        """Convert angle in degrees to servo pulse width in microseconds"""
        # Map angle (0-180) to pulse width (500-2500)
        return int(500 + (angle / 180.0) * 2000)
        
    def step_servo(self, step_up: bool = False, step_down: bool = False):
        """Step the servo up or down by a fixed amount"""
        if step_up:
            self.current_servo_angle = min(self.current_servo_angle + self.servo_step_size, config.MAX_SERVO_ANGLE)
        elif step_down:
            self.current_servo_angle = max(self.current_servo_angle - self.servo_step_size, 0)
            
        # Convert angle to pulse width
        pulse_width = self._angle_to_pulse_width(self.current_servo_angle)
        self.pi.set_servo_pulsewidth(config.SERVO_PIN, pulse_width)
        logger.debug(f"Servo angle: {self.current_servo_angle}° (pulse width: {pulse_width})") 