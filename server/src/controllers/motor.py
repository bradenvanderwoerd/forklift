import RPi.GPIO as GPIO
import time
from typing import Tuple
# Updated import to use relative path to the consolidated config
from ..utils.config import (
    MOTOR_LEFT_FORWARD_PIN, MOTOR_LEFT_BACKWARD_PIN, MOTOR_LEFT_PWM_PIN,
    MOTOR_RIGHT_FORWARD_PIN, MOTOR_RIGHT_BACKWARD_PIN, MOTOR_RIGHT_PWM_PIN
)
import logging

logger = logging.getLogger(__name__)

# MIN_OPERATIONAL_PWM = 15 # Reverting this change

class PIDController:
    def __init__(self, kp: float = 1.0, ki: float = 0.1, kd: float = 0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, target_speed: float, current_speed: float) -> float:
        current_time = time.time()
        dt = current_time - self.last_time
        
        error = target_speed - current_speed
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        self.previous_error = error
        self.last_time = current_time
        
        return output

class MotorController:
    def __init__(self):
        # GPIO.setmode(GPIO.BCM) # Removed
        # GPIO.setwarnings(False) # Removed
        
        # Motor pins from the new config
        self.MOTOR1_PIN1 = MOTOR_LEFT_FORWARD_PIN    # Left motor forward
        self.MOTOR1_PIN2 = MOTOR_LEFT_BACKWARD_PIN   # Left motor backward
        self.MOTOR2_PIN1 = MOTOR_RIGHT_FORWARD_PIN   # Right motor forward
        self.MOTOR2_PIN2 = MOTOR_RIGHT_BACKWARD_PIN  # Right motor backward
        
        # PWM pins from the new config
        self.MOTOR1_PWM = MOTOR_LEFT_PWM_PIN       # Left motor PWM
        self.MOTOR2_PWM = MOTOR_RIGHT_PWM_PIN      # Right motor PWM
        
        # Setup pins
        self._setup_pins()
        
        # Initialize PWM
        self.pwm1 = GPIO.PWM(self.MOTOR1_PWM, 1000)  # 1kHz
        self.pwm2 = GPIO.PWM(self.MOTOR2_PWM, 1000)
        self.pwm1.start(0)
        self.pwm2.start(0)
        
        # Initialize controllers
        self.current_speed = 0
        self.pid_controller = PIDController()
        
    def _setup_pins(self):
        """Setup all GPIO pins for motor control"""
        pins = [self.MOTOR1_PIN1, self.MOTOR1_PIN2, 
                self.MOTOR2_PIN1, self.MOTOR2_PIN2,
                self.MOTOR1_PWM, self.MOTOR2_PWM]
        
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
    
    def set_speed(self, speed: float):
        """Set motor speed with PID control
        
        Args:
            speed: Target speed (-100 to 100)
        """
        speed = max(-100, min(100, speed)) # Clamp input target speed
        adjusted_speed = self.pid_controller.compute(speed, self.current_speed)
        self._apply_speed(adjusted_speed) # Pass PID-adjusted speed
        self.current_speed = speed
    
    def _apply_speed(self, speed: float):
        """Apply speed to both motors
        
        Args:
            speed: Speed value, potentially from PID so could be outside -100 to 100
        """
        direction = 1 if speed >= 0 else -1
        # Clamp magnitude to 0-100 before applying to PWM
        magnitude = max(0, min(100, abs(speed)))
        
        # Set motor directions
        GPIO.output(self.MOTOR1_PIN1, GPIO.HIGH if direction > 0 else GPIO.LOW)
        GPIO.output(self.MOTOR1_PIN2, GPIO.LOW if direction > 0 else GPIO.HIGH)
        GPIO.output(self.MOTOR2_PIN1, GPIO.HIGH if direction > 0 else GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN2, GPIO.LOW if direction > 0 else GPIO.HIGH)
        
        # Set PWM duty cycle
        self.pwm1.ChangeDutyCycle(magnitude)
        self.pwm2.ChangeDutyCycle(magnitude)
    
    def stop(self):
        """Stop both motors directly, bypassing PID for an immediate stop."""
        logger.info("MotorController.stop() called - setting PWMs to 0 and pins to LOW.") # Added log
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
        GPIO.output(self.MOTOR1_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR1_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN2, GPIO.LOW)
        self.current_speed = 0 # Reset current speed state
        # Reset PID controller state when stopping to prevent integral windup issues on restart
        self.pid_controller.integral = 0 
        self.pid_controller.previous_error = 0
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop()
        self.pwm1.stop()
        self.pwm2.stop()
        # Only clean up the pins used by this controller
        pins_to_cleanup = [
            self.MOTOR1_PIN1, self.MOTOR1_PIN2,
            self.MOTOR2_PIN1, self.MOTOR2_PIN2,
            self.MOTOR1_PWM, self.MOTOR2_PWM
        ]
        GPIO.cleanup(pins_to_cleanup)

    def drive_forward(self, speed: float):
        """Drive both motors forward at the given speed (0-100)"""
        speed = max(0, min(100, speed))
        GPIO.output(self.MOTOR1_PIN1, GPIO.HIGH)
        GPIO.output(self.MOTOR1_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN1, GPIO.HIGH)
        GPIO.output(self.MOTOR2_PIN2, GPIO.LOW)
        self.pwm1.ChangeDutyCycle(speed)
        self.pwm2.ChangeDutyCycle(speed)
        self.current_speed = speed

    def drive_backward(self, speed: float):
        """Drive both motors backward at the given speed (0-100)"""
        speed = max(0, min(100, speed))
        GPIO.output(self.MOTOR1_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR1_PIN2, GPIO.HIGH)
        GPIO.output(self.MOTOR2_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN2, GPIO.HIGH)
        self.pwm1.ChangeDutyCycle(speed)
        self.pwm2.ChangeDutyCycle(speed)
        self.current_speed = -speed

    def turn_left(self, speed: float):
        """Turn left by running left motor backward and right motor forward"""
        speed = max(0, min(100, speed))
        # effective_speed = 0 # Reverting
        # if speed > 0: # Reverting
        #     effective_speed = max(MIN_OPERATIONAL_PWM, speed) # Reverting
            
        GPIO.output(self.MOTOR1_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR1_PIN2, GPIO.HIGH)
        GPIO.output(self.MOTOR2_PIN1, GPIO.HIGH)
        GPIO.output(self.MOTOR2_PIN2, GPIO.LOW)
        self.pwm1.ChangeDutyCycle(speed) # Reverted to use speed directly
        self.pwm2.ChangeDutyCycle(speed) # Reverted to use speed directly
        self.current_speed = 0

    def turn_right(self, speed: float):
        """Turn right by running right motor backward and left motor forward"""
        speed = max(0, min(100, speed))
        # effective_speed = 0 # Reverting
        # if speed > 0: # Reverting
        #     effective_speed = max(MIN_OPERATIONAL_PWM, speed) # Reverting
            
        GPIO.output(self.MOTOR1_PIN1, GPIO.HIGH)
        GPIO.output(self.MOTOR1_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN2, GPIO.HIGH)
        self.pwm1.ChangeDutyCycle(speed) # Reverted to use speed directly
        self.pwm2.ChangeDutyCycle(speed) # Reverted to use speed directly
        self.current_speed = 0 