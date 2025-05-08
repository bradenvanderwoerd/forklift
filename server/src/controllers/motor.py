import RPi.GPIO as GPIO
import time
from typing import Tuple

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
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Motor pins
        self.MOTOR1_PIN1 = 5   # Left motor forward
        self.MOTOR1_PIN2 = 6   # Left motor backward
        self.MOTOR2_PIN1 = 22  # Right motor forward
        self.MOTOR2_PIN2 = 23  # Right motor backward
        
        # PWM pins
        self.MOTOR1_PWM = 17   # Left motor speed
        self.MOTOR2_PWM = 24   # Right motor speed
        
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
        # Clamp speed to valid range
        speed = max(-100, min(100, speed))
        
        # Get PID-adjusted speed
        adjusted_speed = self.pid_controller.compute(speed, self.current_speed)
        
        # Apply speed to motors
        self._apply_speed(adjusted_speed)
        self.current_speed = speed
    
    def _apply_speed(self, speed: float):
        """Apply speed to both motors
        
        Args:
            speed: Speed value (-100 to 100)
        """
        # Determine direction and magnitude
        direction = 1 if speed >= 0 else -1
        magnitude = abs(speed)
        
        # Set motor directions
        GPIO.output(self.MOTOR1_PIN1, GPIO.HIGH if direction > 0 else GPIO.LOW)
        GPIO.output(self.MOTOR1_PIN2, GPIO.LOW if direction > 0 else GPIO.HIGH)
        GPIO.output(self.MOTOR2_PIN1, GPIO.HIGH if direction > 0 else GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN2, GPIO.LOW if direction > 0 else GPIO.HIGH)
        
        # Set PWM duty cycle
        self.pwm1.ChangeDutyCycle(magnitude)
        self.pwm2.ChangeDutyCycle(magnitude)
    
    def stop(self):
        """Stop both motors"""
        self.set_speed(0)
        GPIO.output(self.MOTOR1_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR1_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN2, GPIO.LOW)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.stop()
        self.pwm1.stop()
        self.pwm2.stop()
        GPIO.cleanup()

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
        GPIO.output(self.MOTOR1_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR1_PIN2, GPIO.HIGH)
        GPIO.output(self.MOTOR2_PIN1, GPIO.HIGH)
        GPIO.output(self.MOTOR2_PIN2, GPIO.LOW)
        self.pwm1.ChangeDutyCycle(speed)
        self.pwm2.ChangeDutyCycle(speed)
        self.current_speed = 0

    def turn_right(self, speed: float):
        """Turn right by running right motor backward and left motor forward"""
        speed = max(0, min(100, speed))
        GPIO.output(self.MOTOR1_PIN1, GPIO.HIGH)
        GPIO.output(self.MOTOR1_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN2, GPIO.HIGH)
        self.pwm1.ChangeDutyCycle(speed)
        self.pwm2.ChangeDutyCycle(speed)
        self.current_speed = 0 