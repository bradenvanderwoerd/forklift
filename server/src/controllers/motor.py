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
        # self.current_speed = 0 # This was for the old set_speed, less relevant for individual control
        # self.pid_controller = PIDController() # This PID was for the old set_speed, not used by move()

    def _setup_pins(self):
        """Setup all GPIO pins for motor control"""
        pins = [self.MOTOR1_PIN1, self.MOTOR1_PIN2, 
                self.MOTOR2_PIN1, self.MOTOR2_PIN2,
                self.MOTOR1_PWM, self.MOTOR2_PWM]
        
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
    
    def _set_motor_speed(self, motor_pwm_obj, pin_forward, pin_backward, speed: int):
        """Sets the speed and direction for a single motor.

        Args:
            motor_pwm_obj: The GPIO.PWM object for the motor.
            pin_forward: The GPIO pin for forward motion.
            pin_backward: The GPIO pin for backward motion.
            speed: The desired speed for this motor (-100 to 100).
                   Positive for forward, negative for backward.
        """
        speed = int(max(-100, min(100, speed))) # Clamp speed

        if speed > 0: # Forward
            GPIO.output(pin_forward, GPIO.HIGH)
            GPIO.output(pin_backward, GPIO.LOW)
            motor_pwm_obj.ChangeDutyCycle(speed)
        elif speed < 0: # Backward
            GPIO.output(pin_forward, GPIO.LOW)
            GPIO.output(pin_backward, GPIO.HIGH)
            motor_pwm_obj.ChangeDutyCycle(abs(speed))
        else: # Stop
            GPIO.output(pin_forward, GPIO.LOW)
            GPIO.output(pin_backward, GPIO.LOW)
            motor_pwm_obj.ChangeDutyCycle(0)

    def move(self, forward_component: int, turn_component: int):
        """Controls the robot's movement with separate forward and turn components.

        Args:
            forward_component: Speed for forward/backward movement (-100 to 100).
                               Positive is forward, negative is backward.
            turn_component: Speed for turning (-100 to 100).
                            Positive to turn left (CCW), negative to turn right (CW).
        """
        # logger.debug(f"MotorController.move: Fwd:{forward_component}, Turn:{turn_component}")

        # Calculate individual motor speeds
        # Positive turn_component should make the robot turn left (CCW).
        # To turn left: left motor slower or reverse, right motor faster or forward.
        # If forward_component = 0:
        #   turn_component = +50 (left turn) => left_speed = -50, right_speed = +50
        #   turn_component = -50 (right turn) => left_speed = +50, right_speed = -50
        # If forward_component = +50:
        #   turn_component = +20 (left turn) => left_speed = 50 - 20 = 30, right_speed = 50 + 20 = 70 (Incorrect for typical differential)
        #   Let's use standard differential drive mixing:
        #   Left = Forward - Turn
        #   Right = Forward + Turn
        #   With this, positive turn component makes right wheel faster/left slower = turns left.

        left_speed = forward_component - turn_component
        right_speed = forward_component + turn_component

        # Clamp speeds to [-100, 100]
        left_speed = int(max(-100, min(100, left_speed)))
        right_speed = int(max(-100, min(100, right_speed)))
        
        # logger.debug(f"MotorController.move: LeftSpeed:{left_speed}, RightSpeed:{right_speed}")

        self._set_motor_speed(self.pwm1, self.MOTOR1_PIN1, self.MOTOR1_PIN2, left_speed)
        self._set_motor_speed(self.pwm2, self.MOTOR2_PIN1, self.MOTOR2_PIN2, right_speed)
        
    def set_speed(self, speed: float):
        """Set motor speed with PID control. Deprecated by move().
        
        Args:
            speed: Target speed (-100 to 100)
        """
        logger.warning("MotorController.set_speed() is deprecated. Use move() instead.")
        # speed = max(-100, min(100, speed)) # Clamp input target speed
        # adjusted_speed = self.pid_controller.compute(speed, self.current_speed)
        # self._apply_speed(adjusted_speed) # Pass PID-adjusted speed
        # self.current_speed = speed
        # For now, make it a simple straight drive for compatibility if called.
        self._set_motor_speed(self.pwm1, self.MOTOR1_PIN1, self.MOTOR1_PIN2, int(speed))
        self._set_motor_speed(self.pwm2, self.MOTOR2_PIN1, self.MOTOR2_PIN2, int(speed))

    def _apply_speed(self, speed: float):
        """Apply speed to both motors. Deprecated by _set_motor_speed() and move().
        
        Args:
            speed: Speed value, potentially from PID so could be outside -100 to 100
        """
        logger.warning("MotorController._apply_speed() is deprecated.")
        # direction = 1 if speed >= 0 else -1
        # # Clamp magnitude to 0-100 before applying to PWM
        # magnitude = max(0, min(100, abs(speed)))
        # 
        # # Set motor directions
        # GPIO.output(self.MOTOR1_PIN1, GPIO.HIGH if direction > 0 else GPIO.LOW)
        # GPIO.output(self.MOTOR1_PIN2, GPIO.LOW if direction > 0 else GPIO.HIGH)
        # GPIO.output(self.MOTOR2_PIN1, GPIO.HIGH if direction > 0 else GPIO.LOW)
        # GPIO.output(self.MOTOR2_PIN2, GPIO.LOW if direction > 0 else GPIO.HIGH)
        # 
        # # Set PWM duty cycle
        # self.pwm1.ChangeDutyCycle(magnitude)
        # self.pwm2.ChangeDutyCycle(magnitude)
        self._set_motor_speed(self.pwm1, self.MOTOR1_PIN1, self.MOTOR1_PIN2, int(speed))
        self._set_motor_speed(self.pwm2, self.MOTOR2_PIN1, self.MOTOR2_PIN2, int(speed))

    def stop(self):
        """Stop both motors directly, bypassing PID for an immediate stop."""
        logger.info("MotorController.stop() called - setting PWMs to 0 and pins to LOW.") # Added log
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
        GPIO.output(self.MOTOR1_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR1_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN2, GPIO.LOW)
        # Reset PID controller state when stopping to prevent integral windup issues on restart
        # self.pid_controller.integral = 0 
        # self.pid_controller.previous_error = 0
    
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
        """Drives the robot forward at a given speed."""
        speed = self._validate_and_adjust_speed(speed)
        logger.info(f"MotorController: drive_forward called with speed: {speed}")
        GPIO.output(self.MOTOR1_PIN1, GPIO.HIGH) # Left motor forward
        GPIO.output(self.MOTOR1_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN1, GPIO.HIGH) # Right motor forward
        GPIO.output(self.MOTOR2_PIN2, GPIO.LOW)
        self.pwm1.ChangeDutyCycle(speed)
        self.pwm2.ChangeDutyCycle(speed)

    def drive_backward(self, speed: float):
        """Drives the robot backward at a given speed."""
        speed = self._validate_and_adjust_speed(speed)
        logger.info(f"MotorController: drive_backward called with speed: {speed}")
        GPIO.output(self.MOTOR1_PIN1, GPIO.LOW)  # Left motor backward
        GPIO.output(self.MOTOR1_PIN2, GPIO.HIGH)
        GPIO.output(self.MOTOR2_PIN1, GPIO.LOW)  # Right motor backward
        GPIO.output(self.MOTOR2_PIN2, GPIO.HIGH)
        self.pwm1.ChangeDutyCycle(speed)
        self.pwm2.ChangeDutyCycle(speed)

    def turn_left(self, speed: float):
        """Turns the robot left at a given speed."""
        speed = self._validate_and_adjust_speed(speed)
        logger.info(f"MotorController: turn_left called with speed: {speed}")
        GPIO.output(self.MOTOR1_PIN1, GPIO.LOW)  # Left motor backward
        GPIO.output(self.MOTOR1_PIN2, GPIO.HIGH)
        GPIO.output(self.MOTOR2_PIN1, GPIO.HIGH) # Right motor forward
        GPIO.output(self.MOTOR2_PIN2, GPIO.LOW)
        self.pwm1.ChangeDutyCycle(speed)
        self.pwm2.ChangeDutyCycle(speed)

    def turn_right(self, speed: float):
        """Turns the robot right at a given speed."""
        speed = self._validate_and_adjust_speed(speed)
        logger.info(f"MotorController: turn_right called with speed: {speed}")
        GPIO.output(self.MOTOR1_PIN1, GPIO.HIGH) # Left motor forward
        GPIO.output(self.MOTOR1_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN1, GPIO.LOW)  # Right motor backward
        GPIO.output(self.MOTOR2_PIN2, GPIO.HIGH)
        self.pwm1.ChangeDutyCycle(speed)
        self.pwm2.ChangeDutyCycle(speed)

    def _validate_and_adjust_speed(self, speed: float) -> float:
        """Validate and adjust the speed to be within the valid range (0-100 for these direct methods)"""
        # Speed for these direct methods should be positive, direction is handled by pin logic
        return max(0, min(100, abs(speed)))

    # _set_motor_pins method is removed as its logic is now direct in drive/turn methods
    # def _set_motor_pins(self, pin1: int, pin2: int):
    #     """Set motor directions based on the given pins"""
    #     GPIO.output(self.MOTOR1_PIN1, GPIO.HIGH if pin1 > 0 else GPIO.LOW)
    #     GPIO.output(self.MOTOR1_PIN2, GPIO.LOW if pin1 > 0 else GPIO.HIGH)
    #     GPIO.output(self.MOTOR2_PIN1, GPIO.HIGH if pin2 > 0 else GPIO.LOW)
    #     GPIO.output(self.MOTOR2_PIN2, GPIO.LOW if pin2 > 0 else GPIO.HIGH) 