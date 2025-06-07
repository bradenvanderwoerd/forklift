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

# The PIDController class previously here was not used by MotorController's active methods.
# Navigation-level PID control is handled in NavigationController.
# If per-motor PID becomes necessary, it can be re-evaluated.

class MotorController:
    """Controls the robot's two motors for differential drive.

    This class handles low-level motor operations including setting speed and direction
    for individual motors and providing a combined `move` method for differential steering.
    It also includes basic methods for direct forward, backward, left, and right movements.
    GPIO pins are configured based on `server/src/utils/config.py`.
    """
    def __init__(self):
        """Initializes the motor controller, sets up GPIO pins, and starts PWM.
        GPIO.setmode(GPIO.BCM) and GPIO.setwarnings(False) should be called once globally
        before initializing this controller (e.g., in the main server script).
        """
        
        # Motor pins from config
        self.MOTOR1_PIN1 = MOTOR_LEFT_FORWARD_PIN    # Left motor forward enable
        self.MOTOR1_PIN2 = MOTOR_LEFT_BACKWARD_PIN   # Left motor backward enable
        self.MOTOR2_PIN1 = MOTOR_RIGHT_FORWARD_PIN   # Right motor forward enable
        self.MOTOR2_PIN2 = MOTOR_RIGHT_BACKWARD_PIN  # Right motor backward enable
        
        # PWM pins from config (for speed control)
        self.MOTOR1_PWM = MOTOR_LEFT_PWM_PIN       # Left motor PWM input
        self.MOTOR2_PWM = MOTOR_RIGHT_PWM_PIN      # Right motor PWM input
        
        self._setup_pins()
        
        # Initialize PWM objects at 1kHz frequency
        self.pwm1 = GPIO.PWM(self.MOTOR1_PWM, 1000) 
        self.pwm2 = GPIO.PWM(self.MOTOR2_PWM, 1000)
        # Start PWM with 0% duty cycle (motors off)
        self.pwm1.start(0)
        self.pwm2.start(0)
        
        # Obsolete attributes from previous PID logic, removed as `move` is primary.
        # self.current_speed = 0 
        # self.pid_controller = PIDController() 
        
    def _setup_pins(self):
        """Sets up all GPIO pins used by the motor controller as outputs."""
        pins = [
            self.MOTOR1_PIN1, self.MOTOR1_PIN2, 
                self.MOTOR2_PIN1, self.MOTOR2_PIN2,
            self.MOTOR1_PWM, self.MOTOR2_PWM
        ]
        
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW) # Initialize pins to LOW
    
    def _set_motor_speed(self, motor_pwm_obj: GPIO.PWM, pin_forward: int, pin_backward: int, speed: int):
        """Sets the speed and direction for a single motor.

        Args:
            motor_pwm_obj: The RPi.GPIO.PWM object for the motor.
            pin_forward: The GPIO pin number for forward motion of this motor.
            pin_backward: The GPIO pin number for backward motion of this motor.
            speed: The desired speed for this motor, ranging from -100 (full backward)
                   to 100 (full forward). 0 means stop.
        """
        speed = int(max(-100, min(100, speed))) # Clamp speed to [-100, 100]

        if speed > 0: # Forward
            GPIO.output(pin_forward, GPIO.HIGH)
            GPIO.output(pin_backward, GPIO.LOW)
            motor_pwm_obj.ChangeDutyCycle(speed)
        elif speed < 0: # Backward
            GPIO.output(pin_forward, GPIO.LOW)
            GPIO.output(pin_backward, GPIO.HIGH)
            motor_pwm_obj.ChangeDutyCycle(abs(speed))
        else: # Stop (speed is 0)
            GPIO.output(pin_forward, GPIO.LOW)
            GPIO.output(pin_backward, GPIO.LOW)
            motor_pwm_obj.ChangeDutyCycle(0)

    def move(self, forward_component: int, turn_component: int):
        """Controls the robot's movement using differential drive mixing.

        This method takes separate forward/backward and turning components and calculates
        the required speed for each motor.

        Args:
            forward_component: Speed for overall forward or backward movement (-100 to 100).
                               Positive values move the robot forward, negative values backward.
            turn_component: Speed and direction for turning (-100 to 100).
                            Positive values make the robot turn left (Counter-Clockwise).
                            Negative values make the robot turn right (Clockwise).
                            A value of 0 results in no turning (straight movement).
        """
        # logger.debug(f"MotorController.move: FwdCmp:{forward_component}, TurnCmp:{turn_component}")

        # Standard differential drive mixing:
        # LeftMotorSpeed = ForwardComponent - TurnComponent
        # RightMotorSpeed = ForwardComponent + TurnComponent
        # With this scheme:
        # - Positive turn_component: Decreases left motor speed, increases right motor speed -> Turns Left (CCW).
        # - Negative turn_component: Increases left motor speed, decreases right motor speed -> Turns Right (CW).

        left_speed = forward_component - turn_component
        right_speed = forward_component + turn_component

        # Clamp individual motor speeds to the operational range [-100, 100]
        left_speed = int(max(-100, min(100, left_speed)))
        right_speed = int(max(-100, min(100, right_speed)))
        
        # logger.debug(f"MotorController.move: CalLSpd:{left_speed}, CalRSpd:{right_speed}")

        self._set_motor_speed(self.pwm1, self.MOTOR1_PIN1, self.MOTOR1_PIN2, left_speed)
        self._set_motor_speed(self.pwm2, self.MOTOR2_PIN1, self.MOTOR2_PIN2, right_speed)
    
    def set_speed(self, speed: float):
        """(DEPRECATED) Sets the same speed for both motors for straight movement.
        
        This method is deprecated. Use `move(forward_component=speed, turn_component=0)` 
        for straight forward movement, or `move(forward_component=-speed, turn_component=0)`
        for straight backward movement.
        
        Args:
            speed: Target speed for both motors (-100 to 100). Positive for forward,
                   negative for backward (though direct application here assumes positive for PWM).
        """
        logger.warning("MotorController.set_speed() is deprecated. Use move() for better control.")
        # For basic compatibility, interpret speed as a forward/backward command.
        clamped_speed = int(max(-100, min(100, speed)))
        self._set_motor_speed(self.pwm1, self.MOTOR1_PIN1, self.MOTOR1_PIN2, clamped_speed)
        self._set_motor_speed(self.pwm2, self.MOTOR2_PIN1, self.MOTOR2_PIN2, clamped_speed)
    
    def _apply_speed(self, speed: float):
        """(DEPRECATED) Apply a given speed to both motors.
        
        This method is deprecated. Functionality is covered by `_set_motor_speed` 
        and the logic within `move` or the simplified `set_speed`.
        
        Args:
            speed: Speed value for both motors.
        """
        logger.warning("MotorController._apply_speed() is deprecated and effectively a duplicate.")
        clamped_speed = int(max(-100, min(100, speed)))
        self._set_motor_speed(self.pwm1, self.MOTOR1_PIN1, self.MOTOR1_PIN2, clamped_speed)
        self._set_motor_speed(self.pwm2, self.MOTOR2_PIN1, self.MOTOR2_PIN2, clamped_speed)
    
    def stop(self):
        """Stops all motor activity immediately.
        
        Sets PWM duty cycles to 0 and motor direction pins to LOW.
        """
        logger.info("MotorController.stop() called - setting PWMs to 0 and direction pins to LOW.")
        self.pwm1.ChangeDutyCycle(0)
        self.pwm2.ChangeDutyCycle(0)
        GPIO.output(self.MOTOR1_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR1_PIN2, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN1, GPIO.LOW)
        GPIO.output(self.MOTOR2_PIN2, GPIO.LOW)
    
    def cleanup(self):
        """Stops motors and cleans up GPIO resources used by this controller.
        
        Should be called when the motor controller is no longer needed to ensure
        GPIO pins are released properly.
        """
        logger.info("MotorController.cleanup() called.")
        self.stop()
        self.pwm1.stop() # Stop PWM generation
        self.pwm2.stop()
        
        # GPIO.cleanup() by default cleans all channels. 
        # It's often better to clean up only the channels this class used,
        # especially if other parts of the application use GPIO.
        # However, a global GPIO.cleanup() is typically called at the very end of the application.
        # For robustness here, we can list the pins this class set up.
        pins_this_controller_used = [
            self.MOTOR1_PIN1, self.MOTOR1_PIN2, self.MOTOR1_PWM,
            self.MOTOR2_PIN1, self.MOTOR2_PIN2, self.MOTOR2_PWM
        ]
        # GPIO.cleanup(pins_this_controller_used) # Replaced with more specific cleanup
        # Attempt to clean up specific pins. If global cleanup is used later, this might be redundant but safe.
        # No, standard practice is that the main program calls GPIO.cleanup() once.
        # This cleanup method should only stop its own activities.
        logger.info("MotorController GPIO resources (PWM) stopped. General GPIO.cleanup() expected elsewhere.")

    def drive_forward(self, speed: int):
        """(SIMPLIFIED/MANUAL) Drives the robot straight forward at a given speed percentage.

        Args:
            speed: Speed percentage (0-100). Negative values will be treated as 0.
        """
        safe_speed = self._validate_and_adjust_speed_positive(speed)
        logger.info(f"MotorController: drive_forward (manual) called with speed: {safe_speed}")
        self.move(forward_component=safe_speed, turn_component=0)

    def drive_backward(self, speed: int):
        """(SIMPLIFIED/MANUAL) Drives the robot straight backward at a given speed percentage.

        Args:
            speed: Speed percentage (0-100). Negative values will be treated as 0.
        """
        safe_speed = self._validate_and_adjust_speed_positive(speed)
        logger.info(f"MotorController: drive_backward (manual) called with speed: {safe_speed}")
        self.move(forward_component=-safe_speed, turn_component=0)

    def turn_left(self, speed: int):
        """(SIMPLIFIED/MANUAL) Turns the robot left (on the spot) at a given speed percentage.

        Args:
            speed: Speed percentage (0-100) for the turning maneuver.
        """
        safe_speed = self._validate_and_adjust_speed_positive(speed)
        logger.info(f"MotorController: turn_left (manual) called with speed: {safe_speed}")
        # For on-the-spot left turn: Left motor backward, Right motor forward
        # Using move(): forward_component=0, turn_component=positive
        self.move(forward_component=0, turn_component=safe_speed)

    def turn_right(self, speed: int):
        """(SIMPLIFIED/MANUAL) Turns the robot right (on the spot) at a given speed percentage.

        Args:
            speed: Speed percentage (0-100) for the turning maneuver.
        """
        safe_speed = self._validate_and_adjust_speed_positive(speed)
        logger.info(f"MotorController: turn_right (manual) called with speed: {safe_speed}")
        # For on-the-spot right turn: Left motor forward, Right motor backward
        # Using move(): forward_component=0, turn_component=negative
        self.move(forward_component=0, turn_component=-safe_speed)

    def _validate_and_adjust_speed_positive(self, speed: int) -> int:
        """Validates and clamps speed to be a positive integer between 0 and 100.
           Used by simplified manual drive methods.
        Args:
            speed: The input speed value.
        Returns:
            A speed value clamped between 0 and 100.
        """
        return int(max(0, min(100, abs(speed))))

    # _set_motor_pins method is removed as its logic is now direct in drive/turn methods
    # def _set_motor_pins(self, pin1: int, pin2: int):
    #     """Set motor directions based on the given pins"""
    #     GPIO.output(self.MOTOR1_PIN1, GPIO.HIGH if pin1 > 0 else GPIO.LOW)
    #     GPIO.output(self.MOTOR1_PIN2, GPIO.LOW if pin1 > 0 else GPIO.HIGH)
    #     GPIO.output(self.MOTOR2_PIN1, GPIO.HIGH if pin2 > 0 else GPIO.LOW)
    #     GPIO.output(self.MOTOR2_PIN2, GPIO.LOW if pin2 > 0 else GPIO.HIGH) 