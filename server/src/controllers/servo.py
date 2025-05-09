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
        GPIO.output(self.pin, GPIO.LOW) # Explicitly set pin LOW
        logger.info(f"ServoController initialized. Position set to: {self.current_position:.2f} degrees. PWM stopped, pin set LOW.")

    def _angle_to_duty_cycle(self, angle: float) -> float:
        """Converts an angle (0-180 nominally) to a PWM duty cycle (2.5-12.5 for SG90 type servos)."""
        # Standard SG90 servo: 0 deg = 2.5% duty, 180 deg = 12.5% duty.
        # Ensure angle is within a reasonable servo operating range if necessary, though clamping happens in set_position.
        return 2.5 + (angle / 180.0) * 10.0
    
    def set_position(self, target_angle: float, blocking: bool = True):
        target_angle = max(self.min_angle, min(self.max_angle, target_angle))
        current_angle_for_log = self.current_position # For logging clarity
        logger.info(f"Servo (Direct Mode Test): Requested move from {current_angle_for_log:.2f} to {target_angle:.2f} degrees.")

        # Ensure previous PWM is stopped (belt-and-suspenders, should be stopped already)
        try:
            self.pwm.stop()
            GPIO.output(self.pin, GPIO.LOW) # Ensure pin is low if pwm was running
        except Exception: # May occur if not started, or on first call if somehow uninitialized
            pass

        # Start PWM directly at the final target duty cycle
        target_duty_cycle = self._angle_to_duty_cycle(target_angle)
        logger.info(f"Servo (Direct Mode Test): Starting PWM with duty cycle for target {target_angle:.2f} degrees.")
        self.pwm.start(target_duty_cycle)
        
        # Update software position immediately as we assume it will go there
        self.current_position = target_angle 

        if blocking:
            # Estimate travel time. SG90 speed is ~0.1s/60deg. Typical range 0-90 deg.
            # Max travel could be 90 deg. So max time ~0.15s for 90 deg.
            # Let's use a fixed, somewhat generous sleep to ensure it can reach for testing.
            # This is NOT smooth movement.
            estimated_travel_time = 0.5 # seconds, adjust if needed for full sweep
            logger.info(f"Servo (Direct Mode Test): Sleeping for {estimated_travel_time}s to allow travel.")
            time.sleep(estimated_travel_time)

        # Stop PWM
        logger.info(f"Servo (Direct Mode Test): Stopping PWM after attempted move to {self.current_position:.2f}.")
        self.pwm.stop()
        GPIO.output(self.pin, GPIO.LOW) # Explicitly set pin LOW
        logger.info(f"Servo (Direct Mode Test): Move from {current_angle_for_log:.2f} to {self.current_position:.2f} complete. PWM stopped, pin set LOW.")
    
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
        # Ensure PWM is stopped before cleaning up GPIO
        try:
            self.pwm.stop()
            GPIO.output(self.pin, GPIO.LOW) # Explicitly set pin LOW
        except Exception: # pwm might not be initialized if __init__ failed
            pass 
        GPIO.cleanup([self.pin]) # Uncomment
        logger.info("Servo: Cleanup called. PWM stopped, pin set LOW, and GPIO cleaned.") # Revert log 