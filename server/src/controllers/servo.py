from ..utils.config import (
    SERVO_PWM_PIN, FORK_DOWN_POSITION, FORK_UP_POSITION, 
    SERVO_STEP_DEGREES, SERVO_STEP_DELAY_SECONDS, AUTONAV_FORK_LOWER_TO_PICKUP_ANGLE,
    AUTONAV_FORK_CARRY_ANGLE
)
import pigpio # Changed from RPi.GPIO
import time
from typing import Optional
import logging

logger = logging.getLogger(__name__)

# Standard Servo Pulse Widths
SERVO_MIN_PULSEWIDTH = 500  # Microseconds for 0 degrees (typical)
SERVO_MAX_PULSEWIDTH = 2500 # Microseconds for 180 degrees (typical)

class ServoController:
    def __init__(self):
        self.pin = SERVO_PWM_PIN
        # Initialize to the default FORK_DOWN_POSITION (now 80)
        self.current_position_degrees = FORK_DOWN_POSITION 
        
        self.step_degrees = SERVO_STEP_DEGREES
        self.step_delay_seconds = SERVO_STEP_DELAY_SECONDS
        
        # Set min/max angles. Min is FORK_UP_POSITION (0). Max is FORK_DOWN_POSITION (80).
        self.min_angle_degrees = FORK_UP_POSITION 
        self.max_angle_degrees = FORK_DOWN_POSITION # This is 80, same as AUTONAV_FORK_LOWER_TO_PICKUP_ANGLE

        try:
            self.pi = pigpio.pi() 
            if not self.pi.connected:
                raise RuntimeError("Failed to connect to pigpiod. Is the daemon running?")
        except Exception as e:
            logger.error(f"pigpio initialization failed: {e}. Servo will not function.")
            self.pi = None # Ensure pi is None if connection failed
            # Potentially re-raise or handle more gracefully depending on desired app behavior
            return

        self.pi.set_mode(self.pin, pigpio.OUTPUT) # Set pin as output
        
        # Initialize servo to its starting position then stop pulses
        initial_pulsewidth = self._angle_to_pulsewidth(self.current_position_degrees)
        self.pi.set_servo_pulsewidth(self.pin, initial_pulsewidth)
        time.sleep(0.5) # Allow time for servo to reach initial position
        self.pi.set_servo_pulsewidth(self.pin, 0) # Stop sending pulses
        logger.info(f"ServoController initialized with pigpio. Pin: {self.pin}, Initial Pos: {self.current_position_degrees:.2f} deg. Pulses stopped.")

    def _angle_to_pulsewidth(self, angle_degrees: float) -> int:
        """Converts an angle in degrees to a servo pulse width in microseconds."""
        # Clamp angle to 0-180 for the general conversion formula
        clamped_angle = max(0, min(180, angle_degrees))
        pulsewidth = SERVO_MIN_PULSEWIDTH + (clamped_angle / 180.0) * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH)
        return int(round(pulsewidth))

    def set_position(self, target_angle_degrees: float, blocking: bool = True):
        if not self.pi or not self.pi.connected:
            logger.error("pigpio not connected. Cannot set servo position.")
            return

        # Clamp target_angle_degrees to the controller's defined min/max operational range
        target_angle_degrees = max(self.min_angle_degrees, min(self.max_angle_degrees, target_angle_degrees))
        
        logger.info(f"Servo: Moving from {self.current_position_degrees:.2f} to {target_angle_degrees:.2f} degrees.")

        # If already very close, set final pulse width and stop pulses
        if abs(self.current_position_degrees - target_angle_degrees) < self.step_degrees / 2.0:
            logger.debug("Servo: Already at target angle or very close.")
            final_pulsewidth = self._angle_to_pulsewidth(target_angle_degrees)
            self.pi.set_servo_pulsewidth(self.pin, final_pulsewidth)
            self.current_position_degrees = target_angle_degrees # Update software position
            if blocking: # Allow a moment for the final set to take effect
                time.sleep(max(self.step_delay_seconds, 0.05))
            self.pi.set_servo_pulsewidth(self.pin, 0) # Stop sending pulses
            logger.info(f"Servo: At target {self.current_position_degrees:.2f} deg. Pulses stopped.")
            return

        # Stepping logic
        if target_angle_degrees > self.current_position_degrees:
            direction = 1
        else:
            direction = -1
        
        num_steps = int(abs(target_angle_degrees - self.current_position_degrees) / self.step_degrees)
        if num_steps == 0 and self.current_position_degrees != target_angle_degrees: 
            num_steps = 1

        for i in range(num_steps):
            next_step_angle = self.current_position_degrees + (direction * self.step_degrees)
            # Ensure we don't overshoot the final target_angle_degrees with the last step
            if (direction == 1 and next_step_angle > target_angle_degrees) or \
               (direction == -1 and next_step_angle < target_angle_degrees):
                next_step_angle = target_angle_degrees
            
            step_pulsewidth = self._angle_to_pulsewidth(next_step_angle)
            self.pi.set_servo_pulsewidth(self.pin, step_pulsewidth)
            self.current_position_degrees = next_step_angle 
            if blocking:
                time.sleep(self.step_delay_seconds)
        
        # Ensure final pulse width is set for the exact target angle
        final_target_pulsewidth = self._angle_to_pulsewidth(target_angle_degrees)
        self.pi.set_servo_pulsewidth(self.pin, final_target_pulsewidth)
        self.current_position_degrees = target_angle_degrees # Final update of software position
        
        # Allow servo to reach the position and then stop sending pulses
        if blocking:
            final_settle_delay = max(self.step_delay_seconds, 0.3) # Use the previously determined good delay
            time.sleep(final_settle_delay)
        
        self.pi.set_servo_pulsewidth(self.pin, 0) # Stop sending pulses
        logger.info(f"Servo: Reached target {self.current_position_degrees:.2f} deg. Pulses stopped.")
    
    def go_to_down_position(self, blocking: bool = True):
        logger.info("Servo: Moving to FORK_DOWN_POSITION.") # Will target 80
        self.set_position(FORK_DOWN_POSITION, blocking=blocking)

    def go_to_up_position(self, blocking: bool = True):
        logger.info("Servo: Moving to FORK_UP_POSITION.") # Will target 0
        self.set_position(FORK_UP_POSITION, blocking=blocking)

    def get_position(self) -> float:
        return self.current_position_degrees
    
    def cleanup(self):
        logger.info("Servo: Cleanup called. Stopping pulses and pigpio connection.")
        if self.pi and self.pi.connected:
            try:
                self.pi.set_servo_pulsewidth(self.pin, 0) # Turn servo off
                self.pi.stop() # Disconnect from pigpiod
            except Exception as e:
                logger.error(f"Error during pigpio cleanup: {e}")
        # No RPi.GPIO.cleanup needed here 