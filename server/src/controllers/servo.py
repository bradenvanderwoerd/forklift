from ..utils.config import (
    FORK_SERVO_A_PIN, FORK_DOWN_POSITION, FORK_UP_POSITION, 
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
    """Controls a single servo motor using the pigpio library for precise PWM.

    This class manages the position of a servo, converting angles in degrees to
    the appropriate pulse widths. It supports setting an initial position, moving
    to defined up/down positions, and stepping smoothly to a target angle.
    Pulses are stopped after reaching the target to prevent jitter and save power.

    Attributes:
        pin (int): The BCM GPIO pin number the servo is connected to.
        current_position_degrees (float): The last commanded position of the servo in degrees.
        defined_up_angle (float): The servo's specific "up" limit in degrees.
        defined_down_angle (float): The servo's specific "down" limit in degrees.
        min_angle_degrees (float): The minimum operational angle for this servo.
        max_angle_degrees (float): The maximum operational angle for this servo.
        pi (Optional[pigpio.pi]): The pigpio library instance.
    """
    def __init__(self, pin_number: int, initial_position_degrees: float, defined_up_angle: float, defined_down_angle: float):
        """Initializes the ServoController.

        Args:
            pin_number: The BCM GPIO pin the servo is connected to.
            initial_position_degrees: The angle (degrees) to set the servo to upon initialization.
            defined_up_angle: The servo's specific upper position limit in degrees.
            defined_down_angle: The servo's specific lower position limit in degrees.
        """
        self.pin = pin_number
        self.current_position_degrees = initial_position_degrees
        
        self.defined_up_angle = defined_up_angle
        self.defined_down_angle = defined_down_angle

        # Determine operational min/max based on the defined up/down angles
        self.min_angle_degrees = min(self.defined_up_angle, self.defined_down_angle)
        self.max_angle_degrees = max(self.defined_up_angle, self.defined_down_angle)
        
        self.step_degrees = SERVO_STEP_DEGREES # Still used by set_position for smooth stepping
        self.step_delay_seconds = SERVO_STEP_DELAY_SECONDS

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
        """Converts an angle in degrees to a servo pulse width in microseconds.
        
        The angle is first clamped to the general 0-180 degree range for the formula,
        though operational limits are enforced by `min_angle_degrees` and `max_angle_degrees` 
        in the `set_position` method.

        Args:
            angle_degrees: The desired angle in degrees.

        Returns:
            The corresponding pulse width in microseconds.
        """
        # Clamp angle to 0-180 for the general pulse width conversion formula.
        # Specific servo operational limits are handled by min_angle_degrees/max_angle_degrees in set_position.
        clamped_angle_for_formula = max(0.0, min(180.0, angle_degrees))
        
        pulsewidth = SERVO_MIN_PULSEWIDTH + 
                     (clamped_angle_for_formula / 180.0) * (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH)
        return int(round(pulsewidth))

    def set_position(self, target_angle_degrees: float, blocking: bool = True):
        """Moves the servo to the target angle, optionally blocking until movement is complete.

        The target angle is clamped to the servo's defined operational range 
        (`self.min_angle_degrees` to `self.max_angle_degrees`).
        The movement is performed in steps defined by `self.step_degrees` and 
        `self.step_delay_seconds` for smoother motion.

        Args:
            target_angle_degrees: The desired target angle for the servo in degrees.
            blocking: If True, the method will wait for the servo to complete its movement.
                      If False, the method will initiate movement and return immediately.
                      Note: Even with blocking=False, there are small sleeps for pulse sending.
        """
        if not self.pi or not self.pi.connected:
            logger.error(f"pigpio not connected for servo on pin {self.pin}. Cannot set position.")
            return

        target_angle_degrees = float(target_angle_degrees)
        # Clamp target_angle_degrees to this servo's specific operational range.
        clamped_target_angle = max(self.min_angle_degrees, min(self.max_angle_degrees, target_angle_degrees))
        
        if clamped_target_angle != target_angle_degrees:
            logger.warning(f"Servo on pin {self.pin}: Target angle {target_angle_degrees:.2f} deg was clamped to {clamped_target_angle:.2f} deg (Op Min/Max: {self.min_angle_degrees:.1f}/{self.max_angle_degrees:.1f})")
        target_angle_degrees = clamped_target_angle
        
        logger.info(f"Servo on pin {self.pin}: Moving from {self.current_position_degrees:.2f} to {target_angle_degrees:.2f} degrees.")

        # If already very close to the target, set final pulse width and stop.
        # Using a small tolerance (e.g., half a step degree or a fixed small epsilon like 0.1).
        if abs(self.current_position_degrees - target_angle_degrees) < 0.1: # Use a small epsilon
            logger.debug(f"Servo on pin {self.pin}: Already at or very close to target angle {target_angle_degrees:.2f}.")
            final_pulsewidth = self._angle_to_pulsewidth(target_angle_degrees)
            self.pi.set_servo_pulsewidth(self.pin, final_pulsewidth)
            self.current_position_degrees = target_angle_degrees
            if blocking:
                time.sleep(max(self.step_delay_seconds, 0.05)) # Brief moment for final set.
            self.pi.set_servo_pulsewidth(self.pin, 0) # Stop sending pulses.
            logger.info(f"Servo on pin {self.pin}: Confirmed at target {self.current_position_degrees:.2f} deg. Pulses stopped.")
            return

        # Determine direction and number of steps for smooth movement.
        if target_angle_degrees > self.current_position_degrees:
            direction = 1
        else:
            direction = -1
        
        num_steps = int(round(abs(target_angle_degrees - self.current_position_degrees) / self.step_degrees))
        # Ensure at least one step if not already at target, to handle small discrepancies.
        if num_steps == 0 and abs(self.current_position_degrees - target_angle_degrees) > 0.01: 
            num_steps = 1

        current_software_angle = self.current_position_degrees
        for i in range(num_steps):
            next_step_angle = current_software_angle + (direction * self.step_degrees)
            # Ensure the last step lands exactly on target_angle_degrees if it would overshoot.
            if (direction == 1 and next_step_angle > target_angle_degrees) or \
               (direction == -1 and next_step_angle < target_angle_degrees) or \
               (i == num_steps - 1): # Always ensure the last step is to the final target.
                next_step_angle = target_angle_degrees
            
            step_pulsewidth = self._angle_to_pulsewidth(next_step_angle)
            self.pi.set_servo_pulsewidth(self.pin, step_pulsewidth)
            current_software_angle = next_step_angle 
            if blocking:
                time.sleep(self.step_delay_seconds)
            if current_software_angle == target_angle_degrees: # Reached target during steps
                break
        
        # Final set to ensure exact target position and update current_position_degrees.
        final_target_pulsewidth = self._angle_to_pulsewidth(target_angle_degrees)
        self.pi.set_servo_pulsewidth(self.pin, final_target_pulsewidth)
        self.current_position_degrees = target_angle_degrees
        
        if blocking:
            # Allow servo time to physically reach the final position.
            # This delay might need tuning based on servo speed and load.
            final_settle_delay = max(self.step_delay_seconds, 0.2) # Increased default settle for blocking
            time.sleep(final_settle_delay)
        
        self.pi.set_servo_pulsewidth(self.pin, 0) # Stop sending pulses.
        logger.info(f"Servo on pin {self.pin}: Reached target {self.current_position_degrees:.2f} deg. Pulses stopped.")
    
    def go_to_down_position(self, blocking: bool = True):
        """Moves the servo to its pre-configured 'down' position.
        The 'down' position is `self.defined_down_angle` set during initialization.

        Args:
            blocking: If True, waits for movement to complete.
        """
        logger.info(f"Servo (pin {self.pin}): Moving to DEFINED DOWN position: {self.defined_down_angle} deg.")
        self.set_position(self.defined_down_angle, blocking=blocking)

    def go_to_up_position(self, blocking: bool = True):
        """Moves the servo to its pre-configured 'up' position.
        The 'up' position is `self.defined_up_angle` set during initialization.

        Args:
            blocking: If True, waits for movement to complete.
        """
        logger.info(f"Servo (pin {self.pin}): Moving to DEFINED UP position: {self.defined_up_angle} deg.")
        self.set_position(self.defined_up_angle, blocking=blocking)

    def go_to_autonav_carry_position(self, blocking: bool = True):
        """Moves the servo to the `AUTONAV_FORK_CARRY_ANGLE` (for primary autonav fork).
        
        Args:
            blocking: If True, waits for movement to complete.
        """
        logger.info(f"Servo: Moving to AUTONAV_FORK_CARRY_ANGLE ({AUTONAV_FORK_CARRY_ANGLE} deg).")
        self.set_position(AUTONAV_FORK_CARRY_ANGLE, blocking=blocking)
    
    def get_position(self) -> float:
        """Returns the current commanded position of the servo in degrees."""
        return self.current_position_degrees
    
    def cleanup(self):
        """Stops sending pulses to the servo and releases the pigpio connection.
        Should be called when the servo controller is no longer needed.
        """
        logger.info("Servo: Cleanup called. Stopping pulses and pigpio connection.")
        if self.pi and self.pi.connected:
            try:
                self.pi.set_servo_pulsewidth(self.pin, 0) # Turn servo off
                self.pi.stop() # Disconnect from pigpiod
            except Exception as e:
                logger.error(f"Error during pigpio cleanup: {e}")
        # No RPi.GPIO.cleanup needed here 