# Servo Pin Definitions (matching server-side config.py)
# These are the actual BCM pin numbers on the Raspberry Pi.

FORK_SERVO_A_PIN = 13 # UI '1' (Bottom-Left)
FORK_SERVO_B_PIN = 25 # UI '2' (Bottom-Right)
FORK_SERVO_C_PIN = 26 # UI '3' (Middle-Left)
FORK_SERVO_D_PIN = 27 # UI '4' (Middle-Right)
FORK_SERVO_E_PIN = 18 # UI '5' (Top-Left)
FORK_SERVO_F_PIN = 16 # UI '6' (Top-Right)

# Ordered list of servo pins for UI mapping (1-6)
# Index 0 corresponds to UI '1', Index 1 to UI '2', etc.
# This order matches the requested 3x2 grid:
# 5 (E)  6 (F)
# 3 (C)  4 (D)
# 1 (A)  2 (B)
# So, the list should be [A, B, C, D, E, F] to match key '1' to A, '2' to B etc.
UI_SERVO_ORDER_MAPPING = [
    FORK_SERVO_A_PIN,  # Corresponds to '1'
    FORK_SERVO_B_PIN,  # Corresponds to '2'
    FORK_SERVO_C_PIN,  # Corresponds to '3'
    FORK_SERVO_D_PIN,  # Corresponds to '4'
    FORK_SERVO_E_PIN,  # Corresponds to '5'
    FORK_SERVO_F_PIN   # Corresponds to '6'
]

# For direct mapping from Qt.Key_1 to pin:
SERVO_KEY_TO_PIN_MAPPING = {
    '1': FORK_SERVO_A_PIN,
    '2': FORK_SERVO_B_PIN,
    '3': FORK_SERVO_C_PIN,
    '4': FORK_SERVO_D_PIN,
    '5': FORK_SERVO_E_PIN,
    '6': FORK_SERVO_F_PIN
} 