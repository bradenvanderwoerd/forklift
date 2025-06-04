# Configuration settings for the Forklift Client Application

# --- Servo Pin Definitions (Matching Server-Side) ---
# These constants define the BCM pin numbers used for each servo on the Raspberry Pi.
# They must match the `FORK_SERVO_X_PIN` values in `server/src/utils/config.py`
# The comments like "UI '1'" indicate which servo is controlled by keys '1' through '6' in the client UI.

FORK_SERVO_A_PIN = 13 # Controlled by UI Key '1' (Typically Bottom-Left in a 3x2 grid display)
FORK_SERVO_B_PIN = 25 # Controlled by UI Key '2' (Typically Bottom-Right)
FORK_SERVO_C_PIN = 26 # Controlled by UI Key '3' (Typically Middle-Left)
FORK_SERVO_D_PIN = 27 # Controlled by UI Key '4' (Typically Middle-Right)
FORK_SERVO_E_PIN = 18 # Controlled by UI Key '5' (Typically Top-Left)
FORK_SERVO_F_PIN = 16 # Controlled by UI Key '6' (Typically Top-Right)

# --- UI Servo Mapping --- 
# `UI_SERVO_ORDER_MAPPING` provides an ordered list of servo pins.
# This specific order determines how servos are displayed or iterated in the UI
# if they are shown in a sequence or grid. The client UI uses a 3x2 grid,
# and this list maps the key presses '1' through '6' to these pins sequentially.
# The grid visualization mentioned in comments was:
# 5 (E)  6 (F)
# 3 (C)  4 (D)
# 1 (A)  2 (B)
# This list ensures that `UI_SERVO_ORDER_MAPPING[0]` is Servo A (key '1'),
# `UI_SERVO_ORDER_MAPPING[1]` is Servo B (key '2'), and so on.
UI_SERVO_ORDER_MAPPING = [
    FORK_SERVO_A_PIN,  # Corresponds to UI key '1'
    FORK_SERVO_B_PIN,  # Corresponds to UI key '2'
    FORK_SERVO_C_PIN,  # Corresponds to UI key '3'
    FORK_SERVO_D_PIN,  # Corresponds to UI key '4'
    FORK_SERVO_E_PIN,  # Corresponds to UI key '5'
    FORK_SERVO_F_PIN   # Corresponds to UI key '6'
]

# `SERVO_KEY_TO_PIN_MAPPING` provides a direct dictionary lookup from the character
# representation of the number key (e.g., '1', '2') to the corresponding servo pin.
# This is used in the client's main window to select a servo based on key presses.
SERVO_KEY_TO_PIN_MAPPING = {
    '1': FORK_SERVO_A_PIN,
    '2': FORK_SERVO_B_PIN,
    '3': FORK_SERVO_C_PIN,
    '4': FORK_SERVO_D_PIN,
    '5': FORK_SERVO_E_PIN,
    '6': FORK_SERVO_F_PIN
} 

# --- Direct Warehouse Camera Connection Configuration ---
# These settings are used by the client application when it attempts to connect
# directly to the warehouse camera feed. This direct connection bypasses the
# Raspberry Pi server for the overhead video, potentially offering lower latency
# for teleoperation when the processed feed from the Pi (with overlays) is not essential.

# Host (IP address) of the machine running the warehouse camera's dedicated video server.
# This should be the IP address of the computer that has the warehouse camera connected to it
# and is running a service that streams its video (e.g., a simple MJPEG streamer or custom socket server).
DIRECT_WAREHOUSE_CAMERA_HOST = "192.168.0.100"

# Port number on the `DIRECT_WAREHOUSE_CAMERA_HOST` that the warehouse camera's
# video stream service is listening on.
DIRECT_WAREHOUSE_CAMERA_PORT = 5001 