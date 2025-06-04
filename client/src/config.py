import os
from dotenv import load_dotenv
import logging

# Set up basic logging configuration for the application.
# Messages with level INFO and above will be logged.
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# --- Environment Variable Loading ---
# Construct the path to the .env file, which is expected to be in the project's root directory.
# Example: if this file is /path/to/project/client/src/config.py,
# .env is expected at /path/to/project/.env
env_path = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(__file__))), ".env" # Go up three levels for project root
)
logger.info(f"Attempting to load environment variables from: {env_path}")
if os.path.exists(env_path):
    load_dotenv(env_path)
    logger.info(f"Successfully loaded environment variables from {env_path}")
else:
    logger.warning(f".env file not found at {env_path}. Using default configurations.")

# --- Server Connection Configuration ---
# These settings define how the client application connects to the server running on the Raspberry Pi.
# Values are loaded from environment variables if set, otherwise defaults are used.

# Hostname or IP address of the Raspberry Pi server.
# Default: "localhost" (assumes server is running on the same machine as the client, for development).
SERVER_HOST = os.getenv("SERVER_HOST", "localhost")

# Port for the command WebSocket server on the Raspberry Pi.
# This is used for sending control commands (motor, servo, etc.) and receiving status updates.
# Default: 3456
COMMAND_PORT = int(os.getenv("COMMAND_PORT", "3456"))

# Port for the onboard camera video WebSocket server on the Raspberry Pi.
# This stream provides the video feed from the camera mounted on the forklift.
# Default: 3457
VIDEO_PORT = int(os.getenv("VIDEO_PORT", "3457"))

# Port for the processed overhead warehouse camera video WebSocket server on the Raspberry Pi.
# This stream provides the video feed from the overhead camera, potentially with ArUco marker processing.
# Default: 3458
OVERHEAD_VIDEO_PORT = int(os.getenv("OVERHEAD_VIDEO_PORT", "3458"))

# Port for the direct overhead warehouse camera video feed.
# This allows the client to connect directly to the warehouse camera source (e.g., an IP camera).
# Note: This is the port of the *external camera device*, not the Raspberry Pi.
# Default: 5001
DIRECT_WAREHOUSE_VIDEO_PORT = int(os.getenv("DIRECT_WAREHOUSE_VIDEO_PORT", "5001"))

# IP Address of the direct overhead warehouse camera.
# Default: "192.168.0.100"
DIRECT_WAREHOUSE_VIDEO_HOST = os.getenv("DIRECT_WAREHOUSE_VIDEO_HOST", "192.168.0.100")


# Log the final server connection configuration being used.
logger.info(
    f"Server Connection Config: HOST={SERVER_HOST}, "
    f"COMMAND_PORT={COMMAND_PORT}, VIDEO_PORT={VIDEO_PORT}, "
    f"OVERHEAD_VIDEO_PORT={OVERHEAD_VIDEO_PORT}, "
    f"DIRECT_WAREHOUSE_VIDEO_HOST={DIRECT_WAREHOUSE_VIDEO_HOST}, "
    f"DIRECT_WAREHOUSE_VIDEO_PORT={DIRECT_WAREHOUSE_VIDEO_PORT}"
)

# --- Video Display Configuration ---
# These settings affect how video feeds are displayed in the client UI.

# Target width for displaying video frames in the UI.
# Frames received from the server might be scaled to this size.
VIDEO_WIDTH = 1280  # pixels

# Target height for displaying video frames in the UI.
# Frames received from the server might be scaled to this size.
VIDEO_HEIGHT = 720  # pixels

# Target frames per second for video display.
# This might not match the actual FPS received from the server.
VIDEO_FPS = 30  # frames per second

# --- Control Input Configuration ---
# Parameters related to manual control of the forklift.

# Maximum speed value that can be sent to the server for motor control.
# This is the upper bound of the actual speed sent for manual driving.
# The UI slider (0-100) is mapped to the range [MIN_DRIVE_SPEED, MAX_DRIVE_SPEED].
MAX_DRIVE_SPEED = 100  # Max actual speed sent to server (0-100 scale from server's perspective)

# Minimum speed value that can be sent to the server for motor control.
# This is the lower bound of the actual speed sent for manual driving.
MIN_DRIVE_SPEED = 75   # Min actual speed sent to server (0-100 scale from server's perspective)

# Maximum steering value that can be sent to the server.
# Typically corresponds to the maximum turning intensity.
MAX_STEERING = 100  # Arbitrary units, depends on server implementation

# --- User Interface (UI) Configuration ---
# Settings for the main application window.

# Width of the main application window.
WINDOW_WIDTH = 1280 # pixels, increased to better fit multiple video feeds if necessary

# Height of the main application window.
WINDOW_HEIGHT = 720  # pixels, increased to better fit multiple video feeds if necessary

# Title displayed in the application window's title bar.
WINDOW_TITLE = "Forklift Control Client"

# --- Servo Control Configuration ---
# Defines the mapping of servo identifiers (used by the server) to UI elements or keys.
# This allows the client to send commands for specific servos.
# Pins match the GPIO pins used on the server (Raspberry Pi).
# These are primarily for client-side UI mapping; actual pin numbers are canonical on the server.
FORK_SERVO_A_PIN = 13
FORK_SERVO_B_PIN = 25
FORK_SERVO_C_PIN = 26
FORK_SERVO_D_PIN = 27
FORK_SERVO_E_PIN = 18
FORK_SERVO_F_PIN = 16

# Ordered list of servo pins for UI display or sequential access if needed.
# This matches the visual grid layout in main_window.py:
# E(5) F(6)
# C(3) D(4)
# A(1) B(2)
UI_SERVO_ORDER_MAPPING = [
    FORK_SERVO_A_PIN,  # Corresponds to UI key '1'
    FORK_SERVO_B_PIN,  # Corresponds to UI key '2'
    FORK_SERVO_C_PIN,  # Corresponds to UI key '3'
    FORK_SERVO_D_PIN,  # Corresponds to UI key '4'
    FORK_SERVO_E_PIN,  # Corresponds to UI key '5'
    FORK_SERVO_F_PIN   # Corresponds to UI key '6'
]

# Mapping of number keys ('1' through '6') to servo BCM pin numbers.
# Used by MainWindow to select a servo based on keyboard shortcuts.
SERVO_KEY_TO_PIN_MAPPING = {
    '1': FORK_SERVO_A_PIN,
    '2': FORK_SERVO_B_PIN,
    '3': FORK_SERVO_C_PIN,
    '4': FORK_SERVO_D_PIN,
    '5': FORK_SERVO_E_PIN,
    '6': FORK_SERVO_F_PIN
}

# Timeout in seconds for establishing a connection to the direct warehouse feed.
DIRECT_WAREHOUSE_CONNECTION_TIMEOUT = 5.0 # seconds

# Timeout in seconds for receiving a frame from the direct warehouse feed.
# If no frame is received within this period, the connection might be considered stale.
DIRECT_WAREHOUSE_FRAME_TIMEOUT = 2.0 # seconds. Socket recv calls will use this.

logger.info("Client configuration loaded.") 