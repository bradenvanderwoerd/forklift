import cv2
import os
from dotenv import load_dotenv

# Load environment variables from .env file (if it exists)
load_dotenv()

# --- Server Network Configuration ---
HOST = os.getenv("HOST", "0.0.0.0") # From old config.py
SERVER_TCP_PORT = int(os.getenv("COMMAND_PORT", "3456"))
SERVER_VIDEO_UDP_PORT = int(os.getenv("VIDEO_PORT", "3457"))

# --- Video Stream Configuration ---
VIDEO_WIDTH = 640  # From old config.py
VIDEO_HEIGHT = 480 # From old config.py
VIDEO_FPS = 30     # From old config.py

# --- ArUco Marker Configuration ---
ARUCO_DICTIONARY = cv2.aruco.DICT_6X6_50
ARUCO_MARKER_SIZE_METERS = 0.0226  # 22.6mm

# Target Marker IDs
MARKER_ID_WHITE_BOX = 5
MARKER_ID_BLACK_BOX = 6
MARKER_ID_WHITE_BOX_DESTINATION = 1
MARKER_ID_BLACK_BOX_DESTINATION = 2
MARKER_ID_NAVIGATION_AID = -1 # Placeholder for unknown arena marker

# --- Camera Calibration ---
CAMERA_CALIBRATION_FILE_PATH = "server/src/camera_calibration.npz"

# --- Hardware GPIO Pin Configuration (BCM numbering) ---
# Motor Control - Left Motor
MOTOR_LEFT_FORWARD_PIN = 5
MOTOR_LEFT_BACKWARD_PIN = 6
MOTOR_LEFT_PWM_PIN = 17

# Motor Control - Right Motor
MOTOR_RIGHT_FORWARD_PIN = 22
MOTOR_RIGHT_BACKWARD_PIN = 23
MOTOR_RIGHT_PWM_PIN = 24

# Servo Control
FORK_SERVO_A_PIN = 13 # Pin for Fork A (formerly SERVO_PIN_FORK, primary for autonav/default)
FORK_SERVO_B_PIN = 25 # Pin for Fork B (formerly SERVO_PIN_A)
FORK_SERVO_C_PIN = 26 # Pin for Fork C (formerly SERVO_PIN_B)
FORK_SERVO_D_PIN = 27 # Pin for Fork D (formerly SERVO_PIN_C)
FORK_SERVO_E_PIN = 18 # Pin for Fork E (formerly SERVO_PIN_D)
FORK_SERVO_F_PIN = 16 # Pin for Fork F (formerly SERVO_PIN_E)

# --- Forklift Servo Positions (General) ---
FORK_DOWN_POSITION = 85      # Default down position for forks
FORK_UP_POSITION = 0        # Default up position for forks
AUTONAV_FORK_LOWER_TO_PICKUP_ANGLE = 85 
AUTONAV_FORK_CARRY_ANGLE = 60 

# --- Individual Servo Angle Configurations ---
# For FORK_SERVO_A (Pin 13) - Primary Fork
FORK_A_INITIAL_ANGLE = 0
FORK_A_DOWN_ANGLE = 0
FORK_A_UP_ANGLE = 6

# For FORK_SERVO_B (Pin 25)
FORK_B_INITIAL_ANGLE = 83
FORK_B_DOWN_ANGLE = 83
FORK_B_UP_ANGLE = 77

# For FORK_SERVO_C (Pin 26)
FORK_C_INITIAL_ANGLE = 6
FORK_C_DOWN_ANGLE = 6
FORK_C_UP_ANGLE = 12

# For FORK_SERVO_D (Pin 27)
FORK_D_INITIAL_ANGLE = 85
FORK_D_DOWN_ANGLE = 85
FORK_D_UP_ANGLE = 79

# For FORK_SERVO_E (Pin 18)
FORK_E_INITIAL_ANGLE = 0
FORK_E_DOWN_ANGLE = 0
FORK_E_UP_ANGLE = 6

# For FORK_SERVO_F (Pin 16)
FORK_F_INITIAL_ANGLE = 85
FORK_F_DOWN_ANGLE = 85
FORK_F_UP_ANGLE = 79

# --- Control Configuration ---
MAX_SPEED = 100      # From old config.py
MAX_STEERING = 100   # From old config.py (Note: steering logic might not use this directly yet)

# --- Logging Configuration ---
LOG_LEVEL = "INFO"
LOG_FILE = "server.log"
LOG_MAX_BYTES = 1024 * 1024 * 5  # 5 MB max log file size
LOG_BACKUP_COUNT = 3  # Number of backup log files to keep

# --- Navigation Controller Configuration (Overhead Pixel Space) ---
OVERHEAD_NAV_TARGET_APPROACH_DISTANCE_PIXELS = 10.0 # Target distance (pixels) to stop from the target X,Y
OVERHEAD_NAV_DISTANCE_THRESHOLD_PIXELS = 5.0    # Allowed error in pixel distance to consider target X,Y reached
OVERHEAD_NAV_ORIENTATION_THRESHOLD_RAD = 0.1    # Allowed angular error for aligning to target *point* (radians, ~5.7 deg)
OVERHEAD_NAV_FINAL_ANGULAR_THRESHOLD_RAD = 0.05 # Allowed angular error for final *target orientation* (radians, ~2.8 deg)

# PID Gains for Turning to face the target point
OVERHEAD_NAV_TURNING_PID_KP = 0.001
OVERHEAD_NAV_TURNING_PID_KI = 0
OVERHEAD_NAV_TURNING_PID_KD = 0.05

# PID Gains for Distance to the target point (pixels)
OVERHEAD_NAV_DISTANCE_PID_KP = 0.1  # Proportional gain for pixel distance PID
OVERHEAD_NAV_DISTANCE_PID_KI = 0 # Integral gain for pixel distance PID
OVERHEAD_NAV_DISTANCE_PID_KD = 0.2  # Derivative gain for pixel distance PID

# PID Gains for achieving the Final Target Orientation (after X,Y is reached)
OVERHEAD_NAV_FINAL_ORIENTATION_PID_KP = 0.7
OVERHEAD_NAV_FINAL_ORIENTATION_PID_KI = 0.05
OVERHEAD_NAV_FINAL_ORIENTATION_PID_KD = 0.15

# Speed limits (retained from previous, ensure they are 0-100 for PWM)
MANUAL_TURN_SPEED = 75  # Default turn speed for manual control (can be overridden by client)
NAV_MAX_TURNING_SPEED = 100     # Max speed for turning actions during navigation
NAV_MAX_FORWARD_SPEED = 100     # Max speed for forward movement during navigation
NAV_MIN_EFFECTIVE_SPEED = 80  # The minimum speed motor will run at during nav (scaled from PID)

# --- Old Metric Navigation Constants (Commented out) ---
# NAV_TARGET_APPROACH_DISTANCE_M = 0.15
# NAV_DISTANCE_THRESHOLD_M = 0.02
# NAV_X_THRESHOLD_M = 0.02
# NAV_X_THRESHOLD_RAD = 0.05
# NAV_TURNING_PID_KP = 2.0
# NAV_TURNING_PID_KI = 0.05
# NAV_TURNING_PID_KD = 0.5
# NAV_DISTANCE_PID_KP = 1.5
# NAV_DISTANCE_PID_KI = 0.03
# NAV_DISTANCE_PID_KD = 0.3

# --- Servo Control Configuration ---
SERVO_STEP_DEGREES = 1          # Degrees to move the servo per step for smooth movement
SERVO_STEP_DELAY_SECONDS = 0.015 # Delay between each step (controls speed)

# --- New Overhead Camera Configuration ---
OVERHEAD_CAMERA_HOST = "192.168.0.100" # TODO: Update with actual Arena Server IP
OVERHEAD_CAMERA_PORT = 5001
ROBOT_OVERHEAD_ARUCO_ID = 36 # ArUco ID of the marker on top of the robot
OVERHEAD_VIDEO_WEBSOCKET_PORT = 3458 # New port for streaming overhead video via WebSocket
# ARUCO_DICTIONARY_OVERHEAD = cv2.aruco.DICT_6X6_50 # Defined globally if same as ARUCO_DICTIONARY
# If you use a different dictionary for overhead, define it here and import cv2 if not already.

# --- Overhead Robot Pose Offset Configuration ---
# Offsets from the ArUco marker's center to the robot's true center of rotation (e.g., midpoint of axle).
# These are in pixels, from the marker's perspective when the robot is facing its local 0 radians (forward).
# - X_OFFSET: Positive is "ahead" of the marker, negative is "behind".
# - Y_OFFSET: Positive is to the marker's "left", negative is to the marker's "right".
MARKER_TO_ROTATION_CENTER_X_OFFSET_PIXELS = 10
MARKER_TO_ROTATION_CENTER_Y_OFFSET_PIXELS = 10 # 5 pixels to the right

# Example of static locations in pixel space (to be determined via calibration/manual clicking)
# These are placeholders and need to be filled after implementing Sub-Phase 5.2
OVERHEAD_VIEW_STATIC_LOCATIONS = {
    "POLE_CENTER_PIXEL": (0, 0), # (x, y)
    "POLE_RADIUS_PIXEL": 0,
    "BIN_WHITE_START_PIXEL": (0, 0), # (x, y)
    "BIN_BLACK_START_PIXEL": (0, 0), # (x, y)
    "TRUCK_DESTINATION_PIXEL": (0, 0) # (x, y)
}
# --- End of New Overhead Camera Configuration ---

# Make sure this is the last line if utility functions or further setup is needed below. 