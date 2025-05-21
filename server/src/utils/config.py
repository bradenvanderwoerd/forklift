import cv2
import os
from dotenv import load_dotenv

# Load environment variables from .env file (if it exists)
load_dotenv()

# --- Server Network Configuration ---
HOST = os.getenv("HOST", "0.0.0.0") # From old config.py
SERVER_TCP_PORT = int(os.getenv("COMMAND_PORT", "4001")) # Keep 4001 as per doc, allow override
SERVER_VIDEO_UDP_PORT = int(os.getenv("VIDEO_PORT", "5001")) # Keep 5001 as per doc, allow override

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
SERVO_PWM_PIN = 13

# --- Forklift Servo Positions ---
FORK_DOWN_POSITION = 85      # Default down position (startup, shutdown, manual down, auto-nav pickup)
FORK_UP_POSITION = 0        # Default up position (manual up)
AUTONAV_FORK_LOWER_TO_PICKUP_ANGLE = 85 # Special low position for auto-nav to get under a box (same as FORK_DOWN_POSITION)
AUTONAV_FORK_CARRY_ANGLE = 60 # Position for carrying a box during auto-navigation
# SERVO_ABSOLUTE_MAX_ANGLE = 80 # Physical maximum for the servo controller

# --- Control Configuration ---
MAX_SPEED = 100      # From old config.py
MAX_STEERING = 100   # From old config.py (Note: steering logic might not use this directly yet)

# --- Logging Configuration ---
LOG_LEVEL = "INFO"
LOG_FILE = "server.log"

# --- Navigation Controller Configuration ---
NAV_TARGET_APPROACH_DISTANCE_M = 0.15  # Target distance to stop from marker (in meters)
NAV_DISTANCE_THRESHOLD_M = 0.02       # Allowed error in distance (meters)
NAV_X_THRESHOLD_M = 0.02              # Allowed error in x-offset (meters) - Will be replaced by angular for centering
NAV_X_THRESHOLD_RAD = 0.05            # Allowed angular error for centering (radians, e.g., 0.05 rad ~= 2.8 deg)

# PID Gains for Turning (correcting X-offset from tvec)
NAV_TURNING_PID_KP = 2.0  # Proportional gain for turning PID
NAV_TURNING_PID_KI = 0.05  # Integral gain for turning PID
NAV_TURNING_PID_KD = 0.5   # Derivative gain for turning PID

# PID Gains for Distance (correcting Z-offset from tvec)
NAV_DISTANCE_PID_KP = 1.5  # Proportional gain for distance PID
NAV_DISTANCE_PID_KI = 0.03 # Integral gain for distance PID
NAV_DISTANCE_PID_KD = 0.3  # Derivative gain for distance PID

NAV_MAX_TURNING_SPEED = 10       # Maximum speed for turning (PWM duty cycle)
MANUAL_TURN_SPEED = 1            # Fixed speed for MANUAL turning actions (0-100 for motor controller)
NAV_MAX_FORWARD_SPEED = 35       # Maximum speed for forward/backward movement (PWM duty cycle)
NAV_MIN_EFFECTIVE_SPEED = 1      # Minimum speed to ensure motors engage if PID output is too low but error exists - SET LOW FOR TUNING

# --- Servo Control Configuration ---
SERVO_STEP_DEGREES = 1          # Degrees to move the servo per step for smooth movement
SERVO_STEP_DELAY_SECONDS = 0.015 # Delay between each step (controls speed)

# --- Logging Configuration ---
LOG_LEVEL = "INFO"
LOG_FILE = "server.log"

# --- New Overhead Camera Configuration ---
OVERHEAD_CAMERA_HOST = "192.168.0.100" # TODO: Update with actual Arena Server IP
OVERHEAD_CAMERA_PORT = 5001
ROBOT_OVERHEAD_ARUCO_ID = 50 # ArUco ID of the marker on top of the robot
# ARUCO_DICTIONARY_OVERHEAD = cv2.aruco.DICT_6X6_50 # Defined globally if same as ARUCO_DICTIONARY
# If you use a different dictionary for overhead, define it here and import cv2 if not already.

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