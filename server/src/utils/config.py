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
SERVO_PWM_PIN = 25

# --- Forklift Servo Positions (Placeholders - needs calibration) ---
FORK_DOWN_POSITION = 0  # Example angle
FORK_UP_POSITION = 90   # Example angle

# --- Control Configuration ---
MAX_SPEED = 100      # From old config.py
MAX_STEERING = 100   # From old config.py (Note: steering logic might not use this directly yet)

# --- Logging Configuration ---
LOG_LEVEL = "INFO"
LOG_FILE = "server.log" 