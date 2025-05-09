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
NAV_TARGET_APPROACH_DISTANCE_M = 0.15  # Target distance from marker (15cm)
NAV_DISTANCE_THRESHOLD_M = 0.03      # Tolerance for being "at distance" (3cm)
NAV_X_THRESHOLD_M = 0.02             # Tolerance for being "centered" on X (2cm)

# PID Gains for Turning (correcting X-offset from tvec)
NAV_TURNING_PID_KP = 250.0 # Proportional gain (higher means stronger reaction to X-offset)
NAV_TURNING_PID_KI = 5.0   # Integral gain (accumulates error over time)
NAV_TURNING_PID_KD = 10.0  # Derivative gain (dampens overshoot)

# PID Gains for Distance (correcting Z-offset from tvec)
NAV_DISTANCE_PID_KP = 150.0 # Proportional gain (higher means faster approach)
NAV_DISTANCE_PID_KI = 2.0   # Integral gain
NAV_DISTANCE_PID_KD = 5.0  # Derivative gain

NAV_MAX_TURNING_SPEED = 5        # Max speed for AUTONOMOUS turning actions (0-100 for motor controller)
MANUAL_TURN_SPEED = 5            # Fixed speed for MANUAL turning actions (0-100 for motor controller)
NAV_MAX_FORWARD_SPEED = 50       # Max speed for forward/backward actions (0-100)
NAV_MIN_EFFECTIVE_SPEED = 5      # Minimum speed to ensure motors engage if PID output is too low but error exists

# --- Servo Control Configuration ---
SERVO_STEP_DEGREES = 1          # Degrees to move the servo per step for smooth movement
SERVO_STEP_DELAY_SECONDS = 0.015 # Delay between each step (controls speed)

# --- Logging Configuration ---
LOG_LEVEL = "INFO"
LOG_FILE = "server.log" 