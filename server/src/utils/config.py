import cv2

# ArUco Marker Configuration
ARUCO_DICTIONARY = cv2.aruco.DICT_7X7_250  # Example, replace 250 with actual if different
ARUCO_MARKER_SIZE_METERS = 0.0226  # 22.6mm

# Target Marker IDs (Placeholders - replace with actual IDs when known)
MARKER_ID_WHITE_BOX = -1
MARKER_ID_BLACK_BOX = -1
MARKER_ID_WHITE_BOX_DESTINATION = -1
MARKER_ID_BLACK_BOX_DESTINATION = -1
MARKER_ID_NAVIGATION_AID = -1

# Camera Calibration
CAMERA_CALIBRATION_FILE_PATH = "server/src/camera_calibration.npz"

# GPIO Pin Configuration (from PROJECT_DOCUMENTATION.md)
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

# Forklift Servo Positions (from ARUCO_PLAN.md - Phase 3)
# These are placeholders, actual values need calibration
FORK_DOWN_POSITION = 0  # Example angle
FORK_UP_POSITION = 90   # Example angle

# Network Configuration
SERVER_TCP_PORT = 4001
SERVER_VIDEO_UDP_PORT = 5001

# Logging Configuration
LOG_LEVEL = "INFO"
LOG_FILE = "server.log" 