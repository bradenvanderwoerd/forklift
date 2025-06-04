import cv2
import os
from dotenv import load_dotenv

# Load environment variables from .env file (if it exists)
# This allows sensitive or environment-specific settings (like HOST IP) to be
# configured without modifying the codebase directly.
load_dotenv()

# --- Server Network Configuration ---
# These settings define where the Forklift server will listen for connections.
HOST = os.getenv("HOST", "0.0.0.0") # IP address for the server to bind to.
                                     # "0.0.0.0" means listen on all available network interfaces.
                                     # Can be overridden by a HOST variable in a .env file.
SERVER_TCP_PORT = int(os.getenv("COMMAND_PORT", "3456")) # Port for the main command server (WebSocket).
                                                        # Client sends drive/servo commands here.
SERVER_VIDEO_UDP_PORT = int(os.getenv("VIDEO_PORT", "3457")) # Port for the onboard camera video stream (WebSocket).
                                                           # PiCamera video is streamed from here.

# --- Onboard Video Stream Configuration (PiCamera) ---
# Settings for the video stream coming directly from the Raspberry Pi's camera.
VIDEO_WIDTH = 1280  # Desired width of the video stream in pixels.
VIDEO_HEIGHT = 720 # Desired height of the video stream in pixels.
VIDEO_FPS = 30     # Desired frames per second for the video stream.
                   # Actual FPS may be lower depending on Pi's load and camera capabilities.

# --- ArUco Marker Configuration (Onboard Camera) ---
# Settings related to ArUco markers detected by the onboard PiCamera.
ARUCO_DICTIONARY = cv2.aruco.DICT_6X6_50 # The specific ArUco dictionary to use for marker detection.
                                        # DICT_6X6_50 contains markers with 6x6 bits and 50 unique IDs.
ARUCO_MARKER_SIZE_METERS = 0.0226  # Actual physical size of the ArUco markers in meters (22.6mm).
                                   # Crucial for accurate distance and pose estimation.

# Target Marker IDs (for onboard camera object detection, if used for this purpose)
# These were likely for a previous iteration of object detection directly with the onboard camera.
# Current navigation primarily uses the overhead camera.
MARKER_ID_WHITE_BOX = 5
MARKER_ID_BLACK_BOX = 6
MARKER_ID_WHITE_BOX_DESTINATION = 1
MARKER_ID_BLACK_BOX_DESTINATION = 2
MARKER_ID_NAVIGATION_AID = -1 # Placeholder for any generic arena marker if needed.

# --- Camera Calibration (Onboard Camera) ---
# Path to the camera calibration file for the onboard PiCamera.
# This file (.npz) contains the camera matrix and distortion coefficients,
# necessary for correcting lens distortion and for accurate 3D pose estimation.
CAMERA_CALIBRATION_FILE_PATH = "server/src/camera_calibration.npz"

# --- Hardware GPIO Pin Configuration (BCM numbering) ---
# Defines which Raspberry Pi GPIO pins are connected to motor and servo controllers.
# BCM numbering refers to the Broadcom SOC channel numbers.

# Motor Control - Left Motor
MOTOR_LEFT_FORWARD_PIN = 5    # GPIO pin to drive the left motor forward.
MOTOR_LEFT_BACKWARD_PIN = 6   # GPIO pin to drive the left motor backward.
MOTOR_LEFT_PWM_PIN = 17       # GPIO pin for PWM signal to control the left motor's speed.

# Motor Control - Right Motor
MOTOR_RIGHT_FORWARD_PIN = 22  # GPIO pin to drive the right motor forward.
MOTOR_RIGHT_BACKWARD_PIN = 23 # GPIO pin to drive the right motor backward.
MOTOR_RIGHT_PWM_PIN = 24      # GPIO pin for PWM signal to control the right motor's speed.

# Servo Control Pins
# Defines the GPIO pins for each of the six servos.
FORK_SERVO_A_PIN = 13 # Pin for Fork A (Primary fork, used for autonavigation and default control)
FORK_SERVO_B_PIN = 25 # Pin for Fork B
FORK_SERVO_C_PIN = 26 # Pin for Fork C
FORK_SERVO_D_PIN = 27 # Pin for Fork D
FORK_SERVO_E_PIN = 18 # Pin for Fork E
FORK_SERVO_F_PIN = 16 # Pin for Fork F

# --- Forklift Servo Positions (General & Autonavigation Defaults) ---
# These define default positions used by some logic, especially for the primary fork in autonav.
# Individual servos also have their own specific up/down/initial angles defined below.
FORK_DOWN_POSITION = 85      # A general 'down' position in degrees, historically used.
FORK_UP_POSITION = 0        # A general 'up' position in degrees, historically used.

# Angles for the primary fork (Fork A) during autonomous navigation sequence.
AUTONAV_FORK_LOWER_TO_PICKUP_ANGLE = 85 # Angle (degrees) to lower Fork A to when picking up.
AUTONAV_FORK_CARRY_ANGLE = 60          # Angle (degrees) to raise Fork A to when carrying.

# --- Individual Servo Angle Configurations (Degrees) ---
# Each servo has its own defined initial, down, and up positions.
# This allows for fine-tuning the movement range of each servo independently.
# "Initial" is the position the servo moves to upon initialization.
# "Down" is its defined lower limit for step_down commands.
# "Up" is its defined upper limit for step_up commands.

# For FORK_SERVO_A (Pin 13) - Primary Fork for Autonav
FORK_A_INITIAL_ANGLE = 6
FORK_A_DOWN_ANGLE = 6   # Its 'down' position
FORK_A_UP_ANGLE = 12    # Its 'up' position

# For FORK_SERVO_B (Pin 25)
FORK_B_INITIAL_ANGLE = 85
FORK_B_DOWN_ANGLE = 85
FORK_B_UP_ANGLE = 79

# For FORK_SERVO_C (Pin 26)
FORK_C_INITIAL_ANGLE = 4
FORK_C_DOWN_ANGLE = 4
FORK_C_UP_ANGLE = 10

# For FORK_SERVO_D (Pin 27)
FORK_D_INITIAL_ANGLE = 85
FORK_D_DOWN_ANGLE = 85
FORK_D_UP_ANGLE = 77

# For FORK_SERVO_E (Pin 18)
FORK_E_INITIAL_ANGLE = 0
FORK_E_DOWN_ANGLE = 0
FORK_E_UP_ANGLE = 8 # Corrected from 0 to 8 based on previous discussions if this was a typo. Assuming 8.

# For FORK_SERVO_F (Pin 16)
FORK_F_INITIAL_ANGLE = 72
FORK_F_DOWN_ANGLE = 72
FORK_F_UP_ANGLE = 64 # Corrected from 72 to 64 based on previous discussions if this was a typo. Assuming 64.


# --- General Control Configuration ---
# These are general limits, primarily for manual control or older systems.
MAX_SPEED = 100      # Maximum speed value (0-100 scale) used by client/server.
                     # Motor controller interprets this for PWM duty cycle.
MAX_STEERING = 100   # Maximum steering value. (Note: Current differential drive might not use this directly for 'steering').

# --- Logging Configuration ---
# Settings for the Python logging module.
LOG_LEVEL = "INFO"              # Default logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL).
LOG_FILE = "server.log"         # Name of the file to log messages to.
LOG_MAX_BYTES = 1024 * 1024 * 5 # Maximum size of a single log file (5 MB).
LOG_BACKUP_COUNT = 3            # Number of backup log files to keep (e.g., server.log.1, server.log.2).

# --- Navigation Controller Configuration (Overhead Pixel Space) ---
# Parameters for the autonomous navigation logic, operating in the overhead camera's pixel coordinate system.

# Thresholds for determining navigation state completion.
OVERHEAD_NAV_TARGET_APPROACH_DISTANCE_PIXELS = 10.0 # When driving to an XY point, this is the 'close enough' distance to stop before final alignment.
OVERHEAD_NAV_DISTANCE_THRESHOLD_PIXELS = 25.0    # Allowed error in pixel distance from the target XY to consider it "reached".
OVERHEAD_NAV_ORIENTATION_THRESHOLD_RAD = 0.1    # Allowed angular error (radians, ~5.7 deg) when aligning the robot's heading towards the target XY point.
OVERHEAD_NAV_FINAL_ANGULAR_THRESHOLD_RAD = 0.05 # Allowed angular error (radians, ~2.8 deg) when aligning to the final target orientation (theta) after XY is reached.

# PID Gains for Turning to face the target XY point (State 1 of navigation)
# Kp: Proportional gain. Higher values mean stronger reaction to current error.
# Ki: Integral gain. Accumulates past errors; helps eliminate steady-state error but can cause overshoot.
# Kd: Derivative gain. Predicts future error based on rate of change; helps dampen oscillations.
OVERHEAD_NAV_TURNING_PID_KP = 5.0
OVERHEAD_NAV_TURNING_PID_KI = 0.0 # Typically start with Ki=0 and tune Kp, Kd first.
OVERHEAD_NAV_TURNING_PID_KD = 0.05

# PID Gains for controlling the robot's distance to the target XY point (State 2 of navigation)
OVERHEAD_NAV_DISTANCE_PID_KP = 0.1
OVERHEAD_NAV_DISTANCE_PID_KI = 0.0
OVERHEAD_NAV_DISTANCE_PID_KD = 0.2

# PID Gains for achieving the Final Target Orientation (State 3 of navigation, after X,Y is reached)
OVERHEAD_NAV_FINAL_ORIENTATION_PID_KP = 2.0
OVERHEAD_NAV_FINAL_ORIENTATION_PID_KI = 0.05
OVERHEAD_NAV_FINAL_ORIENTATION_PID_KD = 0.15

# Speed limits for navigation actions (PWM duty cycle values, 0-100)
MANUAL_TURN_SPEED = 75          # Default speed for manual turning commands (can be overridden by client speed slider).
NAV_MAX_TURNING_SPEED = 100     # Maximum speed for turning actions during autonomous navigation.
NAV_MAX_FORWARD_SPEED = 100     # Maximum speed for forward movement during autonomous navigation.
NAV_MIN_EFFECTIVE_SPEED = 80    # The minimum speed the motors will run at during navigation if PID output is non-zero.
                                # This helps overcome static friction and ensures movement even with small PID efforts.

# --- Old Metric Navigation Constants (Commented out for reference) ---
# These were from a previous navigation system that operated in meters, not pixels.
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

# --- Servo Control Configuration (General - for incremental movement if used) ---
# These parameters are for the older ServoController.move_smoothly_to method,
# which is not directly used by the current go_to_up/down_position logic that moves in 2-degree steps.
SERVO_STEP_DEGREES = 1          # Degrees to move the servo per incremental step.
SERVO_STEP_DELAY_SECONDS = 0.015 # Delay between each incremental step, controlling smooth movement speed.

# --- Overhead Warehouse Camera Configuration ---
# Settings for connecting to the external "warehouse" camera server.
OVERHEAD_CAMERA_HOST = "192.168.0.100" # IP address of the machine running the warehouse camera server.
                                       # TODO: Update with actual Arena Server IP if different.
OVERHEAD_CAMERA_PORT = 4001            # Port on which the warehouse camera server is listening for frame requests.
ROBOT_OVERHEAD_ARUCO_ID = 36           # The ArUco ID of the marker placed on top of the Forklift robot.
                                       # This is used by the OverheadLocalizer to find the robot in the warehouse view.
OVERHEAD_VIDEO_WEBSOCKET_PORT = 3458   # WebSocket port on this Pi server for streaming the processed overhead video
                                       # (raw feed from warehouse cam + robot pose overlay) to the client.
# ARUCO_DICTIONARY_OVERHEAD = cv2.aruco.DICT_6X6_50 # Defined globally if same as ARUCO_DICTIONARY.
# If you use a different ArUco dictionary for the overhead view, define it here.

# --- Overhead Robot Pose Offset Configuration ---
# These offsets are used by the OverheadLocalizer to calculate the robot's true center of rotation
# from the center of its detected ArUco marker in the overhead camera view.
# Values are in pixels, from the marker's perspective when the robot is facing its local 0 radians (forward along its X-axis).
# - X_OFFSET: Positive values mean the rotation center is "ahead" of (in front of) the marker's center.
#             Negative values mean "behind".
# - Y_OFFSET: Positive values mean the rotation center is to the marker's "left".
#             Negative values mean to the marker's "right".
# Example: If marker is at the back, X_OFFSET might be positive. If marker is off-center to one side, Y_OFFSET is used.
MARKER_TO_ROTATION_CENTER_X_OFFSET_PIXELS = 0  # Robot's rotation center X relative to marker center.
MARKER_TO_ROTATION_CENTER_Y_OFFSET_PIXELS = -5 # Robot's rotation center Y relative to marker center (e.g., -5 means 5px to marker's right).

# Example of static locations in pixel space (for potential future use or manual reference)
# These are placeholders and would need to be determined via calibration or by clicking points in the overhead view.
# Currently not actively used by the autonomous navigation logic, which takes dynamic targets.
OVERHEAD_VIEW_STATIC_LOCATIONS = {
    "POLE_CENTER_PIXEL": (0, 0),        # (x, y) coordinates of a pole or obstacle.
    "POLE_RADIUS_PIXEL": 0,             # Radius of that pole/obstacle.
    "BIN_WHITE_START_PIXEL": (0, 0),    # (x, y) coordinates for a white bin.
    "BIN_BLACK_START_PIXEL": (0, 0),    # (x, y) coordinates for a black bin.
    "TRUCK_DESTINATION_PIXEL": (0, 0)   # (x, y) coordinates for a truck destination.
}
# --- End of New Overhead Camera Configuration ---

# Make sure this is the last line if utility functions or further setup is needed below.
# No further executable code should follow this line unless it's part of configuration loading. 