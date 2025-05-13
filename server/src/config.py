import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Server configuration
HOST = os.getenv("HOST", "0.0.0.0")  # Listen on all interfaces
COMMAND_PORT = int(os.getenv("COMMAND_PORT", "3456"))  # Command WebSocket port
VIDEO_PORT = int(os.getenv("VIDEO_PORT", "3457"))     # Video WebSocket port

# Video configuration
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
VIDEO_FPS = 30

# Hardware configuration
MOTOR_PINS = {
    'left_forward': 5,
    'left_backward': 6,
    'right_forward': 22,
    'right_backward': 23
}
MOTOR1_PWM = 17  # Left motor speed
MOTOR2_PWM = 24  # Right motor speed
SERVO_PIN = 13  # Servo PWM pin

# Control configuration
MAX_SPEED = 100
MAX_STEERING = 100
MAX_SERVO_ANGLE = 38  # Maximum servo angle in degrees

# Video configuration
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
VIDEO_FPS = 30

# Hardware configuration
MOTOR_PINS = {
    'left_forward': 5,
    'left_backward': 6,
    'right_forward': 22,
    'right_backward': 23
}
MOTOR1_PWM = 17  # Left motor speed
MOTOR2_PWM = 24  # Right motor speed
SERVO_PIN = 13  # Servo PWM pin

# Control configuration
MAX_SPEED = 100
MAX_STEERING = 100 