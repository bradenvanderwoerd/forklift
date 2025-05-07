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
    'left_forward': 17,
    'left_backward': 18,
    'right_forward': 22,
    'right_backward': 23
}

SERVO_PIN = 12  # GPIO pin for servo control

# Control configuration
MAX_SPEED = 100
MAX_STEERING = 100 