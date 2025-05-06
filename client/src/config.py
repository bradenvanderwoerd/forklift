import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Server configuration
SERVER_HOST = os.getenv("SERVER_HOST", "localhost")
COMMAND_PORT = int(os.getenv("COMMAND_PORT", "8765"))
VIDEO_PORT = int(os.getenv("VIDEO_PORT", "8766"))

# Video configuration
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
VIDEO_FPS = 30

# Control configuration
MAX_SPEED = 100
MAX_STEERING = 100

# UI configuration
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
WINDOW_TITLE = "Forklift Control" 