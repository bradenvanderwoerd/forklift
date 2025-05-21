import os
from dotenv import load_dotenv
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables from .env file
env_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), '.env')
logger.info(f"Loading environment from: {env_path}")
load_dotenv(env_path)

# Server configuration
SERVER_HOST = os.getenv("SERVER_HOST", "localhost")
COMMAND_PORT = int(os.getenv("COMMAND_PORT", "3456"))  # Command WebSocket port
VIDEO_PORT = int(os.getenv("VIDEO_PORT", "3457"))     # Onboard Video WebSocket port
OVERHEAD_VIDEO_PORT = int(os.getenv("OVERHEAD_VIDEO_PORT", "3458")) # Overhead Video WebSocket port

# Log the configuration
logger.info(f"Server configuration: HOST={SERVER_HOST}, COMMAND_PORT={COMMAND_PORT}, VIDEO_PORT={VIDEO_PORT}, OVERHEAD_VIDEO_PORT={OVERHEAD_VIDEO_PORT}")

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