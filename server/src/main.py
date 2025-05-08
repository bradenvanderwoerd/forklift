import signal
import sys
import time
import threading
import socket
from typing import Dict, Any
import logging
import asyncio
import RPi.GPIO as GPIO

from controllers.motor import MotorController
from controllers.servo import ServoController
from network.video_stream import VideoStreamer
from network.tcp_server import CommandServer
from config import HOST, COMMAND_PORT, VIDEO_PORT, SERVO_PIN

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def check_port_availability(port: int) -> bool:
    """Check if a port is available for use."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind((HOST, port))
            return True
        except socket.error:
            return False

class ForkliftServer:
    def __init__(self):
        """Initialize server components"""
        self.running = True
        self.cleanup_lock = threading.Lock()
        self.cleanup_complete = threading.Event()
        
        # Check port availability
        if not check_port_availability(COMMAND_PORT):
            logger.error(f"Command port {COMMAND_PORT} is already in use!")
            sys.exit(1)
        if not check_port_availability(VIDEO_PORT):
            logger.error(f"Video port {VIDEO_PORT} is already in use!")
            sys.exit(1)
        
        # Initialize hardware controllers
        self.motor_controller = MotorController()
        self.servo_controller = ServoController()
        self.servo_controller.set_position(0)  # Set servo to 0 on startup
        
        # Initialize network components
        self.video_streamer = VideoStreamer()
        self.command_server = CommandServer()
        
        # Register command handlers
        self._register_handlers()
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._handle_shutdown)
        signal.signal(signal.SIGTERM, self._handle_shutdown)
        
        # Create threads for servers
        self.video_thread = threading.Thread(target=self._run_video_server)
        self.command_thread = threading.Thread(target=self._run_command_server)
    
    def _register_handlers(self):
        """Register command handlers"""
        self.command_server.register_handler('drive', self._handle_drive_command)
        self.command_server.register_handler('motor', self._handle_motor_command)
        self.command_server.register_handler('servo', self._handle_servo_command)
        self.command_server.register_handler('stop', self._handle_emergency_stop)
    
    def _handle_drive_command(self, data: Dict[str, Any]):
        logger.info("--- _handle_drive_command ENTERED ---")
        logger.info(f"Received DRIVE command data: {data}")

        command_value = data.get("value", {}) 

        direction = command_value.get("direction", "NONE")
        speed = command_value.get("speed", 0)
        action = command_value.get("action", "START") 

        logger.info(f"Parsed action: {action}, direction: {direction}, speed: {speed}")

        if action == "STOP" or direction == "NONE":
            logger.info("Executing STOP action or direction is NONE")
            self.motor_controller.stop()
            return

        if direction == "FORWARD":
            logger.info("Executing FORWARD action")
            self.motor_controller.drive_forward(speed)
        elif direction == "BACKWARD":
            logger.info("Executing BACKWARD action")
            self.motor_controller.drive_backward(speed)
        elif direction == "LEFT":
            logger.info("Executing LEFT action")
            self.motor_controller.turn_left(speed)
        elif direction == "RIGHT":
            logger.info("Executing RIGHT action")
            self.motor_controller.turn_right(speed)
        else:
            logger.info("Executing ELSE (STOP) action")
            self.motor_controller.stop()
    
    def _handle_motor_command(self, data: Dict[str, Any]):
        logger.info(f"Received MOTOR command: {data}")
        speed = data.get('speed', 0)
        self.motor_controller.set_speed(speed)
    
    def _handle_servo_command(self, data: Dict[str, Any]):
        logger.info(f"Received SERVO command: {data}")
        
        command_value = data.get("value", {}) # Get the nested 'value' dictionary
        position = command_value.get('position')
        action = command_value.get("action", "SET") # Default to SET if not specified

        logger.info(f"Parsed servo action: {action}, position: {position}")

        if action == "SET" and position is not None:
            try:
                angle = float(position) # Ensure position is a number
                self.servo_controller.set_position(angle)
                logger.info(f"Servo position set to: {angle}")
            except ValueError:
                logger.error(f"Invalid servo position received: {position}. Must be a number.")
        elif command_value.get('step_up'): # Keep step_up/step_down if client might send them
            logger.info("Servo stepping up")
            self.servo_controller.step_up()
        elif command_value.get('step_down'):
            logger.info("Servo stepping down")
            self.servo_controller.step_down()
        else:
            logger.warning(f"Unknown servo action or missing position in command: {command_value}")
    
    def _handle_emergency_stop(self, data: Dict[str, Any]):
        """Handle emergency stop command"""
        logger.info(f"--- _handle_emergency_stop ENTERED --- Data: {data}")
        self.motor_controller.stop()
        self.servo_controller.set_position(0) # Also set servo to 0 on emergency stop
    
    def _run_video_server(self):
        """Run video server in a separate thread"""
        try:
            self.video_streamer.start()
        except Exception as e:
            logger.error(f"Error in video server thread: {e}")
            self.running = False
    
    def _run_command_server(self):
        """Run command server in a separate thread"""
        try:
            self.command_server.start()
        except Exception as e:
            logger.error(f"Error in command server thread: {e}")
            self.running = False
    
    def _handle_shutdown(self, signum, frame):
        """Handle shutdown signal
        
        Args:
            signum: Signal number
            frame: Current stack frame
        """
        logger.info(f"\nShutdown signal {signum} received...")
        self.running = False
        self.cleanup()
        # Wait for cleanup to complete
        if not self.cleanup_complete.wait(timeout=5.0):
            logger.warning("Cleanup did not complete in time")
        sys.exit(0)
    
    def start(self):
        """Start server components"""
        logger.info("Starting server...")
        logger.info(f"Command server will listen on {HOST}:{COMMAND_PORT}")
        logger.info(f"Video server will listen on {HOST}:{VIDEO_PORT}")
        
        try:
            # Start servers in separate threads
            self.video_thread.start()
            self.command_thread.start()
            
            # Main loop
            while self.running:
                time.sleep(0.1)  # Reduced sleep time for more responsive shutdown
                
        except KeyboardInterrupt:
            logger.info("\nKeyboard interrupt received...")
            self.running = False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        if not self.cleanup_lock.acquire(blocking=False):
            logger.warning("Cleanup already in progress")
            return
            
        try:
            logger.info("Cleaning up...")
            
            # Stop components
            try:
                self.video_streamer.stop()
                logger.info("Video streamer stopped")
            except Exception as e:
                logger.error(f"Error stopping video streamer: {e}")
                
            try:
                self.command_server.stop()
                logger.info("Command server stopped")
            except Exception as e:
                logger.error(f"Error stopping command server: {e}")
            
            # Clean up resources
            try:
                self.video_streamer.cleanup()
                logger.info("Video streamer cleaned up")
            except Exception as e:
                logger.error(f"Error cleaning up video streamer: {e}")
                
            try:
                self.command_server.cleanup()
                logger.info("Command server cleaned up")
            except Exception as e:
                logger.error(f"Error cleaning up command server: {e}")
                
            try:
                self.motor_controller.cleanup()
                logger.info("Motor controller cleaned up")
            except Exception as e:
                logger.error(f"Error cleaning up motor controller: {e}")
                
            try:
                self.servo_controller.set_position(0) # Set servo to 0 on shutdown
                self.servo_controller.cleanup()
                logger.info("Servo controller cleaned up")
            except Exception as e:
                logger.error(f"Error cleaning up servo controller: {e}")
            
            logger.info("Cleanup complete")
            
        finally:
            self.cleanup_complete.set()
            self.cleanup_lock.release()

if __name__ == '__main__':
    server = ForkliftServer()
    server.start() 