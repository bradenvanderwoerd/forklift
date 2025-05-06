import signal
import sys
import time
import threading
from typing import Dict, Any

from controllers.motor import MotorController
from controllers.servo import ServoController
from network.video_stream import VideoStreamer
from network.tcp_server import CommandServer

class ForkliftServer:
    def __init__(self):
        """Initialize server components"""
        self.running = True
        self.cleanup_lock = threading.Lock()
        
        # Initialize hardware controllers
        self.motor_controller = MotorController()
        self.servo_controller = ServoController()
        
        # Initialize network components
        self.video_streamer = VideoStreamer()
        self.command_server = CommandServer()
        
        # Register command handlers
        self._register_handlers()
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._handle_shutdown)
        signal.signal(signal.SIGTERM, self._handle_shutdown)
    
    def _register_handlers(self):
        """Register command handlers"""
        self.command_server.register_handler('motor', self._handle_motor_command)
        self.command_server.register_handler('servo', self._handle_servo_command)
        self.command_server.register_handler('emergency_stop', self._handle_emergency_stop)
    
    def _handle_motor_command(self, data: Dict[str, Any]):
        """Handle motor control command
        
        Args:
            data: Command data containing speed
        """
        speed = data.get('speed', 0)
        self.motor_controller.set_speed(speed)
    
    def _handle_servo_command(self, data: Dict[str, Any]):
        """Handle servo control command
        
        Args:
            data: Command data containing position
        """
        position = data.get('position')
        if position is not None:
            self.servo_controller.set_position(position)
        elif data.get('step_up'):
            self.servo_controller.step_up()
        elif data.get('step_down'):
            self.servo_controller.step_down()
    
    def _handle_emergency_stop(self, _: Dict[str, Any]):
        """Handle emergency stop command"""
        self.motor_controller.stop()
        self.servo_controller.set_position(0)
    
    def _handle_shutdown(self, signum, frame):
        """Handle shutdown signal
        
        Args:
            signum: Signal number
            frame: Current stack frame
        """
        print("\nShutdown signal received...")
        self.running = False
        self.cleanup()
        sys.exit(0)
    
    def start(self):
        """Start server components"""
        print("Starting server...")
        
        try:
            # Start video streaming
            self.video_streamer.start()
            print("Video streaming started")
            
            # Start command server
            self.command_server.start()
            print("Command server started")
            
            # Main loop
            while self.running:
                time.sleep(0.1)  # Reduced sleep time for more responsive shutdown
                
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received...")
            self.running = False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        with self.cleanup_lock:  # Prevent multiple cleanup calls
            print("Cleaning up...")
            
            # Stop components
            try:
                self.video_streamer.stop()
                print("Video streamer stopped")
            except Exception as e:
                print(f"Error stopping video streamer: {e}")
                
            try:
                self.command_server.stop()
                print("Command server stopped")
            except Exception as e:
                print(f"Error stopping command server: {e}")
            
            # Clean up resources
            try:
                self.video_streamer.cleanup()
                print("Video streamer cleaned up")
            except Exception as e:
                print(f"Error cleaning up video streamer: {e}")
                
            try:
                self.command_server.cleanup()
                print("Command server cleaned up")
            except Exception as e:
                print(f"Error cleaning up command server: {e}")
                
            try:
                self.motor_controller.cleanup()
                print("Motor controller cleaned up")
            except Exception as e:
                print(f"Error cleaning up motor controller: {e}")
                
            try:
                self.servo_controller.cleanup()
                print("Servo controller cleaned up")
            except Exception as e:
                print(f"Error cleaning up servo controller: {e}")
            
            print("Cleanup complete")

if __name__ == '__main__':
    server = ForkliftServer()
    server.start() 