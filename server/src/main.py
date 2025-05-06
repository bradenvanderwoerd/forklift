import signal
import sys
import time
from typing import Dict, Any

from controllers.motor import MotorController
from controllers.servo import ServoController
from network.video_stream import VideoStreamer
from network.tcp_server import CommandServer

class ForkliftServer:
    def __init__(self):
        """Initialize server components"""
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
        print("\nShutting down...")
        self.cleanup()
        sys.exit(0)
    
    def start(self):
        """Start server components"""
        print("Starting server...")
        
        # Start video streaming
        self.video_streamer.start()
        print("Video streaming started")
        
        # Start command server
        self.command_server.start()
        print("Command server started")
        
        # Main loop
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("Cleaning up...")
        
        # Stop components
        self.video_streamer.stop()
        self.command_server.stop()
        
        # Clean up resources
        self.video_streamer.cleanup()
        self.command_server.cleanup()
        self.motor_controller.cleanup()
        self.servo_controller.cleanup()
        
        print("Cleanup complete")

if __name__ == '__main__':
    server = ForkliftServer()
    server.start() 