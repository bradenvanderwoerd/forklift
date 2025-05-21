import signal
import sys
import time
import threading
import socket
from typing import Dict, Any
import logging
import asyncio
import RPi.GPIO as GPIO
import math

from .controllers.motor import MotorController
from .controllers.servo import ServoController
from .controllers.navigation import NavigationController
from .network.video_stream import VideoStreamer
from .network.overhead_video_stream import OverheadStreamer
from .network.tcp_server import CommandServer
from .network.overhead_camera_client import WarehouseCameraClient
from .utils.config import (
    HOST, SERVER_TCP_PORT, SERVER_VIDEO_UDP_PORT, 
    OVERHEAD_VIDEO_WEBSOCKET_PORT,
    SERVO_PWM_PIN, MANUAL_TURN_SPEED, FORK_DOWN_POSITION, FORK_UP_POSITION,
    AUTONAV_FORK_LOWER_TO_PICKUP_ANGLE, AUTONAV_FORK_CARRY_ANGLE,
    OVERHEAD_CAMERA_HOST, OVERHEAD_CAMERA_PORT
)

# Set up logging
logging.basicConfig(
    # level=logging.INFO, # Level will be set by getLogger().setLevel()
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
    ]
)
logging.getLogger().setLevel(logging.INFO) # Set root logger level
logger = logging.getLogger(__name__) # Get a logger for the current module

# Auto-navigation Stages
AUTONAV_STAGE_IDLE = "IDLE"
AUTONAV_STAGE_NAVIGATING = "NAVIGATING"
AUTONAV_STAGE_LOWERING_FORKS = "LOWERING_FORKS"
AUTONAV_STAGE_LIFTING_FORKS = "LIFTING_FORKS"
# AUTONAV_STAGE_CARRYING_BOX = "CARRYING_BOX" # Future state if needed

def check_port_availability(port: int, host: str = HOST) -> bool:
    """Check if a port is available for use on a given host."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind((host, port))
            return True
        except socket.error as e:
            logger.error(f"Port check failed for {host}:{port} - {e}")
            return False

class ForkliftServer:
    def __init__(self):
        # GPIO setup - Call setmode and setwarnings once globally here
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.running = True
        self.cleanup_lock = threading.Lock()
        self.cleanup_complete = threading.Event()
        self.test_autonav_active = False # Flag for testing autonomous navigation
        self.autonav_stage = AUTONAV_STAGE_IDLE # Current stage of auto-navigation
        self.robot_overhead_pose = None # To store (x_pixel, y_pixel, theta_pixel_frame)
        
        # Check port availability
        if not check_port_availability(SERVER_TCP_PORT):
            sys.exit(f"Command port {SERVER_TCP_PORT} is already in use!")
        if not check_port_availability(SERVER_VIDEO_UDP_PORT):
            sys.exit(f"Onboard video port {SERVER_VIDEO_UDP_PORT} is already in use!")
        if not check_port_availability(OVERHEAD_VIDEO_WEBSOCKET_PORT):
            sys.exit(f"Overhead video port {OVERHEAD_VIDEO_WEBSOCKET_PORT} is already in use!")
        
        # Initialize hardware controllers
        self.motor_controller = MotorController()
        self.servo_controller = ServoController()
        self.navigation_controller = NavigationController(self.motor_controller)
        
        # Initialize Overhead Camera Client
        self.overhead_camera_client = WarehouseCameraClient(
            host=OVERHEAD_CAMERA_HOST, 
            port=OVERHEAD_CAMERA_PORT
        )
        # Explicitly set to FORK_DOWN_POSITION (80) on startup
        self.servo_controller.set_position(FORK_DOWN_POSITION)  
        
        # Initialize network components, passing configured host and ports
        self.video_streamer = VideoStreamer(host=HOST, port=SERVER_VIDEO_UDP_PORT)
        self.overhead_video_streamer = OverheadStreamer(host=HOST, port=OVERHEAD_VIDEO_WEBSOCKET_PORT)
        self.command_server = CommandServer(host=HOST, port=SERVER_TCP_PORT)
        
        # Register command handlers
        self._register_handlers()
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._handle_shutdown)
        signal.signal(signal.SIGTERM, self._handle_shutdown)
        
        # Create threads for servers
        self.video_thread = threading.Thread(target=self._run_video_server, name="OnboardVideoStreamThread")
        self.overhead_video_thread = threading.Thread(target=self._run_overhead_video_server, name="OverheadVideoStreamThread")
        self.command_thread = threading.Thread(target=self._run_command_server, name="CommandServerThread")
        # Overhead camera client manages its own thread, started by connect()
    
    def _register_handlers(self):
        """Register command handlers"""
        self.command_server.register_handler('drive', self._handle_drive_command)
        self.command_server.register_handler('motor', self._handle_motor_command)
        self.command_server.register_handler('servo', self._handle_servo_command)
        self.command_server.register_handler('stop', self._handle_emergency_stop)
        self.command_server.register_handler('TOGGLE_AUTONAV', self._handle_toggle_autonav_command)
        self.command_server.register_handler('SET_NAV_TURNING_PID', self._handle_set_nav_turning_pid)
        self.command_server.register_handler('SET_NAV_DISTANCE_PID', self._handle_set_nav_distance_pid)
    
    def _handle_drive_command(self, data: Dict[str, Any]):
        if self.test_autonav_active:
            logger.info("Auto-nav active, drive command ignored.")
            return
        logger.debug(f"Received DRIVE command data: {data}")
        command_value = data.get("value", {})
        direction = command_value.get("direction", "NONE")
        speed = command_value.get("speed", 0)
        action = command_value.get("action", "START")
        if action == "STOP" or direction == "NONE":
            self.motor_controller.stop()
        elif direction == "FORWARD":
            self.motor_controller.drive_forward(speed)
        elif direction == "BACKWARD":
            self.motor_controller.drive_backward(speed)
        elif direction == "LEFT":
            self.motor_controller.turn_right(MANUAL_TURN_SPEED)
        elif direction == "RIGHT":
            self.motor_controller.turn_left(MANUAL_TURN_SPEED)
        else:
            self.motor_controller.stop()
    
    def _handle_motor_command(self, data: Dict[str, Any]):
        if self.test_autonav_active:
            logger.info("Auto-nav active, motor command ignored.")
            return
        logger.debug(f"Received MOTOR command: {data}")
        speed = data.get('speed', 0)
        self.motor_controller.set_speed(speed)
    
    def _handle_servo_command(self, data: Dict[str, Any]):
        logger.debug(f"Received SERVO command: {data}")
        value = data.get("value", {})
        position = value.get('position')
        action = value.get("action", "SET")
        if action == "SET" and position is not None:
            try:
                self.servo_controller.set_position(float(position))
            except ValueError:
                logger.error(f"Invalid servo position: {position}")
        elif value.get('step_up'):
            self.servo_controller.go_to_up_position()
        elif value.get('step_down'):
            self.servo_controller.go_to_down_position()
        else:
            logger.warning(f"Unknown servo action or missing position in command: {value}")
    
    def _handle_emergency_stop(self, data: Dict[str, Any]):
        """Handle emergency stop command"""
        logger.info("EMERGENCY STOP received.")
        self.motor_controller.stop()
        if self.test_autonav_active:
            self.test_autonav_active = False
            self.autonav_stage = AUTONAV_STAGE_IDLE
            logger.info("Autonomous navigation DEACTIVATED by E-Stop.")
        
        # Stop navigation controller
        if hasattr(self, 'navigation_controller') and self.navigation_controller:
            self.navigation_controller.clear_target() # Stops motors and clears nav target
        else:
            # Fallback if navigation_controller somehow not present, directly stop motors
            self.motor_controller.stop()
        
        # self.servo_controller.set_position(0) # Optional: reset servo on e-stop. Current plan doesn't specify this.
        logger.info("Emergency stop: All motor activity should cease.")
    
    def _handle_toggle_autonav_command(self, data: Dict[str, Any]):
        """Handles the command to toggle autonomous navigation test mode."""
        self.test_autonav_active = not self.test_autonav_active
        if self.test_autonav_active:
            self.autonav_stage = AUTONAV_STAGE_NAVIGATING
            logger.info(f"Autonomous navigation ACTIVATED. Stage: {self.autonav_stage}")
            # Ensure servo is in a known state if needed, e.g., up or carry.
            # For now, we assume it starts appropriately or the first action will set it.
        else:
            self.autonav_stage = AUTONAV_STAGE_IDLE
            logger.info(f"Autonomous navigation DEACTIVATED.")
            if hasattr(self, 'navigation_controller') and self.navigation_controller:
                self.navigation_controller.clear_target() # Stops motors
            # Optionally, reset servo to a default position, e.g., down
            logger.info("Setting servo to FORK_DOWN_POSITION on auto-nav deactivation.")
            self.servo_controller.set_position(FORK_DOWN_POSITION) 
    
    def _handle_set_nav_turning_pid(self, data: Dict[str, Any]):
        """Handles command to set turning PID gains for NavigationController."""
        if not hasattr(self, 'navigation_controller') or not self.navigation_controller:
            logger.error("NavigationController not available to set PID gains.")
            return
        
        pid_values = data.get("value", {})
        try:
            kp, ki, kd = float(pid_values["kp"]), float(pid_values["ki"]), float(pid_values["kd"])
            self.navigation_controller.update_turning_pid_gains(kp, ki, kd)
            logger.info(f"Turning PID updated: Kp={kp}, Ki={ki}, Kd={kd}")
        except (TypeError, ValueError, KeyError) as e:
            logger.error(f"Invalid turning PID values: {pid_values}. Error: {e}")

    def _handle_set_nav_distance_pid(self, data: Dict[str, Any]):
        """Handles command to set distance PID gains for NavigationController."""
        if not hasattr(self, 'navigation_controller') or not self.navigation_controller:
            logger.error("NavigationController not available to set PID gains.")
            return
        
        pid_values = data.get("value", {})
        try:
            kp, ki, kd = float(pid_values["kp"]), float(pid_values["ki"]), float(pid_values["kd"])
            self.navigation_controller.update_distance_pid_gains(kp, ki, kd)
            logger.info(f"Distance PID updated: Kp={kp}, Ki={ki}, Kd={kd}")
        except (TypeError, ValueError, KeyError) as e:
            logger.error(f"Invalid distance PID values: {pid_values}. Error: {e}")
    
    def _run_video_server(self):
        """Run video server in a separate thread"""
        try:
            self.video_streamer.start()
        except Exception as e:
            logger.error(f"Error in onboard video server thread: {e}")
            self.running = False
    
    def _run_overhead_video_server(self):
        """Run overhead video server in a separate thread"""
        try:
            self.overhead_video_streamer.start()
        except Exception as e:
            logger.error(f"Error in overhead video server thread: {e}")
            self.running = False
    
    def _run_command_server(self):
        """Run command server in a separate thread"""
        try:
            self.command_server.start()
        except Exception as e:
            logger.error(f"Error in command server thread: {e}")
            self.running = False
    
    def _handle_shutdown(self, signum, frame):
        """Handle shutdown signal"""
        logger.info(f"Shutdown signal {signum} received. Telling main loop to stop...")
        self.running = False
    
    def start(self):
        """Start server components"""
        logger.info("Starting ForkliftServer...")
        logger.info(f"Command server on {HOST}:{SERVER_TCP_PORT}")
        logger.info(f"Onboard Video (PiCam) on ws://{HOST}:{SERVER_VIDEO_UDP_PORT}")
        logger.info(f"Overhead Video (Warehouse) on ws://{HOST}:{OVERHEAD_VIDEO_WEBSOCKET_PORT}")
        
        try:
            self.video_thread.start()
            self.overhead_video_thread.start()
            self.command_thread.start()
            self.overhead_camera_client.connect() # Start the overhead camera client thread
            
            logger.info("ForkliftServer main loop started. Use TOGGLE_AUTONAV command to test navigation.")

            while self.running:
                # Get overhead camera frame
                overhead_frame = self.overhead_camera_client.get_video_frame()
                if overhead_frame is not None:
                    logger.debug(f"Received overhead frame of shape: {overhead_frame.shape}. Forwarding to OverheadStreamer.")
                    self.overhead_video_streamer.set_frame(overhead_frame)
                    # Here, you would also pass overhead_frame to OverheadLocalizer to get self.robot_overhead_pose
                # else: VideoStreamer will use its own Pi camera frame by default

                if self.test_autonav_active:
                    current_pose_onboard = self.video_streamer.shared_primary_target_pose # This is from onboard camera
                    # TODO: Replace current_pose with self.robot_overhead_pose for navigation decisions
                    
                    if self.autonav_stage == AUTONAV_STAGE_NAVIGATING:
                        if current_pose_onboard:
                            tvec, rvec = current_pose_onboard
                            self.navigation_controller.set_target(tvec, rvec) 
                            
                            nav_still_moving = self.navigation_controller.navigate()

                            x_cam = tvec[0][0]
                            z_cam = tvec[0][2]
                            
                            is_definitely_at_target = False
                            if z_cam > 0: # Valid pose for calculation
                                angle_to_marker_rad = math.atan2(x_cam, z_cam)
                                current_planar_distance_m = math.sqrt(x_cam**2 + z_cam**2)
                                if self.navigation_controller.is_at_target(angle_to_marker_rad, current_planar_distance_m):
                                    logger.info("ForkliftServer (AutoNav NAVIGATING): Target Reached (checked in main loop via is_at_target).")
                                    is_definitely_at_target = True
                            
                            if not nav_still_moving and not is_definitely_at_target:
                                logger.info("ForkliftServer (AutoNav NAVIGATING): navigate() indicates movement stopped (likely target reached by its own logic).")
                                is_definitely_at_target = True

                            if is_definitely_at_target:
                                logger.info("AutoNav: Target Reached! Transition: NAVIGATING -> LOWERING_FORKS")
                                self.navigation_controller.clear_target() # Stop motors
                                self.autonav_stage = AUTONAV_STAGE_LOWERING_FORKS
                            # else: Still navigating or target calculations were invalid (z_cam <= 0)
                        
                        else: # No current_pose during NAVIGATING stage
                            logger.info("ForkliftServer (AutoNav NAVIGATING): Primary target lost. Stopping navigation.")
                            self.navigation_controller.clear_target() # Stop motors

                    elif self.autonav_stage == AUTONAV_STAGE_LOWERING_FORKS:
                        logger.info(f"AutoNav: Stage LOWERING_FORKS. Lowering forks to {AUTONAV_FORK_LOWER_TO_PICKUP_ANGLE} deg.")
                        self.servo_controller.set_position(AUTONAV_FORK_LOWER_TO_PICKUP_ANGLE, blocking=True)
                        logger.info("AutoNav: Forks lowered. Simulating pickup (delay 1s). Transition: LOWERING_FORKS -> LIFTING_FORKS")
                        time.sleep(1.0) # Simulate pickup time
                        self.autonav_stage = AUTONAV_STAGE_LIFTING_FORKS

                    elif self.autonav_stage == AUTONAV_STAGE_LIFTING_FORKS:
                        logger.info(f"AutoNav: Stage LIFTING_FORKS. Lifting forks to {AUTONAV_FORK_CARRY_ANGLE} deg.")
                        self.servo_controller.set_position(AUTONAV_FORK_CARRY_ANGLE, blocking=True) # Corrected call
                        logger.info("AutoNav: Forks lifted. Sequence complete. Transition: LIFTING_FORKS -> IDLE")
                        self.autonav_stage = AUTONAV_STAGE_IDLE 
                        # To re-run, user must toggle autonav off then on, or logic for re-engaging NAVIGATING from IDLE would be needed.

                    elif self.autonav_stage == AUTONAV_STAGE_IDLE:
                        # logger.debug("AutoNav: Stage IDLE. Waiting for commands or new target acquisition if autonav is toggled.")
                        pass
                
                else: # self.test_autonav_active is False
                    if self.autonav_stage != AUTONAV_STAGE_IDLE:
                        logger.warning(f"Autonav not active, but stage was {self.autonav_stage}. Resetting to IDLE and stopping nav.")
                        if hasattr(self, 'navigation_controller') and self.navigation_controller:
                            self.navigation_controller.clear_target()
                        self.autonav_stage = AUTONAV_STAGE_IDLE
                        
                time.sleep(0.02) # Main loop tick rate (50Hz)
                
        except KeyboardInterrupt:
            logger.info("\nKeyboard interrupt (Ctrl+C) received. Initiating shutdown...")
            self.running = False # Ensure running is false, though signal handler should also do this
        finally:
            logger.info("ForkliftServer._run_main_loop_async.finally: Main loop ended or exception. Ensuring motors stopped.")
            self.motor_controller.stop()
            if self.servo_controller:
                logger.info("ForkliftServer._run_main_loop_async.finally: Cleaning up servo controller.")
                self.servo_controller.cleanup()
            
            # Disconnect overhead camera client
            if self.overhead_camera_client:
                logger.info("ForkliftServer cleanup: Disconnecting overhead camera client.")
                self.overhead_camera_client.disconnect()

            logger.info("Main loop terminated. Proceeding to cleanup...")
            self.cleanup() # Primary call to cleanup
            # Wait for cleanup to complete before exiting application context
            if not self.cleanup_complete.wait(timeout=10.0): # Increased timeout
                logger.warning("Cleanup did not complete in the allotted time.")
            logger.info("ForkliftServer application shutdown sequence finished.")

    def cleanup(self):
        """Clean up resources"""
        if not self.cleanup_lock.acquire(blocking=False):
            logger.warning("Cleanup already in progress")
            return
            
        try:
            logger.info("Cleaning up ForkliftServer resources...")

            # Ensure motors are stopped and servo is reset first
            try:
                logger.info("Stopping motors definitively...")
                self.motor_controller.stop()
            except Exception as e:
                logger.error(f"Error during motor_controller.stop() in cleanup: {e}")
            try:
                logger.info("Resetting servo to default down position definitively...") # Updated log to reflect 80 degrees
                self.servo_controller.set_position(FORK_DOWN_POSITION) # Use FORK_DOWN_POSITION (80)
            except Exception as e:
                logger.error(f"Error during servo_controller.set_position(FORK_DOWN_POSITION) in cleanup: {e}")
            
            # Then stop network components
            try:
                self.video_streamer.stop()
                logger.info("Video streamer stopped")
            except Exception as e:
                logger.error(f"Error stopping video streamer: {e}")
                
            try:
                self.overhead_video_streamer.stop()
                logger.info("Overhead video streamer stopped")
            except Exception as e:
                logger.error(f"Error stopping overhead video streamer: {e}")
                
            try:
                self.command_server.stop()
                logger.info("Command server stopped")
            except Exception as e:
                logger.error(f"Error stopping command server: {e}")
            
            # Then clean up resources for network components
            try:
                self.video_streamer.cleanup()
                logger.info("Video streamer cleaned up")
            except Exception as e:
                logger.error(f"Error cleaning up video streamer: {e}")
                
            try:
                self.overhead_video_streamer.cleanup()
                logger.info("Overhead video streamer cleaned up")
            except Exception as e:
                logger.error(f"Error cleaning up overhead video streamer: {e}")
                
            try:
                self.command_server.cleanup()
                logger.info("Command server cleaned up")
            except Exception as e:
                logger.error(f"Error cleaning up command server: {e}")
                
            # Then clean up hardware controllers
            try:
                self.motor_controller.cleanup()
                logger.info("Motor controller cleaned up")
            except Exception as e:
                logger.error(f"Error cleaning up motor controller: {e}")
                
            try:
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