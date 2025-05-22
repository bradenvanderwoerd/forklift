import signal
import sys
import time
import threading
import socket
from typing import Dict, Any, Optional, Tuple
import logging
import asyncio
import RPi.GPIO as GPIO
import math
import json # Added for JSON operations
import cv2 # Import OpenCV for drawing

from .controllers.motor import MotorController
from .controllers.servo import ServoController
from .controllers.navigation import NavigationController
from .network.video_stream import VideoStreamer
from .network.overhead_video_stream import OverheadStreamer
from .network.tcp_server import CommandServer
from .network.overhead_camera_client import WarehouseCameraClient
from .localization.overhead_localizer import OverheadLocalizer
from .utils.config import (
    HOST, SERVER_TCP_PORT, SERVER_VIDEO_UDP_PORT, 
    OVERHEAD_VIDEO_WEBSOCKET_PORT,
    SERVO_PWM_PIN, MANUAL_TURN_SPEED, FORK_DOWN_POSITION, FORK_UP_POSITION,
    AUTONAV_FORK_LOWER_TO_PICKUP_ANGLE, AUTONAV_FORK_CARRY_ANGLE,
    OVERHEAD_CAMERA_HOST, OVERHEAD_CAMERA_PORT,
    # Assuming a default file path, can be moved to config.py if preferred
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

OVERHEAD_TARGETS_FILE = "overhead_targets.json" # Define path for targets file

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
        self.robot_overhead_pose: Optional[Tuple[float, float, float]] = None
        # Initialize with default structure, will be populated by _load_overhead_target_poses
        self.overhead_target_poses: Dict[str, Optional[Tuple[float, float, float]]] = {
            "pickup": None,
            "dropoff": None
        }
        self._load_overhead_target_poses() # Load poses at startup
        
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
        # Initialize Overhead Localizer
        self.overhead_localizer = OverheadLocalizer()
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
        self.command_server.register_handler('servo', self._handle_servo_command)
        self.command_server.register_handler('stop', self._handle_emergency_stop)
        self.command_server.register_handler('TOGGLE_AUTONAV', self._handle_toggle_autonav_command)
        self.command_server.register_handler('SET_NAV_TURNING_PID', self._handle_set_nav_turning_pid)
        self.command_server.register_handler('SET_NAV_DISTANCE_PID', self._handle_set_nav_distance_pid)
        self.command_server.register_handler('SET_OVERHEAD_TARGET', self._handle_set_overhead_target)
        self.command_server.register_handler('SET_SPEED', self._handle_set_speed_command)
    
    def _handle_drive_command(self, data: Dict[str, Any]):
        if self.test_autonav_active:
            logger.info("Auto-nav active, drive command ignored.")
            return
        logger.debug(f"Received DRIVE command data: {data}")
        command_value = data.get("value", {})
        direction = command_value.get("direction")
        speed = command_value.get("speed")
        action = command_value.get("action", "START")

        if action == "STOP" or (direction and direction.upper() == "NONE"):
            self.motor_controller.stop()
            logger.info("Drive command: STOP")
        elif direction and speed is not None:
            logger.info(f"Drive command: {direction} at speed {speed}")
            if direction == "FORWARD":
                self.motor_controller.drive_forward(speed)
            elif direction == "BACKWARD":
                self.motor_controller.drive_backward(speed)
            elif direction == "LEFT":
                self.motor_controller.turn_right(speed)
            elif direction == "RIGHT":
                self.motor_controller.turn_left(speed)
        elif speed is not None and direction is None and action == "UPDATE_SPEED_ONLY":
             logger.info(f"Received deprecated speed update via drive cmd: {speed}. Use SET_SPEED.")
        else:
            logger.warning(f"Unknown drive command or missing parameters: {data}")
    
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
            logger.info(f"Autonomous navigation ACTIVATED.")
            # Attempt to set the 'pickup' pose as the initial target
            pickup_target_pose = self.overhead_target_poses.get("pickup")
            if pickup_target_pose:
                self.navigation_controller.set_target(pickup_target_pose)
                self.autonav_stage = AUTONAV_STAGE_NAVIGATING # Set stage only if target is successfully set
                logger.info(f"Initial target set to 'pickup'. Stage: {self.autonav_stage}")
            else:
                logger.warning("AutoNav activated, but no 'pickup' target pose is set. Robot will not navigate.")
                self.autonav_stage = AUTONAV_STAGE_IDLE # Remain idle if no target
                self.test_autonav_active = False # Immediately turn off autonav if no target can be set.
                # self.motor_controller.stop() # Ensure motors are stopped. Already handled by clear_target if called.
        else:
            self.autonav_stage = AUTONAV_STAGE_IDLE
            logger.info(f"Autonomous navigation DEACTIVATED.")
            if hasattr(self, 'navigation_controller') and self.navigation_controller:
                self.navigation_controller.clear_target() # Stops motors and resets NavController
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

    def _handle_set_overhead_target(self, data: Dict[str, Any]):
        """Handles command to set an overhead target pose (pickup or dropoff)."""
        target_name = data.get("value", {}).get("target_name")
        
        if not target_name or target_name not in self.overhead_target_poses:
            logger.warning(f"Invalid or missing target_name for SET_OVERHEAD_TARGET: {target_name}. Expected 'pickup' or 'dropoff'.")
            return

        if self.robot_overhead_pose is not None:
            self.overhead_target_poses[target_name] = self.robot_overhead_pose
            px, py, ptheta_deg = self.robot_overhead_pose[0], self.robot_overhead_pose[1], math.degrees(self.robot_overhead_pose[2])
            logger.info(f"Overhead target '{target_name}' SET to current robot pose: X={px:.0f}, Y={py:.0f}, Theta={ptheta_deg:.1f}°")
            self._save_overhead_target_poses() # Save after setting
        else:
            logger.warning(f"Could not set overhead target '{target_name}': Robot overhead pose is currently not detected.")
    
    def _handle_set_speed_command(self, data: Dict[str, Any]):
        """Handles the SET_SPEED command from the client's speed slider."""
        if self.test_autonav_active:
            logger.info("Auto-nav active, SET_SPEED command ignored.")
            return
        
        speed_value = data.get("value")
        if speed_value is not None:
            try:
                speed = int(speed_value)
                if 0 <= speed <= 100:
                    # self.motor_controller.set_speed(speed) # This call is now redundant
                    logger.info(f"SET_SPEED command received with value {speed}. Speed is applied with drive commands.")
                else:
                    logger.warning(f"Invalid speed value for SET_SPEED: {speed}. Must be 0-100.")
            except ValueError:
                logger.error(f"Invalid speed format for SET_SPEED: {speed_value}.")
        else:
            logger.warning("SET_SPEED command received without a 'value'.")
    
    def _run_video_server(self):
        """Run video server in a separate thread"""
        try:
            self.video_streamer.start()
        except Exception as e:
            logger.error(f"Error in onboard video server thread: {e}")
            self.running = False
    
    def _run_overhead_video_server(self):
        logger.info("Overhead video server thread started.")
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self.overhead_video_streamer.start_server())
            logger.info(f"Overhead video WebSocket server started on {self.overhead_video_streamer.host}:{self.overhead_video_streamer.port}")
            while self.running:
                frame = self.overhead_camera_client.get_latest_frame() # This is a blocking call with timeout
                if frame is not None:
                    # Detect robot pose in the frame
                    current_time = time.monotonic()
                    self.robot_overhead_pose, annotated_frame = self.overhead_localizer.detect_robot_pose(frame, current_time)
                    
                    # Draw target pose indicator if a pickup target exists
                    pickup_target = self.overhead_target_poses.get("pickup")
                    if pickup_target and annotated_frame is not None:
                        # pickup_target is (x, y, theta)
                        # We only need x, y for the circle center
                        center_x = int(pickup_target[0])
                        center_y = int(pickup_target[1])
                        # Draw a yellow circle for the pickup target
                        cv2.circle(annotated_frame, (center_x, center_y), 10, (0, 255, 255), 2) # Yellow circle, radius 10, thickness 2

                    # Send the (potentially) annotated frame to clients
                    if annotated_frame is not None:
                        self.overhead_video_streamer.add_frame_to_queue(annotated_frame)
                    else:
                        # If detection failed or no frame to annotate, send the original frame
                        self.overhead_video_streamer.add_frame_to_queue(frame)
                else:
                    # logger.debug("No new frame from overhead camera client.") # Can be noisy
                    time.sleep(0.01) # Small delay if no frame to prevent busy-waiting
        except Exception as e:
            logger.error(f"Exception in overhead video server: {e}", exc_info=True)
        finally:
            if hasattr(self.overhead_video_streamer, 'server') and self.overhead_video_streamer.server:
                self.overhead_video_streamer.server.close()
                loop.run_until_complete(self.overhead_video_streamer.server.wait_closed())
            loop.close()
            logger.info("Overhead video server thread stopped.")
    
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
        
        # Variables for tracking overhead camera FPS on Pi
        self.overhead_frames_received_count = 0
        self.last_overhead_log_time = time.time()
        self.log_interval_seconds = 5.0 # Log FPS every 5 seconds
        
        try:
            self.video_thread.start()
            self.overhead_video_thread.start()
            self.command_thread.start()
            self.overhead_camera_client.connect() # Start the overhead camera client thread
            
            logger.info("ForkliftServer main loop started. Use TOGGLE_AUTONAV command to test navigation.")

            while self.running:
                # Get overhead camera frame
                raw_overhead_frame = self.overhead_camera_client.get_video_frame()
                processed_overhead_frame = None # Frame to be sent to streamer

                if raw_overhead_frame is not None:
                    self.overhead_frames_received_count += 1
                    
                    # Detect robot pose from the raw frame
                    current_overhead_pose = self.overhead_localizer.detect_robot_pose(raw_overhead_frame)
                    if current_overhead_pose is not None:
                        self.robot_overhead_pose = current_overhead_pose
                        # logger.info(f"Overhead Pose: x={self.robot_overhead_pose[0]:.0f}, y={self.robot_overhead_pose[1]:.0f}, theta={math.degrees(self.robot_overhead_pose[2]):.0f}")
                        # For visual feedback, draw the pose on the frame that will be streamed
                        processed_overhead_frame = self.overhead_localizer.draw_pose_on_frame(raw_overhead_frame.copy(), self.robot_overhead_pose)
                    else:
                        self.robot_overhead_pose = None # Explicitly set to None if not detected
                        # logger.debug("Robot marker not detected in overhead view.") # This might be too frequent
                        processed_overhead_frame = raw_overhead_frame # Send the raw frame if no pose

                    # Send the (potentially annotated) frame to the overhead streamer
                    if processed_overhead_frame is not None:
                        self.overhead_video_streamer.set_frame(processed_overhead_frame)
                    
                # else: raw_overhead_frame was None, do nothing with it for streamer

                # Log overhead camera FPS periodically (Commented out as per user request)
                # current_time_for_fps_log = time.time()
                # if current_time_for_fps_log - self.last_overhead_log_time >= self.log_interval_seconds:
                #     elapsed_time = current_time_for_fps_log - self.last_overhead_log_time
                #     if elapsed_time > 0: # Avoid division by zero if interval is very small or time didn't advance
                #         fps = self.overhead_frames_received_count / elapsed_time
                #         logger.info(f"Overhead Camera FPS (Pi perspective): {fps:.2f} FPS ({self.overhead_frames_received_count} frames in {elapsed_time:.2f}s)")
                #     else:
                #         logger.info(f"Overhead Camera FPS (Pi perspective): Unable to calculate FPS, zero elapsed time. Frames: {self.overhead_frames_received_count}")
                #     self.overhead_frames_received_count = 0
                #     self.last_overhead_log_time = current_time_for_fps_log

                if self.test_autonav_active:
                    # Autonomous navigation logic using overhead camera pose
                    current_overhead_pose_from_server = self.robot_overhead_pose # Use the continuously updated pose
                    
                    if self.autonav_stage == AUTONAV_STAGE_NAVIGATING:
                        if current_overhead_pose_from_server and self.navigation_controller.current_target_pixel_pose:
                            # Target is already set in NavController when autonav was enabled
                            nav_still_moving = self.navigation_controller.navigate(current_overhead_pose_from_server)

                            if not nav_still_moving: # Target reached according to NavigationController
                                logger.info("AutoNav (NAVIGATING): Target Reached! Transitioning to LOWERING_FORKS.")
                                # NavigationController should have already stopped motors.
                                # self.navigation_controller.clear_target() # Clear NavController's internal target for safety / next run.
                                self.autonav_stage = AUTONAV_STAGE_LOWERING_FORKS
                            # else: still navigating
                        
                        elif not current_overhead_pose_from_server:
                            logger.warning("AutoNav (NAVIGATING): Robot overhead pose not available. Stopping motion.")
                            self.motor_controller.stop() # Stop motors if pose is lost
                            # self.navigation_controller.clear_target() # Optionally clear NavController target
                        elif not self.navigation_controller.current_target_pixel_pose:
                            logger.warning("AutoNav (NAVIGATING): NavigationController has no target set. Stopping motion.")
                            self.motor_controller.stop()
                            # Consider deactivating autonav or setting to IDLE if target is unexpectedly lost from NavController
                            # self.test_autonav_active = False 
                            # self.autonav_stage = AUTONAV_STAGE_IDLE

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

    def _load_overhead_target_poses(self):
        try:
            with open(OVERHEAD_TARGETS_FILE, 'r') as f:
                loaded_poses = json.load(f)
                # Basic validation: check if keys exist, can be more thorough
                if "pickup" in loaded_poses and "dropoff" in loaded_poses:
                    self.overhead_target_poses["pickup"] = tuple(loaded_poses["pickup"]) if loaded_poses["pickup"] else None
                    self.overhead_target_poses["dropoff"] = tuple(loaded_poses["dropoff"]) if loaded_poses["dropoff"] else None
                    logger.info(f"Overhead target poses loaded from {OVERHEAD_TARGETS_FILE}.")
                    if self.overhead_target_poses["pickup"]:
                        px, py, pth = self.overhead_target_poses["pickup"]
                        logger.info(f"  Loaded Pickup: X={px:.0f}, Y={py:.0f}, Theta={math.degrees(pth):.1f}°")
                    if self.overhead_target_poses["dropoff"]:
                        dx, dy, dth = self.overhead_target_poses["dropoff"]
                        logger.info(f"  Loaded Dropoff: X={dx:.0f}, Y={dy:.0f}, Theta={math.degrees(dth):.1f}°")
                else:
                    logger.warning(f"Invalid format in {OVERHEAD_TARGETS_FILE}. Using default empty poses.")
        except FileNotFoundError:
            logger.info(f"{OVERHEAD_TARGETS_FILE} not found. Will be created when targets are set. Using default empty poses.")
        except json.JSONDecodeError:
            logger.error(f"Error decoding JSON from {OVERHEAD_TARGETS_FILE}. Using default empty poses.")
        except Exception as e:
            logger.error(f"Unexpected error loading overhead target poses: {e}. Using default empty poses.")

    def _save_overhead_target_poses(self):
        try:
            with open(OVERHEAD_TARGETS_FILE, 'w') as f:
                json.dump(self.overhead_target_poses, f, indent=4)
            logger.info(f"Overhead target poses saved to {OVERHEAD_TARGETS_FILE}.")
        except Exception as e:
            logger.error(f"Error saving overhead target poses to {OVERHEAD_TARGETS_FILE}: {e}")

if __name__ == '__main__':
    server = ForkliftServer()
    server.start() 