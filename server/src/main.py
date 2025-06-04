import signal
import sys
import time
import threading
import socket
from typing import Dict, Any, Optional, Tuple, Union
import logging
import asyncio
import RPi.GPIO as GPIO
import math
import json
import cv2
import numpy as np
import os

from .controllers.motor import MotorController
from .controllers.servo import ServoController
from .controllers.navigation import NavigationController
from .network.video_stream import VideoStreamer
from .network.overhead_video_stream import OverheadStreamer
from .network.tcp_server import CommandServer
from .network.overhead_camera_client import WarehouseCameraClient
from .localization.overhead_localizer import OverheadLocalizer
from .utils import config

# Set up logging
logging.basicConfig(
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)
logging.getLogger().setLevel(logging.INFO)
logger = logging.getLogger(__name__)

# Auto-navigation Stages
AUTONAV_STAGE_IDLE = "IDLE"
AUTONAV_STAGE_NAV_TO_PICKUP = "NAV_TO_PICKUP"
AUTONAV_STAGE_LOWER_FORKS_FOR_PICKUP = "LOWER_FORKS_FOR_PICKUP"
AUTONAV_STAGE_PICKUP_DELAY = "PICKUP_DELAY"
AUTONAV_STAGE_LIFT_FORKS_WITH_ITEM = "LIFT_FORKS_WITH_ITEM"
AUTONAV_STAGE_NAV_TO_DROPOFF = "NAV_TO_DROPOFF"
AUTONAV_STAGE_LOWER_FORKS_FOR_DROPOFF = "LOWER_FORKS_FOR_DROPOFF"
AUTONAV_STAGE_DROPOFF_DELAY = "DROPOFF_DELAY"
AUTONAV_STAGE_LIFT_FORKS_AFTER_DROPOFF = "LIFT_FORKS_AFTER_DROPOFF"
AUTONAV_STAGE_COMPLETE = "COMPLETE"

OVERHEAD_TARGETS_FILE = "overhead_targets.json"

def check_port_availability(port: int, host: str = config.HOST) -> bool:
    """Checks if a specified network port is available to bind to on the given host.

    Args:
        port: The port number to check.
        host: The host address (IP) to check the port on.

    Returns:
        True if the port is available, False otherwise.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((host, port))
            return True
    except socket.error as e:
        logger.error(f"ForkliftServer: Port check failed for {host}:{port} - {e}")
        return False

class ForkliftServer:
    """Main server application for the Forklift robot.

    This class orchestrates all components of the robot server, including:
    - Hardware controllers (motors, servos, navigation).
    - Network services for command reception and video streaming (onboard and overhead).
    - Client for the external warehouse camera.
    - Overhead localization using ArUco markers.
    - Autonomous navigation state machine.
    - Graceful startup and shutdown procedures.
    """
    def __init__(self):
        """Initializes the ForkliftServer and all its sub-components."""
        logger.info("ForkliftServer: Initializing...")
        
        # --- Global GPIO Setup ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.is_running = True
        self._cleanup_in_progress_lock = threading.Lock()
        self._cleanup_initiated = False
        self.cleanup_complete_event = threading.Event()

        self.autonav_active = False
        self.autonav_current_stage = AUTONAV_STAGE_IDLE
        self.autonav_stage_entry_time = 0.0

        self.robot_overhead_pose_pixels: Optional[Tuple[float, float, float]] = None
        self.overhead_target_poses_pixels: Dict[str, Optional[Tuple[float, float, float]]] = {
            "pickup": None,
            "dropoff": None
        }
        self._load_overhead_target_poses()

        self.overhead_cam_matrix: Optional[np.ndarray] = None
        self.overhead_cam_dist_coeffs: Optional[np.ndarray] = None
        self.new_overhead_cam_matrix: Optional[np.ndarray] = None
        self.overhead_calibration_roi: Optional[Tuple[int, int, int, int]] = None
        self._load_overhead_camera_calibration()

        if not check_port_availability(config.SERVER_TCP_PORT):
            logger.critical(f"ForkliftServer: Command port {config.SERVER_TCP_PORT} is already in use! Exiting.")
            sys.exit(1)
        if not check_port_availability(config.SERVER_VIDEO_UDP_PORT):
            logger.critical(f"ForkliftServer: Onboard video port {config.SERVER_VIDEO_UDP_PORT} is already in use! Exiting.")
            sys.exit(1)
        if not check_port_availability(config.OVERHEAD_VIDEO_WEBSOCKET_PORT):
            logger.critical(f"ForkliftServer: Overhead video port {config.OVERHEAD_VIDEO_WEBSOCKET_PORT} is already in use! Exiting.")
            sys.exit(1)
        
        self.motor_controller = MotorController()
        self.navigation_controller = NavigationController(self.motor_controller)

        self.servo_controllers: Dict[int, ServoController] = {}
        self._initialize_servo_controllers()
        self.primary_fork_servo: Optional[ServoController] = self.servo_controllers.get(config.FORK_SERVO_A_PIN)
        if self.primary_fork_servo:
            logger.info(f"Primary fork servo (Pin {config.FORK_SERVO_A_PIN}) initialized to {self.primary_fork_servo.get_position():.1f} degrees.")
        else:
            logger.warning(f"Primary fork servo (Pin {config.FORK_SERVO_A_PIN}) not found after initialization. Autonav fork control will fail.")

        self.warehouse_camera_client = WarehouseCameraClient(
            host=config.OVERHEAD_CAMERA_HOST, 
            port=config.OVERHEAD_CAMERA_PORT
        )
        self.overhead_localizer = OverheadLocalizer()
        
        self.onboard_video_streamer = VideoStreamer(host=config.HOST, port=config.SERVER_VIDEO_UDP_PORT)
        self.overhead_video_streamer = OverheadStreamer(host=config.HOST, port=config.OVERHEAD_VIDEO_WEBSOCKET_PORT)
        self.command_receiver_server = CommandServer(host=config.HOST, port=config.SERVER_TCP_PORT)
        
        self._register_command_handlers()
        
        signal.signal(signal.SIGINT, self._signal_shutdown_handler)
        signal.signal(signal.SIGTERM, self._signal_shutdown_handler)

        self.onboard_video_thread = threading.Thread(target=self.onboard_video_streamer.start, name="OnboardVideoStreamThread", daemon=True)
        self.overhead_video_thread = threading.Thread(target=self.overhead_video_streamer.start, name="OverheadVideoStreamThread", daemon=True)
        self.command_receiver_thread = threading.Thread(target=self.command_receiver_server.start, name="CommandServerThread", daemon=True)
        
        logger.info("ForkliftServer: Initialization complete.")

    def _load_overhead_camera_calibration(self):
        """Loads overhead camera calibration data (matrix and distortion coeffs)
        from the file specified in `config.OVERHEAD_CAMERA_CALIBRATION_FILE_PATH`.
        This data is used for undistorting raw frames from the warehouse camera.
        """
        try:
            calib_file_path = config.OVERHEAD_CAMERA_CALIBRATION_FILE_PATH
            if not os.path.isabs(calib_file_path):
                script_dir = os.path.dirname(__file__)
                calib_file_path = os.path.join(script_dir, calib_file_path)

            if not os.path.exists(calib_file_path):
                logger.warning(f"ForkliftServer: Overhead camera calibration file NOT FOUND at {calib_file_path}. Frames will not be undistorted.")
                return

            with np.load(calib_file_path) as data:
                self.overhead_cam_matrix = data['mtx']
                self.overhead_cam_dist_coeffs = data['dist']
                logger.info(f"ForkliftServer: Successfully loaded overhead camera calibration from {calib_file_path}.")
                
                self.new_overhead_cam_matrix = self.overhead_cam_matrix
        except FileNotFoundError:
            logger.warning(f"ForkliftServer: Overhead camera calibration file specified but not found at {config.OVERHEAD_CAMERA_CALIBRATION_FILE_PATH}.")
        except Exception as e:
            logger.error(f"ForkliftServer: Error loading overhead camera calibration data: {e}. Frames will not be undistorted.", exc_info=True)
            self.overhead_cam_matrix = None
            self.overhead_cam_dist_coeffs = None
            self.new_overhead_cam_matrix = None
            self.overhead_calibration_roi = None
        
    def _initialize_servo_controllers(self):
        """Initializes all configured servo controllers based on settings in `config.py`."""
        servo_configs = {
            "A": (config.FORK_SERVO_A_PIN, config.FORK_A_INITIAL_ANGLE, config.FORK_A_UP_ANGLE, config.FORK_A_DOWN_ANGLE),
            "B": (config.FORK_SERVO_B_PIN, config.FORK_B_INITIAL_ANGLE, config.FORK_B_UP_ANGLE, config.FORK_B_DOWN_ANGLE),
            "C": (config.FORK_SERVO_C_PIN, config.FORK_C_INITIAL_ANGLE, config.FORK_C_UP_ANGLE, config.FORK_C_DOWN_ANGLE),
            "D": (config.FORK_SERVO_D_PIN, config.FORK_D_INITIAL_ANGLE, config.FORK_D_UP_ANGLE, config.FORK_D_DOWN_ANGLE),
            "E": (config.FORK_SERVO_E_PIN, config.FORK_E_INITIAL_ANGLE, config.FORK_E_UP_ANGLE, config.FORK_E_DOWN_ANGLE),
            "F": (config.FORK_SERVO_F_PIN, config.FORK_F_INITIAL_ANGLE, config.FORK_F_UP_ANGLE, config.FORK_F_DOWN_ANGLE),
        }

        for name, params in servo_configs.items():
            pin, initial, up_pos, down_pos = params
            try:
                controller = ServoController(
                    pin_number=pin,
                    initial_position_degrees=initial,
                    defined_up_angle=up_pos,
                    defined_down_angle=down_pos
                )
                self.servo_controllers[pin] = controller
                logger.info(f"ForkliftServer: Initialized servo '{name}' on pin {pin} (Initial: {initial}°, Up: {up_pos}°, Down: {down_pos}°)")
            except Exception as e:
                logger.error(f"ForkliftServer: Failed to initialize servo '{name}' on pin {pin}: {e}", exc_info=True)
    
    def _register_command_handlers(self):
        """Registers handler methods for commands received by the `CommandServer`."""
        self.command_receiver_server.register_handler('drive', self._handle_drive_command)
        self.command_receiver_server.register_handler('servo', self._handle_servo_command)
        self.command_receiver_server.register_handler('stop_all', self._handle_emergency_stop_command)
        self.command_receiver_server.register_handler('toggle_autonav', self._handle_toggle_autonav_command)
        self.command_receiver_server.register_handler('set_nav_pid', self._handle_set_nav_pid_command)
        self.command_receiver_server.register_handler('set_overhead_target', self._handle_set_overhead_target_command)
        self.command_receiver_server.register_handler('set_speed_limits', self._handle_set_speed_limits_command)
        logger.info("ForkliftServer: Command handlers registered.")
    
    def _handle_drive_command(self, data: Dict[str, Any]):
        """Handles 'drive' commands for manual motor control.
        Payload expected: {"direction": "FORWARD"/"BACKWARD"/"LEFT"/"RIGHT"/"NONE", "speed": 0-100, "action": "START"/"STOP"}
        If autonav is active, manual drive commands are ignored.
        """
        if self.autonav_active:
            logger.info("ForkliftServer: Autonav active, manual drive command ignored.")
            return
        
        logger.debug(f"ForkliftServer: Received DRIVE command: {data}")
        direction = data.get("direction", "NONE").upper()
        speed = data.get("speed", config.MANUAL_DRIVE_SPEED)
        action = data.get("action", "START").upper()

        if action == "STOP" or direction == "NONE":
            self.motor_controller.stop()
            logger.info("ForkliftServer: Drive command: STOP/NONE - Motors stopped.")
        elif direction and isinstance(speed, (int, float)) and 0 <= speed <= 100:
            logger.info(f"ForkliftServer: Drive command: {action} {direction} at speed {speed}")
            if direction == "FORWARD":
                self.motor_controller.drive_forward(int(speed))
            elif direction == "BACKWARD":
                self.motor_controller.drive_backward(int(speed))
            elif direction == "LEFT":
                self.motor_controller.turn_left(int(speed))
            elif direction == "RIGHT":
                self.motor_controller.turn_right(int(speed))
            elif direction == "MOVE" and "forward_component" in data and "turn_component" in data:
                 fwd = int(data.get("forward_component", 0))
                 trn = int(data.get("turn_component", 0))
                 self.motor_controller.move(forward_component=fwd, turn_component=trn)
                 logger.info(f"ForkliftServer: Drive command: MOVE Fwd:{fwd}, Trn:{trn}")
            else:
                logger.warning(f"ForkliftServer: Unknown drive direction '{direction}' or invalid MOVE payload in command: {data}")
        else:
            logger.warning(f"ForkliftServer: Invalid drive command payload (missing direction/speed, or speed out of range): {data}")
    
    def _handle_servo_command(self, data: Dict[str, Any]):
        """Handles 'servo' commands for controlling individual servos.
        Payload expected: {"pin": <servo_pin_BCM>, "action": "UP"/"DOWN"/"SET_ANGLE", "angle": <degrees_if_SET_ANGLE>}
        """
        logger.debug(f"ForkliftServer: Received SERVO command: {data}")
        target_pin = data.get('pin')
        action = data.get("action", "").upper()
        angle = data.get("angle")

        if target_pin is None:
            logger.warning("ForkliftServer: Servo command missing 'pin'. Ignoring.")
            return

        servo_to_control = self.servo_controllers.get(target_pin)
        if not servo_to_control:
            logger.warning(f"ForkliftServer: No servo controller found for pin {target_pin}. Ignoring command.")
            return

        if action == "UP":
            logger.info(f"ForkliftServer: Servo on pin {target_pin} command: UP")
            servo_to_control.go_to_up_position()
        elif action == "DOWN":
            logger.info(f"ForkliftServer: Servo on pin {target_pin} command: DOWN")
            servo_to_control.go_to_down_position()
        elif action == "SET_ANGLE" and angle is not None:
            try:
                angle_deg = float(angle)
                logger.info(f"ForkliftServer: Servo on pin {target_pin} command: SET_ANGLE to {angle_deg}°")
                servo_to_control.set_position(angle_deg)
            except ValueError:
                logger.warning(f"ForkliftServer: Invalid angle '{angle}' for SET_ANGLE on pin {target_pin}. Must be a number.")
        elif action == "STOP_PULSES":
             logger.info(f"ForkliftServer: Servo on pin {target_pin} command: STOP_PULSES")
             servo_to_control.stop_pwm()
        else:
            logger.warning(f"ForkliftServer: Unknown servo action '{action}' or missing angle for SET_ANGLE on pin {target_pin}. Data: {data}")

    def _handle_emergency_stop_command(self, data: Dict[str, Any]):
        """Handles 'stop_all' command for emergency stop.
        Stops motors, cancels autonav, and could potentially stop servos.
        Payload: Not strictly required, but can be empty {}.
        """
        logger.critical(f"ForkliftServer: EMERGENCY STOP command received! Data: {data}")
        if self.autonav_active:
            logger.info("ForkliftServer: Autonav was active, cancelling due to emergency stop.")
            self.autonav_active = False
            self.autonav_current_stage = AUTONAV_STAGE_IDLE
            self.navigation_controller.clear_target()
        
        self.motor_controller.stop()
        
        logger.info("ForkliftServer: Motors stopped due to emergency stop. Autonav cancelled if active.")

    def _handle_toggle_autonav_command(self, data: Dict[str, Any]):
        """Handles 'toggle_autonav' command to start or stop the autonomous navigation sequence.
        Payload expected: {"target": "pickup" or "dropoff"} when starting.
                         Can be empty {} to stop current autonav.
        """
        target_location_name = data.get("target", "pickup").lower()
        
        if self.autonav_active:
            logger.info("ForkliftServer: TOGGLE_AUTONAV received. Autonav is active, attempting to stop.")
            self.autonav_active = False
            self.autonav_current_stage = AUTONAV_STAGE_IDLE
            self.motor_controller.stop()
            self.navigation_controller.clear_target()
            logger.info("ForkliftServer: Autonomous navigation stopped by command.")
        else:
            logger.info(f"ForkliftServer: TOGGLE_AUTONAV received. Attempting to start autonav to '{target_location_name}'.")
            if target_location_name not in self.overhead_target_poses_pixels or \
               self.overhead_target_poses_pixels[target_location_name] is None:
                logger.error(f"ForkliftServer: Cannot start autonav. Target '{target_location_name}' pose not defined.")
                return

            if self.robot_overhead_pose_pixels is None:
                logger.error("ForkliftServer: Cannot start autonav. Current robot pose unknown.")
                return

            self.autonav_active = True
            if target_location_name == "pickup":
                self.autonav_current_stage = AUTONAV_STAGE_NAV_TO_PICKUP
                target_pose_for_nav = self.overhead_target_poses_pixels["pickup"]
            elif target_location_name == "dropoff":
                self.autonav_current_stage = AUTONAV_STAGE_NAV_TO_DROPOFF
                target_pose_for_nav = self.overhead_target_poses_pixels["dropoff"]
            else:
                logger.error(f"ForkliftServer: Unknown autonav target '{target_location_name}'. Valid: 'pickup', 'dropoff'.")
                self.autonav_active = False
                return

            if target_pose_for_nav:
                self.navigation_controller.set_target(target_pose_for_nav)
                logger.info(f"ForkliftServer: Autonomous navigation started towards {self.autonav_current_stage} ({target_location_name}). Target pose: {target_pose_for_nav}")
                self.autonav_stage_entry_time = time.time()
            else:
                logger.error(f"ForkliftServer: Target pose for '{target_location_name}' is None even after check. Autonav not started.")
                self.autonav_active = False
                self.autonav_current_stage = AUTONAV_STAGE_IDLE

    def _handle_set_nav_pid_command(self, data: Dict[str, Any]):
        """Handles 'set_nav_pid' command to update PID gains for navigation controllers.
        Payload expected: {"pid_name": "point_turning"/"xy_distance"/"final_orientation", 
                         "kp": <float>, "ki": <float>, "kd": <float>}
        """
        pid_name = data.get("pid_name")
        kp = data.get("kp")
        ki = data.get("ki")
        kd = data.get("kd")

        if None in [pid_name, kp, ki, kd]:
            logger.warning(f"ForkliftServer: SET_NAV_PID command missing parameters. Received: {data}")
            return
        
        try:
            kp_f, ki_f, kd_f = float(kp), float(ki), float(kd)
            self.navigation_controller.update_pid_gains(pid_name, kp_f, ki_f, kd_f)
        except ValueError:
            logger.error(f"ForkliftServer: Invalid PID gain values in SET_NAV_PID. Must be numbers. Received: {data}")
        except Exception as e:
            logger.error(f"ForkliftServer: Error updating PID gains for '{pid_name}': {e}", exc_info=True)

    def _handle_set_overhead_target_command(self, data: Dict[str, Any]):
        """Handles 'set_overhead_target' command to define pickup or dropoff waypoints.
        These are based on the overhead camera's view and an ArUco marker placed at the target.
        Payload expected: {"target_type": "pickup"/"dropoff", "x": <px>, "y": <px>, "theta_deg": <degrees>}
        If x,y,theta are not provided, it means to capture current primary ArUco marker from onboard camera
        as the target. This part is more complex and might need UI interaction or a specific "capture target" mode.
        For now, assumes x,y,theta are provided explicitly for overhead targets.
        """
        target_type = data.get("target_type", "").lower()
        x_px = data.get("x")
        y_px = data.get("y")
        theta_deg = data.get("theta_deg")

        if target_type not in ["pickup", "dropoff"]:
            logger.warning(f"ForkliftServer: Invalid target_type '{target_type}' in SET_OVERHEAD_TARGET. Must be 'pickup' or 'dropoff'.")
            return

        if None in [x_px, y_px, theta_deg]:
            logger.warning(f"ForkliftServer: SET_OVERHEAD_TARGET for '{target_type}' missing x, y, or theta_deg. Received: {data}. Target not set.")
            if data.get("clear") == True:
                 self.overhead_target_poses_pixels[target_type] = None
                 self._save_overhead_target_poses()
                 logger.info(f"ForkliftServer: Cleared overhead target for '{target_type}'.")
            return

        try:
            x_val, y_val, theta_val_deg = float(x_px), float(y_px), float(theta_deg)
            theta_val_rad = math.radians(theta_val_deg)
            self.overhead_target_poses_pixels[target_type] = (x_val, y_val, theta_val_rad)
            self._save_overhead_target_poses()
            logger.info(f"ForkliftServer: Overhead target for '{target_type}' set to (X:{x_val:.0f}px, Y:{y_val:.0f}px, Theta:{theta_val_deg:.1f}°). Saved.")
        except ValueError:
            logger.error(f"ForkliftServer: Invalid numeric values for x, y, or theta_deg in SET_OVERHEAD_TARGET. Received: {data}")

    def _handle_set_speed_limits_command(self, data: Dict[str, Any]):
        """Handles 'set_speed_limits' to update navigation speed parameters.
        Payload example: {"NAV_MAX_FORWARD_SPEED": 80, "NAV_MAX_TURNING_SPEED": 70, "NAV_MIN_EFFECTIVE_SPEED": 60}
        Updates corresponding values in the `config` module if they exist as attributes.
        Also updates the NavigationController's internal speed limits.
        """
        logger.info(f"ForkliftServer: Received SET_SPEED_LIMITS command: {data}")
        updated_any = False
        for key, value in data.items():
            if hasattr(config, key):
                try:
                    current_val = getattr(config, key)
                    new_val = type(current_val)(value)
                    setattr(config, key, new_val)
                    logger.info(f"ForkliftServer: Config value '{key}' updated from {current_val} to {new_val}.")
                    updated_any = True
                except Exception as e:
                    logger.error(f"ForkliftServer: Failed to update config value '{key}' to '{value}': {e}")
            else:
                logger.warning(f"ForkliftServer: Config key '{key}' not found in config module.")

        if updated_any:
            self.navigation_controller.max_forward_speed = config.NAV_MAX_FORWARD_SPEED
            self.navigation_controller.max_turning_speed = config.NAV_MAX_TURNING_SPEED
            self.navigation_controller.min_effective_speed = config.NAV_MIN_EFFECTIVE_SPEED
            logger.info("ForkliftServer: NavigationController speed limits updated from config.")
            for pid_controller in [self.navigation_controller.point_turning_pid, 
                                   self.navigation_controller.xy_distance_pid, 
                                   self.navigation_controller.final_orientation_pid]:
                if hasattr(pid_controller, 'max_integral'):
                    pid_controller.max_integral = float(config.NAV_MAX_FORWARD_SPEED)
                    pid_controller.min_integral = -float(config.NAV_MAX_FORWARD_SPEED)
            logger.info("ForkliftServer: PID controller integral limits updated based on NAV_MAX_FORWARD_SPEED.")

    def _signal_shutdown_handler(self, signum, frame):
        """Handles SIGINT (Ctrl+C) and SIGTERM signals for graceful shutdown."""
        signal_name = signal.Signals(signum).name
        logger.info(f"ForkliftServer: Shutdown signal {signal_name} (ID: {signum}) received. Initiating cleanup...")
        if not self._cleanup_initiated:
             self.cleanup()
        else:
            logger.info("ForkliftServer: Cleanup already in progress or completed.")

    def start(self):
        """Starts all server components and the main processing loop."""
        logger.info("ForkliftServer: Starting all services...")
        try:
            self.onboard_video_thread.start()
            self.overhead_video_thread.start()
            self.command_receiver_thread.start()
            
            self.warehouse_camera_client.connect()

            logger.info("ForkliftServer: All network services started/starting.")
            
            while self.is_running:
                loop_start_time = time.perf_counter()

                warehouse_time_sec = self.warehouse_camera_client.get_warehouse_time()
                raw_overhead_frame = self.warehouse_camera_client.get_video_frame()

                processed_overhead_frame = None
                current_robot_pose_for_nav = None

                if raw_overhead_frame is not None:
                    if self.overhead_cam_matrix is not None and \
                       self.overhead_cam_dist_coeffs is not None and \
                       self.new_overhead_cam_matrix is not None:
                        try:
                            undistorted_frame = cv2.undistort(raw_overhead_frame, 
                                                              self.overhead_cam_matrix, 
                                                              self.overhead_cam_dist_coeffs, 
                                                              None, 
                                                              self.new_overhead_cam_matrix)
                            if self.overhead_calibration_roi:
                                x, y, w, h = self.overhead_calibration_roi
                                undistorted_frame = undistorted_frame[y:y+h, x:x+w]
                            processed_overhead_frame = undistorted_frame
                        except Exception as e:
                            logger.error(f"ForkliftServer: Error undistorting overhead frame: {e}")
                            processed_overhead_frame = raw_overhead_frame
                    else:
                        processed_overhead_frame = raw_overhead_frame

                    localization_result = self.overhead_localizer.get_robot_pose_from_frame(processed_overhead_frame)
                    
                    if localization_result:
                        self.robot_overhead_pose_pixels = localization_result.pose_xy_theta_rad
                        current_robot_pose_for_nav = self.robot_overhead_pose_pixels
                        logger.debug(f"ForkliftServer: Robot localized at X:{localization_result.pose_xy_theta_rad[0]:.0f}, Y:{localization_result.pose_xy_theta_rad[1]:.0f}, Theta:{math.degrees(localization_result.pose_xy_theta_rad[2]):.1f}°")
                        
                        processed_overhead_frame = self.overhead_localizer.draw_robot_pose_on_frame(
                            processed_overhead_frame, localization_result
                        )
                    else:
                        self.robot_overhead_pose_pixels = None
                        logger.debug("ForkliftServer: Robot not localized in current overhead frame.")
                    
                    if self.overhead_target_poses_pixels["pickup"]:
                        self.overhead_localizer.draw_target_on_frame(processed_overhead_frame, self.overhead_target_poses_pixels["pickup"], "PICKUP", (0, 255, 0))
                    if self.overhead_target_poses_pixels["dropoff"]:
                        self.overhead_localizer.draw_target_on_frame(processed_overhead_frame, self.overhead_target_poses_pixels["dropoff"], "DROPOFF", (0, 0, 255))

                    self.overhead_video_streamer.set_frame(processed_overhead_frame)
                
                if self.autonav_active and current_robot_pose_for_nav:
                    is_target_reached_nav = False
                    
                    if self.autonav_current_stage == AUTONAV_STAGE_NAV_TO_PICKUP:
                        logger.info(f"Autonav: NAV_TO_PICKUP. Current pose: X:{current_robot_pose_for_nav[0]:.0f}, Y:{current_robot_pose_for_nav[1]:.0f}, Theta:{math.degrees(current_robot_pose_for_nav[2]):.1f}°")
                        if self.navigation_controller.current_target_pixel_pose != self.overhead_target_poses_pixels["pickup"]:
                            pickup_target = self.overhead_target_poses_pixels["pickup"]
                            if pickup_target:
                                self.navigation_controller.set_target(pickup_target)
                                logger.info(f"Autonav: Re-set NAV_TO_PICKUP target: {pickup_target}")
                            else:
                                logger.error("Autonav: Pickup target is None during NAV_TO_PICKUP stage. Stopping autonav.")
                                self.autonav_active = False; self.autonav_current_stage = AUTONAV_STAGE_IDLE

                        if self.autonav_active:
                            is_target_reached_nav = self.navigation_controller.navigate(current_robot_pose_for_nav)
                            if is_target_reached_nav:
                                logger.info("Autonav: Reached PICKUP location.")
                                self.motor_controller.stop()
                                self.autonav_current_stage = AUTONAV_STAGE_LOWER_FORKS_FOR_PICKUP
                                self.autonav_stage_entry_time = time.time()
                    
                    elif self.autonav_current_stage == AUTONAV_STAGE_LOWER_FORKS_FOR_PICKUP:
                        if self.primary_fork_servo:
                            logger.info("Autonav: LOWER_FORKS_FOR_PICKUP.")
                            self.primary_fork_servo.set_position(config.AUTONAV_FORK_LOWER_TO_PICKUP_ANGLE)
                            self.autonav_current_stage = AUTONAV_STAGE_PICKUP_DELAY
                            self.autonav_stage_entry_time = time.time()
                        else:
                            logger.error("Autonav: No primary fork servo for LOWER_FORKS. Stopping autonav.")
                            self.autonav_active = False; self.autonav_current_stage = AUTONAV_STAGE_IDLE
                    
                    elif self.autonav_current_stage == AUTONAV_STAGE_PICKUP_DELAY:
                        delay_duration = config.AUTONAV_PICKUP_DELAY_SECONDS
                        if time.time() - self.autonav_stage_entry_time >= delay_duration:
                            logger.info(f"Autonav: PICKUP_DELAY of {delay_duration}s complete.")
                            self.autonav_current_stage = AUTONAV_STAGE_LIFT_FORKS_WITH_ITEM
                            self.autonav_stage_entry_time = time.time()
                            
                    elif self.autonav_current_stage == AUTONAV_STAGE_LIFT_FORKS_WITH_ITEM:
                        if self.primary_fork_servo:
                            logger.info("Autonav: LIFT_FORKS_WITH_ITEM.")
                            self.primary_fork_servo.set_position(config.AUTONAV_FORK_CARRY_ANGLE)
                            dropoff_target = self.overhead_target_poses_pixels["dropoff"]
                            if dropoff_target:
                                self.navigation_controller.set_target(dropoff_target)
                                self.autonav_current_stage = AUTONAV_STAGE_NAV_TO_DROPOFF
                                self.autonav_stage_entry_time = time.time()
                                logger.info(f"Autonav: Set NAV_TO_DROPOFF target: {dropoff_target}")
                            else:
                                logger.error("Autonav: Dropoff target not set. Cannot proceed to NAV_TO_DROPOFF. Stopping.")
                                self.autonav_active = False; self.autonav_current_stage = AUTONAV_STAGE_COMPLETE
                        else:
                             logger.error("Autonav: No primary fork servo for LIFT_FORKS. Stopping autonav.")
                             self.autonav_active = False; self.autonav_current_stage = AUTONAV_STAGE_IDLE

                    elif self.autonav_current_stage == AUTONAV_STAGE_NAV_TO_DROPOFF:
                        logger.info(f"Autonav: NAV_TO_DROPOFF. Current pose: X:{current_robot_pose_for_nav[0]:.0f}, Y:{current_robot_pose_for_nav[1]:.0f}, Theta:{math.degrees(current_robot_pose_for_nav[2]):.1f}°")
                        if self.navigation_controller.current_target_pixel_pose != self.overhead_target_poses_pixels["dropoff"]:
                             dropoff_target = self.overhead_target_poses_pixels["dropoff"]
                             if dropoff_target:
                                 self.navigation_controller.set_target(dropoff_target)
                                 logger.info(f"Autonav: Re-set NAV_TO_DROPOFF target: {dropoff_target}")
                             else:
                                 logger.error("Autonav: Dropoff target is None during NAV_TO_DROPOFF stage. Stopping autonav.")
                                 self.autonav_active = False; self.autonav_current_stage = AUTONAV_STAGE_IDLE
                        
                        if self.autonav_active:
                            is_target_reached_nav = self.navigation_controller.navigate(current_robot_pose_for_nav)
                            if is_target_reached_nav:
                                logger.info("Autonav: Reached DROPOFF location.")
                                self.motor_controller.stop()
                                self.autonav_current_stage = AUTONAV_STAGE_LOWER_FORKS_FOR_DROPOFF
                                self.autonav_stage_entry_time = time.time()

                    elif self.autonav_current_stage == AUTONAV_STAGE_LOWER_FORKS_FOR_DROPOFF:
                        if self.primary_fork_servo:
                            logger.info("Autonav: LOWER_FORKS_FOR_DROPOFF.")
                            self.primary_fork_servo.set_position(config.AUTONAV_FORK_LOWER_TO_PICKUP_ANGLE) 
                            self.autonav_current_stage = AUTONAV_STAGE_DROPOFF_DELAY
                            self.autonav_stage_entry_time = time.time()
                        else:
                            logger.error("Autonav: No primary fork servo for LOWER_FORKS_DROPOFF. Stopping autonav.")
                            self.autonav_active = False; self.autonav_current_stage = AUTONAV_STAGE_IDLE
                    
                    elif self.autonav_current_stage == AUTONAV_STAGE_DROPOFF_DELAY:
                        delay_duration = config.AUTONAV_DROPOFF_DELAY_SECONDS
                        if time.time() - self.autonav_stage_entry_time >= delay_duration:
                            logger.info(f"Autonav: DROPOFF_DELAY of {delay_duration}s complete.")
                            self.autonav_current_stage = AUTONAV_STAGE_LIFT_FORKS_AFTER_DROPOFF
                            self.autonav_stage_entry_time = time.time()

                    elif self.autonav_current_stage == AUTONAV_STAGE_LIFT_FORKS_AFTER_DROPOFF:
                        if self.primary_fork_servo:
                            logger.info("Autonav: LIFT_FORKS_AFTER_DROPOFF (to clear item).")
                            self.primary_fork_servo.set_position(config.AUTONAV_FORK_CARRY_ANGLE)
                            self.autonav_current_stage = AUTONAV_STAGE_COMPLETE
                            self.autonav_stage_entry_time = time.time()
                        else:
                            logger.error("Autonav: No primary fork servo for LIFT_FORKS_AFTER_DROPOFF. Autonav ending.")
                            self.autonav_current_stage = AUTONAV_STAGE_COMPLETE

                    elif self.autonav_current_stage == AUTONAV_STAGE_COMPLETE:
                        logger.info("Autonav: Cycle COMPLETE. Returning to IDLE.")
                        self.autonav_active = False
                        self.autonav_current_stage = AUTONAV_STAGE_IDLE
                        self.motor_controller.stop()
                        self.navigation_controller.clear_target()
                        if self.primary_fork_servo:
                             self.primary_fork_servo.set_position(config.FORK_A_INITIAL_ANGLE)

                loop_duration = time.perf_counter() - loop_start_time
                sleep_time = max(0, (1.0 / config.MAIN_LOOP_MAX_FPS) - loop_duration)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    logger.debug(f"ForkliftServer: Main loop iteration took {loop_duration*1000:.2f}ms (longer than target max FPS period). Consider optimizing.")

        except KeyboardInterrupt:
            logger.info("ForkliftServer: KeyboardInterrupt caught in main start() loop. Initiating cleanup...")
        except Exception as e:
            logger.error(f"ForkliftServer: Unhandled exception in main start() loop: {e}", exc_info=True)
        finally:
            logger.info("ForkliftServer: Main start() loop ending. Ensuring cleanup is called.")
            if not self._cleanup_initiated:
                self.cleanup()
            else:
                self.cleanup_complete_event.wait(timeout=10)
            logger.info("ForkliftServer: Start method fully exited.")

    def cleanup(self):
        """Stops all services, cleans up resources, and waits for threads to join.
        This method is designed to be called once, either by a signal handler or at
        the end of the main loop.
        """
        with self._cleanup_in_progress_lock:
            if self._cleanup_initiated:
                logger.info("ForkliftServer: Cleanup already initiated or completed. Skipping.")
                return
            self._cleanup_initiated = True
        
        logger.info("ForkliftServer: Starting cleanup sequence...")
        self.is_running = False

        logger.info("ForkliftServer: Stopping CommandServer...")
        if self.command_receiver_server: self.command_receiver_server.stop()
        
        logger.info("ForkliftServer: Stopping Onboard VideoStreamer...")
        if self.onboard_video_streamer: self.onboard_video_streamer.stop()
        
        logger.info("ForkliftServer: Stopping Overhead VideoStreamer...")
        if self.overhead_video_streamer: self.overhead_video_streamer.stop()

        logger.info("ForkliftServer: Disconnecting WarehouseCameraClient...")
        if self.warehouse_camera_client: self.warehouse_camera_client.disconnect()

        logger.info("ForkliftServer: Stopping MotorController...")
        if self.motor_controller: self.motor_controller.stop()
        
        logger.info("ForkliftServer: Cleaning up ServoControllers...")
        for pin, servo_ctrl in self.servo_controllers.items():
            try:
                logger.debug(f"Cleaning up servo on pin {pin}...")
                servo_ctrl.cleanup()
            except Exception as e:
                logger.error(f"Error cleaning up servo on pin {pin}: {e}")
        self.servo_controllers.clear()

        thread_timeout = 5.0
        threads_to_join = {
            "CommandServerThread": self.command_receiver_thread,
            "OnboardVideoStreamThread": self.onboard_video_thread,
            "OverheadVideoStreamThread": self.overhead_video_thread,
        }
        for name, thread_obj in threads_to_join.items():
            if thread_obj and thread_obj.is_alive():
                logger.info(f"ForkliftServer: Waiting for {name} to join...")
                thread_obj.join(timeout=thread_timeout)
                if thread_obj.is_alive():
                    logger.warning(f"ForkliftServer: {name} did not join within {thread_timeout}s timeout.")
                else:
                    logger.info(f"ForkliftServer: {name} joined.")
        
        logger.info("ForkliftServer: Performing GPIO.cleanup()...")
        try:
            GPIO.cleanup()
            logger.info("ForkliftServer: GPIO cleanup successful.")
        except Exception as e:
            logger.error(f"ForkliftServer: Error during GPIO.cleanup(): {e}")

        self.cleanup_complete_event.set()
        logger.info("ForkliftServer: Cleanup sequence complete.")

    def is_cleanup_complete(self) -> bool:
        """Checks if the cleanup process has finished."""
        return self.cleanup_complete_event.is_set()

    def _load_overhead_target_poses(self):
        """Loads saved pickup and dropoff target poses from a JSON file."""
        script_dir = os.path.dirname(__file__)
        file_path = os.path.join(script_dir, OVERHEAD_TARGETS_FILE)
        try:
            if os.path.exists(file_path):
                with open(file_path, 'r') as f:
                    loaded_poses = json.load(f)
                    pickup_pose = loaded_poses.get("pickup")
                    if isinstance(pickup_pose, list) and len(pickup_pose) == 3:
                        self.overhead_target_poses_pixels["pickup"] = tuple(pickup_pose)
                    
                    dropoff_pose = loaded_poses.get("dropoff")
                    if isinstance(dropoff_pose, list) and len(dropoff_pose) == 3:
                         self.overhead_target_poses_pixels["dropoff"] = tuple(dropoff_pose)
                    logger.info(f"ForkliftServer: Loaded overhead target poses from {file_path}. Pickup: {self.overhead_target_poses_pixels['pickup']}, Dropoff: {self.overhead_target_poses_pixels['dropoff']}")
            else:
                logger.info(f"ForkliftServer: Overhead targets file ({file_path}) not found. Targets will be None.")
        except Exception as e:
            logger.error(f"ForkliftServer: Error loading overhead target poses from {file_path}: {e}", exc_info=True)

    def _save_overhead_target_poses(self):
        """Saves the current pickup and dropoff target poses to a JSON file."""
        script_dir = os.path.dirname(__file__)
        file_path = os.path.join(script_dir, OVERHEAD_TARGETS_FILE)
        try:
            with open(file_path, 'w') as f:
                poses_to_save = {
                    "pickup": list(self.overhead_target_poses_pixels["pickup"]) if self.overhead_target_poses_pixels["pickup"] else None,
                    "dropoff": list(self.overhead_target_poses_pixels["dropoff"]) if self.overhead_target_poses_pixels["dropoff"] else None,
                }
                json.dump(poses_to_save, f, indent=4)
                logger.info(f"ForkliftServer: Saved overhead target poses to {file_path}.")
        except Exception as e:
            logger.error(f"ForkliftServer: Error saving overhead target poses to {file_path}: {e}", exc_info=True)

if __name__ == '__main__':
    logging.getLogger().handlers.clear()
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(threadName)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )
    main_logger = logging.getLogger("ForkliftServerApp")
    main_logger.info("Starting Forklift Server application directly...")

    server_instance = None
    try:
        server_instance = ForkliftServer()
        server_instance.start()
        
    except Exception as e:
        main_logger.critical(f"ForkliftServerApp: Critical unhandled exception at top level: {e}", exc_info=True)
    finally:
        main_logger.info("ForkliftServerApp: Main execution block finished or exited via exception.")
        if server_instance and not server_instance.is_cleanup_complete():
            main_logger.info("ForkliftServerApp: Cleanup might not have completed. Initiating/Waiting for cleanup again just in case.")
            if not server_instance._cleanup_initiated:
                 server_instance.cleanup()
            server_instance.cleanup_complete_event.wait(timeout=10)
            if not server_instance.is_cleanup_complete():
                 main_logger.warning("ForkliftServerApp: Cleanup still not marked complete after final wait.")
        
        main_logger.info("ForkliftServerApp: Exiting.")
 