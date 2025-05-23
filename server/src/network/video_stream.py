import cv2
import numpy as np
import asyncio
import logging
import threading
from typing import Optional
import signal
from websockets.server import serve as ws_serve
from websockets.exceptions import ConnectionClosed
import io
import time
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
import cv2.aruco as aruco
import os
from ..utils import config # Added import for config
import socket

logger = logging.getLogger(__name__)

class AdaptiveQualityController:
    def __init__(self, initial_quality: int = 80):
        """Initialize quality controller
        
        Args:
            initial_quality: Initial JPEG quality (0-100)
        """
        self.quality = initial_quality
        self.min_quality = 20
        self.max_quality = 95
        self.step_size = 5
        self.last_adjustment = time.time()
        self.adjustment_interval = 1.0  # seconds
    
    def adjust_quality(self, frame_time: float, target_time: float = 0.033):
        """Adjust quality based on frame processing time
        
        Args:
            frame_time: Time taken to process last frame
            target_time: Target frame processing time (default: 30 FPS)
        """
        current_time = time.time()
        if current_time - self.last_adjustment < self.adjustment_interval:
            return
        
        if frame_time > target_time * 1.1:  # Too slow
            self.quality = max(self.min_quality, self.quality - self.step_size)
        elif frame_time < target_time * 0.9:  # Too fast
            self.quality = min(self.max_quality, self.quality + self.step_size)
        
        self.last_adjustment = current_time
    
    def get_quality(self) -> int:
        """Get current quality setting
        
        Returns:
            Current JPEG quality (0-100)
        """
        return self.quality

class FrameSequencer:
    def __init__(self):
        """Initialize frame sequencer"""
        self.sequence = 0
    
    def next(self) -> int:
        """Get next frame sequence number
        
        Returns:
            Next sequence number
        """
        self.sequence = (self.sequence + 1) % 65536
        return self.sequence

class VideoStreamer:
    def __init__(self, host: str = "0.0.0.0", port: int = 3457):
        self.host = host
        self.port = port
        self.server = None
        self.clients = set()
        self.running = False
        self._stop_event = None
        self.camera = None
        self.loop = None
        self.quality_controller = AdaptiveQualityController()
        self.last_logged_primary_target_id = None
        self.shared_primary_target_pose = None # For sharing latest primary target pose with ForkliftServer
        
        # Use ArUco settings from config
        self.aruco_dict = aruco.getPredefinedDictionary(config.ARUCO_DICTIONARY)
        self.aruco_params = aruco.DetectorParameters() # Consider making aruco_params configurable if needed
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        logger.info(f"ArUco detector initialized with dictionary ID: {config.ARUCO_DICTIONARY}.")

        # Load camera calibration data from config path
        self.camera_matrix = None
        self.dist_coeffs = None
        calibration_file_path = config.CAMERA_CALIBRATION_FILE_PATH
        
        try:
            with np.load(calibration_file_path) as data:
                self.camera_matrix = data['mtx']
                self.dist_coeffs = data['dist']
                logger.info(f"Successfully loaded camera calibration data from {calibration_file_path}")
                logger.info(f"Camera Matrix (mtx):\\n{self.camera_matrix}") 
                logger.info(f"Principal Point: cx = {self.camera_matrix[0, 2]}, cy = {self.camera_matrix[1, 2]}")
                logger.info(f"Video Width from config: {config.VIDEO_WIDTH}, Video Height from config: {config.VIDEO_HEIGHT}")
        except FileNotFoundError:
            logger.warning(f"Camera calibration file not found at {calibration_file_path}. Pose estimation will be disabled.")
        except Exception as e:
            logger.error(f"Error loading camera calibration data from {calibration_file_path}: {e}")

        # Use marker size from config
        self.marker_actual_size_m = config.ARUCO_MARKER_SIZE_METERS
        
    def _initialize_camera(self):
        """Initialize the camera"""
        try:
            # Initialize the camera using picamera2
            self.camera = Picamera2()
            
            # --- Inspect sensor modes (for debugging/understanding) ---
            try:
                modes = self.camera.sensor_modes
                if modes:
                    logger.info(f"Available sensor modes ({len(modes)} total):")
                    for i, mode in enumerate(modes):
                        logger.info(f"  Sensor Mode {i}: {mode}")
                else:
                    logger.info("No sensor modes reported by camera.")
            except Exception as e:
                logger.error(f"Error inspecting sensor modes: {e}")
            # --- End inspection ---
            
            # Define the full sensor area for ScalerCrop (assuming a V2 camera, 3280x2464)
            # This tells the camera to use the full sensor and then scale to the main/lores size.
            # If you have a different camera (V1, V3, HQ), these max values might differ.
            # For a Raspberry Pi Camera Module v2, max resolution is 3280x2464.
            # For Camera Module v3, it's 4608x2592.
            # Using V2's max resolution as a common default.
            # If you know your exact camera, update these:
            full_sensor_width = 3280 
            full_sensor_height = 2464

            logger.info(f"Attempting to set ScalerCrop to (0, 0, {full_sensor_width}, {full_sensor_height}) to maximize FoV.")
            logger.info(f"Output resolution will be {config.VIDEO_WIDTH}x{config.VIDEO_HEIGHT}.")

            camera_config = self.camera.create_video_configuration(
                main={"size": (config.VIDEO_WIDTH, config.VIDEO_HEIGHT), "format": "RGB888"},
                lores={"size": (config.VIDEO_WIDTH, config.VIDEO_HEIGHT), "format": "YUV420"}, # Keep lores same for simplicity or adjust if needed
                controls={"ScalerCrop": (0, 0, full_sensor_width, full_sensor_height)}
            )
            self.camera.configure(camera_config)
            
            # Start the camera
            self.camera.start()
            
            # Verify camera is working by attempting to capture a frame
            try:
                self.camera.capture_array()
                logger.info("Camera initialized successfully")
                logger.info(f"Camera configuration: {camera_config}")
            except Exception as e:
                raise RuntimeError(f"Failed to start camera: {e}")
                
        except Exception as e:
            logger.error(f"Error initializing camera: {e}")
            if self.camera:
                self.camera.stop()
                self.camera = None
            raise
            
    async def _handle_client(self, websocket, path):
        self.clients.add(websocket)
        logger.info(f"New video client connected. Total clients: {len(self.clients)}")
        try:
            while self.running:
                # REMOVE external frame logic block
                # current_frame_to_process = None
                # is_external_frame = False

                # # Check for an external frame first
                # with self.external_frame_lock:
                #     if self.external_frame is not None:
                #         current_frame_to_process = self.external_frame
                #         self.external_frame = None # Consume the frame
                #         is_external_frame = True
                #         logger.debug("VideoStreamer: Using external frame for this cycle.")
                
                # if not is_external_frame:
                #     # If no external frame, use PiCamera

                # REVERT to direct PiCamera usage
                if not self.camera:
                    logger.error("PiCamera not initialized.")
                    await asyncio.sleep(1)
                    continue
                
                current_frame_to_process = None # Initialize
                try:
                    frame_color_pi = self.camera.capture_array()
                    if frame_color_pi is None:
                        logger.error("Failed to capture frame from PiCamera")
                        await asyncio.sleep(0.1)
                        continue
                    current_frame_to_process = cv2.rotate(frame_color_pi, cv2.ROTATE_180)
                except Exception as e:
                    logger.error(f"Error capturing or rotating PiCamera frame: {e}")
                    await asyncio.sleep(0.1)
                    continue

                if current_frame_to_process is None:
                    logger.warning("No frame available to process for this cycle.") # Should be redundant now
                    await asyncio.sleep(0.1)
                    continue

                # --- ArUco Detection (Always for PiCamera frames now) ---
                frame_display = current_frame_to_process # Default to current frame
                # if not is_external_frame: <--- REMOVE THIS CONDITION
                aruco_start_time = time.time()
                gray_frame = cv2.cvtColor(current_frame_to_process, cv2.COLOR_RGB2GRAY)
                corners, ids, rejected = self.aruco_detector.detectMarkers(gray_frame)
                
                if ids is not None:
                    frame_display = aruco.drawDetectedMarkers(current_frame_to_process.copy(), corners, ids)
                    # ... (rest of ArUco processing as it was, no more is_external_frame checks needed inside here)
                elif self.last_logged_primary_target_id is not None: # No IDs this frame, but was tracking
                    logger.info(f"Primary Target (ID: {self.last_logged_primary_target_id}) Lost (no markers detected this frame).")
                    self.last_logged_primary_target_id = None
                    self.shared_primary_target_pose = None

                # --- Encoding and Sending --- 
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality_controller.get_quality()]
                result, encoded_jpeg = cv2.imencode('.jpg', frame_display, encode_param)
                
                if not result:
                    logger.error("Failed to encode frame to JPEG")
                    await asyncio.sleep(0.1)
                    continue
                
                # FrameSequencer could be used here if your client needs it
                # For simplicity, direct send for now.
                
                message = encoded_jpeg.tobytes()
                
                # Send to all connected clients
                # Use a copy of self.clients for safe iteration if clients can connect/disconnect concurrently
                clients_copy = list(self.clients)
                for client_ws in clients_copy:
                    try:
                        await client_ws.send(message)
                    except ConnectionClosed:
                        logger.info("Client connection closed, removing from list.")
                        self.clients.remove(client_ws)
                    except Exception as e:
                        logger.error(f"Error sending frame to client: {e}")
                        # Optionally remove problematic client
                        # self.clients.remove(client_ws) 
                
                # Small delay to prevent hogging CPU if not much is happening
                await asyncio.sleep(0.01) 
        
        except ConnectionClosed:
            logger.info("Client disconnected gracefully.")
        except Exception as e:
            logger.error(f"Error in client handler: {e}", exc_info=True)
        finally:
            self.clients.remove(websocket)
            logger.info(f"Client removed. Total clients: {len(self.clients)}")

    async def _send_video_data_udp(self):
        """Sends video data over UDP. (Conceptual - to be implemented/adapted)
           This is where the old UDP logic would go, adapted for external frames.
        """
        logger.info(f"UDP Video Streamer starting on {self.host}:{self.port}")
        # udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) <-- socket was imported but not used here originally
        # ... (rest of placeholder UDP logic) ...
        logger.info("UDP Video Streamer stopped.")

    async def _shutdown_server_resources(self):
        logger.info(f"VideoStreamer ({self.host}:{self.port}) shutting down resources...")
        closing_tasks = []
        for client_ws in list(self.clients):
            if client_ws.open:
                task = asyncio.wait_for(client_ws.close(), timeout=2.0)
                closing_tasks.append(task)
        
        if closing_tasks:
            results = await asyncio.gather(*closing_tasks, return_exceptions=True)
            for i, result in enumerate(results):
                if isinstance(result, asyncio.TimeoutError):
                    logger.warning(f"VideoStreamer: Timeout closing client connection {i+1}.")
                elif isinstance(result, Exception):
                    logger.error(f"VideoStreamer: Error closing client connection {i+1}: {result}")
        self.clients.clear()

        if self.server:
            self.server.close()
            await self.server.wait_closed()
            logger.info(f"VideoStreamer ({self.host}:{self.port}) has been closed.")

    async def start_server_async(self):
        """Start the WebSocket server (async part)"""
        self._stop_event = asyncio.Event()
        async with ws_serve(
            self._handle_client,
            self.host,
            self.port,
            ping_interval=20,
            ping_timeout=20,
            max_size=None
        ) as server:
            self.server = server
            self.running = True
            logger.info(f"Video server started on {self.host}:{self.port}")
            await self._stop_event.wait()
            logger.info("VideoStreamer stop event received, proceeding to resource shutdown.")
            await self._shutdown_server_resources()
            
    def start(self):
        """Start the video streamer (called by ForkliftServer thread)"""
        try:
            self._initialize_camera()
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self.start_server_async())
        except Exception as e:
            logger.error(f"Error starting video streamer: {e}", exc_info=True)
            self.cleanup()
        finally:
            if self.loop and not self.loop.is_closed():
                logger.info("Closing VideoStreamer event loop in start() finally.")
                self.loop.close()
            logger.info("VideoStreamer start() method finished.")
            
    def stop(self):
        """Stop the video streamer (called by ForkliftServer)"""
        if not self.running and not (self.loop and self.loop.is_running()):
            logger.info(f"VideoStreamer stop() called, but server or loop not active.")
            return

        logger.info(f"VideoStreamer stop() called. Signaling stop event.")
        self.running = False
        
        if self.loop and self._stop_event and not self._stop_event.is_set():
            self.loop.call_soon_threadsafe(self._stop_event.set)
        else:
            logger.info("VideoStreamer: Loop or _stop_event not available or already set.")
            
    def cleanup(self):
        """Clean up resources (called by ForkliftServer)"""
        logger.info(f"Cleaning up VideoStreamer...")
        if self.camera:
            try:
                self.camera.stop()
                self.camera = None
                logger.info("Camera stopped and released in VideoStreamer cleanup.")
            except Exception as e:
                logger.error(f"Error stopping camera in VideoStreamer cleanup: {e}")
        
        if self.loop and not self.loop.is_closed():
             logger.warning("VideoStreamer event loop was not closed by start() method; closing in cleanup.")
             current_thread = threading.current_thread()
             if self.loop._thread_id != current_thread.ident:
                 time.sleep(1.0)

             if self.loop.is_running():
                 self.loop.call_soon_threadsafe(self.loop.stop)
                 time.sleep(0.1)
             self.loop.close()
             logger.info("VideoStreamer event loop closed in cleanup.")
        self.loop = None
        logger.info(f"VideoStreamer cleanup complete.") 