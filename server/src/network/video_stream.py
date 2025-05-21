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
        self.external_frame: Optional[np.ndarray] = None
        self.external_frame_lock = threading.Lock()
        
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
        
    def set_external_frame(self, frame: np.ndarray):
        """Sets an external frame to be streamed, typically from the overhead camera."""
        with self.external_frame_lock:
            self.external_frame = frame.copy() # Store a copy
            logger.debug("VideoStreamer: External frame received and stored.")

    def _initialize_camera(self):
        """Initialize the camera"""
        try:
            # Initialize the camera using picamera2
            self.camera = Picamera2()
            
            # Configure the camera using dimensions from config
            camera_config = self.camera.create_video_configuration(
                main={"size": (config.VIDEO_WIDTH, config.VIDEO_HEIGHT), "format": "RGB888"},
                lores={"size": (config.VIDEO_WIDTH, config.VIDEO_HEIGHT), "format": "YUV420"} # Keep lores same for simplicity or adjust if needed
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
                current_frame_to_process = None
                is_external_frame = False

                # Check for an external frame first
                with self.external_frame_lock:
                    if self.external_frame is not None:
                        current_frame_to_process = self.external_frame
                        self.external_frame = None # Consume the frame
                        is_external_frame = True
                        logger.debug("VideoStreamer: Using external frame for this cycle.")
                
                if not is_external_frame:
                    # If no external frame, use PiCamera
                    if not self.camera:
                        logger.error("PiCamera not initialized and no external frame available.")
                        await asyncio.sleep(1)
                        continue
                    
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
                    # Should not happen if logic above is correct, but as a safeguard
                    logger.warning("No frame available to process for this cycle.")
                    await asyncio.sleep(0.1)
                    continue

                # --- ArUco Detection (Only for PiCamera frames, not external/overhead frames) ---
                frame_display = current_frame_to_process # Default to current frame
                if not is_external_frame: # Process ArUco only if it's a PiCamera frame
                    aruco_start_time = time.time()
                    gray_frame = cv2.cvtColor(current_frame_to_process, cv2.COLOR_RGB2GRAY) # Assuming RGB input from PiCam
                    corners, ids, rejected = self.aruco_detector.detectMarkers(gray_frame)
                    
                    if ids is not None:
                        # Create a mutable copy for drawing if markers are detected
                        frame_display = aruco.drawDetectedMarkers(current_frame_to_process.copy(), corners, ids)
                        # logger.debug(f"Detected ArUco IDs: {ids.flatten().tolist()}") # Can be verbose

                        if self.camera_matrix is not None and self.dist_coeffs is not None:
                            try:
                                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                                    corners, 
                                    self.marker_actual_size_m, 
                                    self.camera_matrix, 
                                    self.dist_coeffs
                                )
                                primary_target_visible_this_frame = False
                                for i in range(len(ids)):
                                    rvec = rvecs[i]
                                    tvec = tvecs[i]
                                    current_id = ids[i][0]
                                    marker_name = "Unknown"
                                    is_primary_target = False

                                    if current_id == config.MARKER_ID_WHITE_BOX:
                                        marker_name = "White Box"
                                        is_primary_target = True
                                    elif current_id == config.MARKER_ID_BLACK_BOX:
                                        marker_name = "Black Box"
                                        is_primary_target = True
                                    elif current_id == config.MARKER_ID_WHITE_BOX_DESTINATION:
                                        marker_name = "White Box Destination"
                                    elif current_id == config.MARKER_ID_BLACK_BOX_DESTINATION:
                                        marker_name = "Black Box Destination"
                                    elif config.MARKER_ID_NAVIGATION_AID != -1 and current_id == config.MARKER_ID_NAVIGATION_AID:
                                        marker_name = "Navigation Aid"

                                    if marker_name != "Unknown":
                                        log_details = f"ID: {current_id}, tvec: {tvec.flatten().round(3)}, rvec: {rvec.flatten().round(3)}"
                                        if is_primary_target:
                                            primary_target_visible_this_frame = True
                                            if self.last_logged_primary_target_id != current_id:
                                                logger.info(f"Primary Target Acquired: {marker_name} ({log_details})")
                                                self.last_logged_primary_target_id = current_id
                                            # Always update pose if primary target is visible
                                            self.shared_primary_target_pose = (tvec, rvec)
                                            # Log subsequent at DEBUG handled by primary_target_visible_this_frame check below
                                        else: 
                                            logger.info(f"Auxiliary Target Visible: {marker_name} ({log_details})")
                                    else:
                                        logger.debug(f"Detected non-target ArUco ID: {current_id}, tvec: {tvec.flatten().round(3)}, rvec: {rvec.flatten().round(3)}")
                                    
                                    cv2.drawFrameAxes(frame_display, 
                                                      self.camera_matrix, 
                                                      self.dist_coeffs, 
                                                      rvec, tvec, 
                                                      self.marker_actual_size_m * 0.5)

                                # After iterating all markers, if a primary target was previously logged but not seen now
                                if self.last_logged_primary_target_id is not None and not primary_target_visible_this_frame:
                                    logger.info(f"Primary Target (ID: {self.last_logged_primary_target_id}) Lost.")
                                    self.last_logged_primary_target_id = None 
                                    self.shared_primary_target_pose = None # Clear shared pose
                                elif primary_target_visible_this_frame and self.shared_primary_target_pose is not None:
                                     # Primary target is still visible, log tvec/rvec at debug if it's the same one
                                     # This assumes self.shared_primary_target_pose was updated within the loop for the current primary
                                     tvec_s, _ = self.shared_primary_target_pose
                                     logger.debug(f"Primary Target Still Visible (ID: {self.last_logged_primary_target_id}), tvec: {tvec_s.flatten().round(3)}")

                            except cv2.error as e:
                                logger.error(f"OpenCV error during pose estimation or drawing: {e}")
                            except Exception as e:
                                logger.error(f"Unexpected error during pose estimation or drawing: {e}")
                        else:
                            if not hasattr(self, '_warned_missing_calib_for_pose'):
                                logger.warning("Camera calibration data not available. Skipping pose estimation and axis drawing.")
                                self._warned_missing_calib_for_pose = True 
                        aruco_processing_time = (time.time() - aruco_start_time) * 1000
                        logger.debug(f"ArUco processing time: {aruco_processing_time:.2f} ms")
                    # If no IDs, and it was a PiCamera frame, frame_display is already current_frame_to_process
                    elif not is_external_frame and self.last_logged_primary_target_id is not None:
                        # This case: PiCamera frame, no ArUco IDs detected THIS frame,
                        # but a primary target WAS being tracked.
                        logger.info(f"Primary Target (ID: {self.last_logged_primary_target_id}) Lost (no markers detected this frame).")
                        self.last_logged_primary_target_id = None
                        self.shared_primary_target_pose = None # Clear shared pose

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
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Set a timeout for the socket if needed, or handle blocking appropriately

        # This loop would be similar to _handle_client but for UDP clients
        # It needs a way to know client addresses, perhaps from an initial handshake
        # or a fixed list.
        # For now, this is a placeholder for where UDP sending logic would integrate.
        # If your Mac client connects via WebSocket, this UDP part might not be what you're using for video.

        # Example structure:
        # while self.running:
        #     current_frame_to_process = None
        #     is_external_frame = False
        #     with self.external_frame_lock:
        #         if self.external_frame is not None:
        #             current_frame_to_process = self.external_frame
        #             self.external_frame = None
        #             is_external_frame = True
        #     
        #     if not is_external_frame and self.camera:
        #         try:
        #             frame_color_pi = self.camera.capture_array()
        #             # ... processing ...
        #             current_frame_to_process = cv2.rotate(frame_color_pi, cv2.ROTATE_180)
        #         except Exception as e:
        #             logger.error(f"UDP: Error capturing PiCamera frame: {e}")
        #             time.sleep(0.1)
        #             continue
        # 
        #     if current_frame_to_process is not None:
        #          # ... ArUco for PiCam frames only ...
        #          # ... Encoding ...
        #          # udp_socket.sendto(encoded_jpeg.tobytes(), (client_host, client_port))
        #     time.sleep(0.01) # Loop delay
        
        udp_socket.close()
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