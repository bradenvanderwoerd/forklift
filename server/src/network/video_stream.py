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
                # print("Camera Matrix (mtx):\n", self.camera_matrix) # For debugging
                # print("Distortion Coefficients (dist):\n", self.dist_coeffs) # For debugging
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
            
            # Configure the camera
            config = self.camera.create_video_configuration(
                main={"size": (640, 480), "format": "RGB888"},
                lores={"size": (640, 480), "format": "YUV420"}
            )
            self.camera.configure(config)
            
            # Start the camera
            self.camera.start()
            
            # Verify camera is working by attempting to capture a frame
            try:
                self.camera.capture_array()
                logger.info("Camera initialized successfully")
                logger.info(f"Camera configuration: {config}")
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
                if not self.camera:
                    logger.error("Camera not initialized")
                    await asyncio.sleep(1)
                    continue
                
                frame_color = self.camera.capture_array()
                if frame_color is None:
                    logger.error("Failed to capture frame")
                    await asyncio.sleep(0.1)
                    continue
                
                frame_color = cv2.rotate(frame_color, cv2.ROTATE_180)
                
                # --- ArUco Detection --- 
                gray_frame = cv2.cvtColor(frame_color, cv2.COLOR_RGB2GRAY)
                corners, ids, rejected = self.aruco_detector.detectMarkers(gray_frame)
                
                if ids is not None:
                    frame_display = aruco.drawDetectedMarkers(frame_color.copy(), corners, ids)
                    logger.debug(f"Detected ArUco IDs: {ids.flatten().tolist()}") 

                    # --- Pose Estimation and Visualization ---
                    if self.camera_matrix is not None and self.dist_coeffs is not None:
                        try:
                            # Estimate pose for each detected marker
                            # rvecs, tvecs are arrays of rotation and translation vectors for each marker
                            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                                corners, 
                                self.marker_actual_size_m, 
                                self.camera_matrix, 
                                self.dist_coeffs
                            )

                            # Draw axes for each marker
                            for i in range(len(ids)):
                                rvec = rvecs[i]
                                tvec = tvecs[i]
                                cv2.drawFrameAxes(frame_display, 
                                                  self.camera_matrix, 
                                                  self.dist_coeffs, 
                                                  rvec, tvec, 
                                                  self.marker_actual_size_m * 0.5) # Length of axis
                                
                                # You can log or use rvec/tvec here for navigation later
                                # logger.debug(f"Marker ID: {ids[i][0]}, tvec: {tvec.flatten()}, rvec: {rvec.flatten()}")

                        except cv2.error as e:
                            logger.error(f"OpenCV error during pose estimation or drawing: {e}")
                        except Exception as e:
                            logger.error(f"Unexpected error during pose estimation or drawing: {e}")
                    else:
                        if not hasattr(self, '_warned_missing_calib_for_pose'):
                            logger.warning("Camera calibration data not available. Skipping pose estimation and axis drawing.")
                            self._warned_missing_calib_for_pose = True # Warn only once
                else:
                    frame_display = frame_color
                # --- End ArUco Detection & Pose Estimation ---

                quality = self.quality_controller.get_quality()
                _, buffer = cv2.imencode('.jpg', frame_display, [cv2.IMWRITE_JPEG_QUALITY, quality])
                
                await websocket.send(buffer.tobytes())
                
                await asyncio.sleep(1/30)
                
        except ConnectionClosed:
            logger.info("Video client connection closed")
        except Exception as e:
            logger.error(f"Error in video stream handler: {e}", exc_info=True)
        finally:
            self.clients.remove(websocket)
            logger.info(f"Video client disconnected. Remaining clients: {len(self.clients)}")
            
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