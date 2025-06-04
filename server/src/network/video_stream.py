import cv2
import numpy as np
import asyncio
import logging
import threading
from typing import Optional, Set, Any, List
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
from ..utils import config
import socket

logger = logging.getLogger(__name__)

class AdaptiveQualityController:
    """Dynamically adjusts JPEG encoding quality to manage performance.

    If frame processing/sending time is too high, quality is reduced.
    If it's low, quality is increased, up to a maximum.
    """
    def __init__(self, initial_quality: int = 80):
        """Initializes the adaptive quality controller.

        Args:
            initial_quality: The starting JPEG quality (0-100).
        """
        self.quality = initial_quality
        self.min_quality = 20  # Minimum allowed JPEG quality.
        self.max_quality = 95  # Maximum allowed JPEG quality.
        self.step_size = 5     # How much to change quality by at each adjustment.
        self.last_adjustment_time = time.time()
        self.adjustment_interval_seconds = 1.0  # Adjust quality at most once per second.
    
    def adjust_quality(self, frame_processing_time: float, target_frame_time: float = 1.0/config.VIDEO_FPS):
        """Adjusts JPEG quality based on the last frame's processing time.

        Args:
            frame_processing_time: Time taken to process and send the last frame (seconds).
            target_frame_time: The desired time per frame (e.g., 1/30 for 30 FPS).
        """
        current_time = time.time()
        if current_time - self.last_adjustment_time < self.adjustment_interval_seconds:
            return # Don't adjust too frequently.
        
        if frame_processing_time > target_frame_time * 1.15:  # Frame took >15% longer than target
            self.quality = max(self.min_quality, self.quality - self.step_size)
            logger.debug(f"Quality decreased to {self.quality} due to slow frame time: {frame_processing_time:.3f}s")
        elif frame_processing_time < target_frame_time * 0.85: # Frame took <85% of target (fast)
            self.quality = min(self.max_quality, self.quality + self.step_size)
            logger.debug(f"Quality increased to {self.quality} due to fast frame time: {frame_processing_time:.3f}s")
        
        self.last_adjustment_time = current_time
    
    def get_quality(self) -> int:
        """Returns the current JPEG quality setting."""
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
    """Manages the onboard Raspberry Pi camera (PiCamera2), performs ArUco marker
    detection on its frames, and streams the processed video via WebSockets.

    It initializes the camera for optimal field of view, loads camera calibration
    data for potential pose estimation, detects ArUco markers, draws them on
    the video frames, and sends JPEG encoded frames to connected WebSocket clients.
    Includes adaptive quality control for the JPEG stream.
    """
    def __init__(self, host: str = "0.0.0.0", port: int = config.SERVER_VIDEO_UDP_PORT):
        """Initializes the VideoStreamer.

        Args:
            host: The network host to bind the WebSocket server to (default "0.0.0.0").
            port: The port for the WebSocket video streaming server.
                  Defaults to `config.SERVER_VIDEO_UDP_PORT` (Note: name implies UDP but uses WebSocket/TCP).
        """
        self.host = host
        self.port = port
        self.server_task: Optional[asyncio.Task[None]] = None # Task for the WebSocket server
        self.clients: Set[Any] = set() # Set of connected WebSocket clients.
        self._running_capture_loop = False # Controls the main frame capture and processing loop.
        self._stop_event: Optional[asyncio.Event] = None # Event for signaling server task to stop.
        self.camera: Optional[Picamera2] = None
        self.capture_thread: Optional[threading.Thread] = None
        self.async_loop: Optional[asyncio.AbstractEventLoop] = None

        self.quality_controller = AdaptiveQualityController()
        self.last_logged_primary_target_id: Optional[int] = None
        # This attribute is intended to share the latest primary ArUco target's pose (if calculated)
        # with other parts of the server (e.g., ForkliftServer main loop). Currently, full 3D pose
        # calculation might not be implemented here beyond drawing markers.
        self.shared_primary_target_pose: Optional[Any] = None 
        
        # Initialize ArUco detector using settings from config.py
        self.aruco_dict = aruco.getPredefinedDictionary(config.ARUCO_DICTIONARY)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        logger.info(f"VideoStreamer: ArUco detector initialized with dictionary ID: {config.ARUCO_DICTIONARY}.")

        # Load onboard camera calibration data (for PiCamera)
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        calibration_file_path = config.CAMERA_CALIBRATION_FILE_PATH
        
        try:
            if os.path.exists(calibration_file_path):
                with np.load(calibration_file_path) as data:
                    self.camera_matrix = data['mtx']
                    self.dist_coeffs = data['dist']
                    logger.info(f"VideoStreamer: Successfully loaded onboard camera calibration from {calibration_file_path}")
                    # logger.debug(f"Camera Matrix (mtx):\n{self.camera_matrix}") 
                    # logger.debug(f"Distortion Coefficients (dist):\n{self.dist_coeffs}")
            else:
                logger.warning(f"VideoStreamer: Onboard camera calibration file not found at {calibration_file_path}. ArUco Pose estimation will be impaired.")
        except Exception as e:
            logger.error(f"VideoStreamer: Error loading onboard camera calibration data from {calibration_file_path}: {e}")

        self.marker_actual_size_m = config.ARUCO_MARKER_SIZE_METERS # Used for pose estimation if performed.
        
    def _initialize_camera(self) -> bool:
        """Initializes the PiCamera2 instance.

        Configures the camera for video streaming with specific settings for resolution,
        format, and controls (like ScalerCrop) to achieve the widest possible field of view.
        It prioritizes using the full raw sensor area and then scaling to the desired output size.

        Returns:
            True if camera initialization was successful, False otherwise.
        """
        try:
            self.camera = Picamera2()

            # Log available sensor modes for debugging/information.
            # The actual configuration relies on explicit raw stream sizing and ScalerCrop.
            try:
                modes = self.camera.sensor_modes
                if modes:
                    logger.debug(f"VideoStreamer: Available sensor modes ({len(modes)} total):")
                    for i, mode in enumerate(modes):
                        logger.debug(f"  Sensor Mode {i}: {mode}")
                else:
                    logger.debug("VideoStreamer: No sensor modes reported by camera.")
            except Exception as e:
                logger.error(f"VideoStreamer: Error inspecting sensor modes: {e}")
            
            # Define desired raw stream size (e.g., full sensor for IMX219/Pi Camera V2).
            # This provides the widest field of view, which is then scaled by ScalerCrop.
            raw_stream_w, raw_stream_h = 3280, 2464 # IMX219 common max resolution

            logger.info(f"VideoStreamer: Requesting raw stream at {raw_stream_w}x{raw_stream_h} for maximum FoV.")
            logger.info(f"VideoStreamer: ScalerCrop will be (0, 0, {raw_stream_w}, {raw_stream_h}).")
            logger.info(f"VideoStreamer: Main output stream configured to {config.VIDEO_WIDTH}x{config.VIDEO_HEIGHT} at {config.VIDEO_FPS} FPS.")

            # Create video configuration: main stream for output, lores for potential other uses (not currently used actively),
            # and a raw stream to capture the full sensor data.
            # ScalerCrop ensures the main stream uses the full FoV from the raw stream, scaled down.
            camera_config = self.camera.create_video_configuration(
                main={"size": (config.VIDEO_WIDTH, config.VIDEO_HEIGHT), "format": "RGB888"},
                lores={"size": (config.VIDEO_WIDTH, config.VIDEO_HEIGHT), "format": "YUV420"}, # Lo-res stream (optional)
                raw={"size": (raw_stream_w, raw_stream_h)}, # Request full sensor raw stream
                controls={
                    "ScalerCrop": (0, 0, raw_stream_w, raw_stream_h), # Use full sensor area
                    "FrameDurationLimits": (int(1_000_000 / config.VIDEO_FPS), int(1_000_000 / config.VIDEO_FPS))
                }
            )
            self.camera.configure(camera_config)
            
            self.camera.start()
            logger.info("VideoStreamer: PiCamera2 started.")
            # logger.debug(f"VideoStreamer: Actual camera configuration: {self.camera.camera_configuration()}")
            return True
                
        except Exception as e:
            logger.error(f"VideoStreamer: Fatal error initializing PiCamera2: {e}")
            if self.camera:
                try:
                    self.camera.stop()
                except Exception as e_stop:
                    logger.error(f"VideoStreamer: Error stopping camera during cleanup: {e_stop}")
                self.camera = None
            return False
            
    async def _handle_client_websocket(self, websocket: Any, path: str):
        """Handles an individual WebSocket client connection for video streaming.

        This method is run for each connected client. It enters a loop, capturing
        frames from the PiCamera, processing them for ArUco markers, encoding
        them as JPEG, and sending them to the client.

        Args:
            websocket: The WebSocket connection object for this client.
            path: The path of the WebSocket connection (unused).
        """
        self.clients.add(websocket)
        logger.info(f"VideoStreamer: New video client connected from {websocket.remote_address}. Total clients: {len(self.clients)}")
        
        try:
            while self._running_capture_loop: # Loop tied to the main capture loop flag
                if not self.camera or not self._running_capture_loop:
                    await asyncio.sleep(0.1) # Wait if camera not ready or stopping
                    continue
                
                frame_processing_start_time = time.perf_counter()
                frame_to_send = None

                try:
                    # Capture frame from PiCamera.
                    # capture_array() returns an RGB numpy array.
                    frame_rgb = self.camera.capture_array()
                    if frame_rgb is None:
                        logger.warning("VideoStreamer: capture_array() returned None. Skipping frame.")
                        await asyncio.sleep(0.01) # Brief pause before retrying
                        continue
                    
                    # Rotate frame if necessary (PiCamera often mounted upside down).
                    processed_frame = cv2.rotate(frame_rgb, cv2.ROTATE_180)
                except Exception as e:
                    logger.error(f"VideoStreamer: Error capturing or rotating PiCamera frame: {e}")
                    await asyncio.sleep(0.1) # Wait a bit before trying again
                    continue # Skip this frame processing cycle

                # --- ArUco Marker Detection and Drawing ---
                # Process the frame for ArUco markers.
                gray_frame = cv2.cvtColor(processed_frame, cv2.COLOR_RGB2GRAY)
                corners, ids, rejected_img_points = self.aruco_detector.detectMarkers(gray_frame)
                
                frame_display = processed_frame.copy() # Work on a copy for drawing

                if ids is not None and len(ids) > 0:
                    aruco.drawDetectedMarkers(frame_display, corners, ids)
                    
                    # Optional: Pose Estimation (if camera_matrix and dist_coeffs are loaded)
                    # This part currently only logs and updates shared_primary_target_pose.
                    # Full 3D pose might not be directly used by client but can be useful for server-side logic.
                    if self.camera_matrix is not None and self.dist_coeffs is not None:
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(
                            corners, self.marker_actual_size_m, 
                            self.camera_matrix, self.dist_coeffs
                        )
                        # Example: find and log primary target (e.g., first detected or specific ID)
                        # This is a placeholder for more sophisticated target selection if needed.
                        primary_id_this_frame = ids[0][0]
                        if primary_id_this_frame != self.last_logged_primary_target_id:
                            logger.info(f"VideoStreamer: Primary ArUco Target (ID: {primary_id_this_frame}) DETECTED.")
                            self.last_logged_primary_target_id = primary_id_this_frame
                        # For simplicity, share the pose of the first detected marker.
                        self.shared_primary_target_pose = (rvecs[0], tvecs[0]) 
                        # aruco.drawAxis(frame_display, self.camera_matrix, self.dist_coeffs, rvecs[0], tvecs[0], 0.1) # Draw axis
                
                elif self.last_logged_primary_target_id is not None:
                    logger.info(f"VideoStreamer: Primary ArUco Target (ID: {self.last_logged_primary_target_id}) LOST (no markers this frame).")
                    self.last_logged_primary_target_id = None
                    self.shared_primary_target_pose = None

                # --- JPEG Encoding and Sending --- 
                encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality_controller.get_quality()]
                result, encoded_jpeg = cv2.imencode('.jpg', frame_display, encode_params)
                
                if not result:
                    logger.error("VideoStreamer: Failed to encode frame to JPEG.")
                    await asyncio.sleep(0.01)
                    continue
                
                frame_to_send = encoded_jpeg.tobytes()
                
                if frame_to_send:
                    try:
                        await websocket.send(frame_to_send)
                    except ConnectionClosed:
                        logger.info(f"VideoStreamer: Client {websocket.remote_address} disconnected (send failed).")
                        break # Exit loop for this client
                    except Exception as e:
                        logger.error(f"VideoStreamer: Error sending frame to client {websocket.remote_address}: {e}")
                        break # Assume connection is problematic

                # Adaptive quality adjustment based on processing time for this frame
                frame_processing_duration = time.perf_counter() - frame_processing_start_time
                self.quality_controller.adjust_quality(frame_processing_duration)
                
                # Control frame rate - simple sleep if processing is faster than target frame time.
                # A more sophisticated approach might use a precise timer or integrate with asyncio loop scheduling.
                target_delay = 1.0 / config.VIDEO_FPS
                if frame_processing_duration < target_delay:
                    await asyncio.sleep(target_delay - frame_processing_duration)
                else:
                    await asyncio.sleep(0.001) # Minimal sleep if already over budget

        except ConnectionClosed:
            logger.info(f"VideoStreamer: Client {websocket.remote_address} connection closed gracefully.")
        except Exception as e:
            logger.error(f"VideoStreamer: Unhandled error in WebSocket handler for {websocket.remote_address}: {e}", exc_info=True)
        finally:
            self.clients.discard(websocket)
            logger.info(f"VideoStreamer: Client {websocket.remote_address} removed. Total clients: {len(self.clients)}")

    async def _main_capture_and_broadcast_loop(self):
        """This was intended as the main capture loop, but client handling is per-websocket.
        Keeping structure for potential refactor if direct broadcast to multiple clients from one capture is needed.
        Currently, each _handle_client_websocket does its own capture and send.
        This method is NOT CURRENTLY CALLED or fully implemented for shared capture.
        """
        logger.warning("_main_capture_and_broadcast_loop is not actively used in current design.")
        # This loop would be more complex, involving capturing one frame and then iterating
        # through self.clients to send it. Current design has capture per client connection.
        # For now, it does nothing to avoid interference.
        while self._running_capture_loop:
            await asyncio.sleep(1) # Placeholder

    async def _shutdown_server_resources(self):
        """Gracefully shuts down the WebSocket server and associated resources."""
        logger.info("VideoStreamer: Shutting down server resources...")
        if self.server_task and not self.server_task.done():
            # Close listening server first
            if hasattr(self.server_task, 'ws_server') and self.server_task.ws_server: # type: ignore
                self.server_task.ws_server.close() # type: ignore
                try:
                    await asyncio.wait_for(self.server_task.ws_server.wait_closed(), timeout=5.0) # type: ignore
                    logger.info("VideoStreamer: WebSocket server closed.")
                except asyncio.TimeoutError:
                    logger.warning("VideoStreamer: Timeout waiting for WebSocket server to close. Forcing task cancellation.")
                    self.server_task.cancel()
                except Exception as e:
                    logger.error(f"VideoStreamer: Error during WebSocket server wait_closed: {e}")
                    self.server_task.cancel()
            else:
                 logger.info("VideoStreamer: server_task does not have ws_server attribute or it's None. Cancelling task.")
                 self.server_task.cancel()
            
            try:
                await asyncio.wait_for(self.server_task, timeout=5.0) # Wait for the server task to finish
                logger.info("VideoStreamer: Server task finished.")
            except asyncio.TimeoutError:
                logger.warning("VideoStreamer: Timeout waiting for server task to finish. It might be stuck.")
            except asyncio.CancelledError:
                logger.info("VideoStreamer: Server task was cancelled.")
            except Exception as e:
                logger.error(f"VideoStreamer: Error waiting for server task: {e}")
        
        # Close all client connections
        logger.info(f"VideoStreamer: Closing {len(self.clients)} client connections...")
        clients_to_close = list(self.clients)
        for client_ws in clients_to_close:
            try:
                await client_ws.close(code=1001, reason="Server shutting down")
                logger.debug(f"VideoStreamer: Closed client connection {client_ws.remote_address}")
            except Exception as e:
                logger.warning(f"VideoStreamer: Error closing client {client_ws.remote_address}: {e}")
        self.clients.clear()
        logger.info("VideoStreamer: All client connections processed for closure.")

    async def start_server_async(self):
        """Starts the WebSocket video streaming server asynchronously."""
        if self._running_capture_loop:
            logger.warning("VideoStreamer: Server already running or start initiated.")
            return

        if not self._initialize_camera():
            logger.error("VideoStreamer: Camera initialization failed. Cannot start video stream server.")
            return

        self._running_capture_loop = True
        self._stop_event = asyncio.Event() # Event used to signal the server task to stop gracefully
        
        logger.info(f"VideoStreamer: Starting WebSocket server on {self.host}:{self.port}")
        try:
            # Pass self._handle_client_websocket which will be invoked for each new connection
            ws_server_instance = await ws_serve(
                self._handle_client_websocket, 
                self.host, 
                self.port, 
                ping_interval=20, 
                ping_timeout=20,
                max_size=None, # Allow large frames
                reuse_address=True # Add SO_REUSEADDR option
            )
            logger.info("VideoStreamer: WebSocket server is listening.")
            # Store the server object to allow for graceful shutdown
            if hasattr(ws_server_instance, 'close'): # ws_serve returns a Server object
                self.server_task.ws_server = ws_server_instance # type: ignore # Store the actual server for closing
            else:
                logger.error("VideoStreamer: ws_serve did not return a recognizable server object with close().")

            await self._stop_event.wait() # Keep the server running until stop_event is set
            logger.info("VideoStreamer: Stop event received, proceeding to shutdown server resources.")

        except OSError as e:
            logger.error(f"VideoStreamer: Failed to start WebSocket server on {self.host}:{self.port}. Address likely in use. Error: {e}")
            self._running_capture_loop = False # Ensure flag is reset
            if self.camera:
                self.camera.stop()
                self.camera = None
        except Exception as e:
            logger.error(f"VideoStreamer: Unexpected error in start_server_async: {e}", exc_info=True)
            self._running_capture_loop = False # Ensure flag is reset
            if self.camera:
                self.camera.stop()
                self.camera = None
        finally:
            logger.info("VideoStreamer: start_server_async finally block. Server task should be ending.")
            # Ensure cleanup of resources happens if loop exits unexpectedly
            # Actual client connection closing and camera stop should be handled by stop() or cleanup()

    def start(self):
        """Starts the video streamer in a new thread with its own asyncio event loop.
        Initializes the camera and starts the WebSocket server.
        """
        if self.capture_thread and self.capture_thread.is_alive():
            logger.warning("VideoStreamer: Start called but capture thread is already running.")
            return

        logger.info("VideoStreamer: Starting video stream thread and asyncio event loop...")
        # Create a new event loop for this thread
        self.async_loop = asyncio.new_event_loop()
        
        def run_loop():
            asyncio.set_event_loop(self.async_loop)
            try:
                # Create a task for start_server_async within this new loop
                self.server_task = self.async_loop.create_task(self.start_server_async())
                self.async_loop.run_until_complete(self.server_task)
            except KeyboardInterrupt:
                logger.info("VideoStreamer: KeyboardInterrupt in run_loop.")
            except Exception as e:
                logger.error(f"VideoStreamer: Exception in run_loop: {e}", exc_info=True)
            finally:
                logger.info("VideoStreamer: Asyncio loop in capture thread ended.")
                # Ensure any pending tasks are cancelled and loop is closed
                if self.server_task and not self.server_task.done():
                    self.server_task.cancel()
                    try:
                        self.async_loop.run_until_complete(self.server_task)
                    except asyncio.CancelledError:
                        pass # Expected
                self.async_loop.close()
        
        self.capture_thread = threading.Thread(target=run_loop, daemon=True)
        self.capture_thread.start()
        logger.info("VideoStreamer: Video stream thread started.")

    def stop(self):
        """Signals the video streamer to stop its operations gracefully."""
        logger.info("VideoStreamer: Stop requested.")
        if not self._running_capture_loop and not (self.capture_thread and self.capture_thread.is_alive()):
            logger.info("VideoStreamer: Stop called but streamer not considered active.")
            # Call cleanup just in case some resources were partially initialized
            self.cleanup() 
            return

        self._running_capture_loop = False # Signal all loops to stop

        if self.async_loop and self._stop_event and not self._stop_event.is_set():
            logger.info("VideoStreamer: Setting stop_event for async_loop.")
            # Schedule setting the event in the streamer's own loop
            self.async_loop.call_soon_threadsafe(self._stop_event.set)
        
        # Wait for the capture thread to finish
        if self.capture_thread and self.capture_thread.is_alive():
            logger.info("VideoStreamer: Waiting for capture thread to join...")
            self.capture_thread.join(timeout=10.0) # Increased timeout
            if self.capture_thread.is_alive():
                logger.warning("VideoStreamer: Capture thread did not join in time. It might be stuck.")
            else:
                logger.info("VideoStreamer: Capture thread joined.")
        self.capture_thread = None

        # Final resource cleanup (camera, etc.)
        self.cleanup() 
        logger.info("VideoStreamer: Stop sequence complete.")

    def cleanup(self):
        """Releases camera resources and cleans up any other held resources.
        This method is called by stop() and can also be called directly if needed.
        """
        logger.info("VideoStreamer: Cleanup initiated.")
        # Ensure capture loop flag is false
        self._running_capture_loop = False

        if self.camera:
            logger.info("VideoStreamer: Stopping and closing PiCamera2...")
            try:
                self.camera.stop() # Stop the camera stream
                # self.camera.close() # close() is not available in all Picamera2 versions / contexts
                                   # or might be handled by stop() or GC.
                logger.info("VideoStreamer: PiCamera2 stopped.")
            except Exception as e:
                logger.error(f"VideoStreamer: Error stopping/closing PiCamera2: {e}", exc_info=True)
            finally:
                self.camera = None # Ensure camera object is released
        
        # Further cleanup of clients or server task might be redundant if stop() handled it,
        # but can be added here for robustness if cleanup is called independently.
        if self._stop_event and not self._stop_event.is_set() and self.async_loop and self.async_loop.is_running():
            logger.debug("VideoStreamer cleanup: stop_event was not set, setting it now.")
            self.async_loop.call_soon_threadsafe(self._stop_event.set)
        
        logger.info("VideoStreamer: Cleanup finished.")

# Example usage (for testing this module directly)
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    logger.info("Starting VideoStreamer test...")
    
    # Ensure pigpiod is running: sudo pigpiod
    
    streamer = VideoStreamer(host='0.0.0.0', port=3457)
    
    def signal_handler(sig, frame):
        logger.info(f"Signal {sig} received, stopping streamer...")
        streamer.stop()
        # Allow some time for threads to clean up before exiting main thread
        # This might not be strictly necessary if join() in stop() is effective
        time.sleep(2) 
        logger.info("Exiting VideoStreamer test.")
        sys.exit(0) # Use sys.exit for cleaner exit from signal handler

    import sys # For sys.exit
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        streamer.start()
        # Keep the main thread alive while the streamer runs in its daemon thread
        while streamer.capture_thread and streamer.capture_thread.is_alive():
            time.sleep(0.5)
    except Exception as e:
        logger.error(f"Unhandled exception in main: {e}", exc_info=True)
    finally:
        logger.info("Main thread finally block. Ensuring streamer is stopped.")
        if streamer: # Check if streamer was successfully initialized
            streamer.stop() # Ensure stop is called on exit
    logger.info("VideoStreamer test finished.") 