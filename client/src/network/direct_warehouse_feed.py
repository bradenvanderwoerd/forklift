import socket
import logging
import time
import queue
import threading
import cv2
import numpy as np
from typing import Optional

# Get a logger for the current module, but make its name distinct
# to avoid confusion with the server-side WarehouseCameraClient logs if running locally.
logger = logging.getLogger(f"client.{__name__}")

class DirectWarehouseFeedReceiver:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.socket: Optional[socket.socket] = None
        self.is_connected = False
        self.video_queue = queue.Queue(maxsize=5)  # Queue for video frames
        self.thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._reconnect_delay = 1.0  # Start with 1 second delay
        self._max_reconnect_delay = 30.0  # Maximum delay
        self.consecutive_failures = 0 # Added for exponential backoff

        # JPEG Start of Image (SOI) and End of Image (EOI) markers
        self._jpeg_soi = b'\xff\xd8'
        self._jpeg_eoi = b'\xff\xd9'
        
    def connect(self):
        if self.is_connected and self.thread and self.thread.is_alive():
            logger.warning("DirectWarehouseFeedReceiver: Already connected.")
            return
        self._stop_event.clear()
        self.consecutive_failures = 0 # Reset on new connect attempt
        self.thread = threading.Thread(target=self._run_client, name="DirectWarehouseFeedThread")
        self.thread.daemon = True # Important for clean exit
        self.thread.start()
        logger.info(f"DirectWarehouseFeedReceiver: Connect called, thread {self.thread.name} started.")
        
    def disconnect(self):
        if not self.thread and not self.is_connected:
            logger.info("DirectWarehouseFeedReceiver: Already disconnected or not started.")
            return
            
        logger.info("DirectWarehouseFeedReceiver: Disconnecting...")
        self._stop_event.set()
        
        if self.thread:
            self.thread.join(timeout=3.0) # Shorter timeout as it's just a feed
            if self.thread.is_alive():
                logger.warning("DirectWarehouseFeedReceiver: Thread did not terminate cleanly.")
        
        self._cleanup_socket()
        self.is_connected = False
        self.thread = None # Clear the thread reference
        logger.info("DirectWarehouseFeedReceiver: Disconnected.")
        
    def get_video_frame(self) -> Optional[np.ndarray]:
        try:
            return self.video_queue.get_nowait()
        except queue.Empty:
            return None
            
    def _create_socket(self) -> Optional[socket.socket]:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.settimeout(2.0) # Consistent with server's connected socket timeout
            return sock
        except Exception as e:
            logger.error(f"DirectWarehouseFeedReceiver: Error creating socket: {e}")
            return None
            
    def _run_client(self):
        last_video_request_time = 0
        # Aim for roughly 30 FPS, but network conditions will dictate actual rate
        # Sending requests too fast might not be beneficial if the server can't keep up
        # or if it causes network congestion. Start with a slightly more conservative rate.
        video_request_interval_seconds = 1.0 / 20.0  # Target 20 FPS for requests

        while not self._stop_event.is_set():
            try:
                if self.socket is None or not self.is_connected:
                    self.socket = self._create_socket()
                    if self.socket is None:
                        self._handle_connection_error(is_connect_phase=True)
                        continue
                        
                    logger.info(f"DirectWarehouseFeedReceiver: Attempting to connect to {self.host}:{self.port}")
                    self.socket.connect((self.host, self.port))
                    self.is_connected = True
                    logger.info(f"DirectWarehouseFeedReceiver: Connected to {self.host}:{self.port}")
                    self._reconnect_delay = 1.0 # Reset reconnect delay on successful connection
                    self.consecutive_failures = 0 # Reset on successful connect
                
                current_time = time.time()
                
                if current_time - last_video_request_time >= video_request_interval_seconds:
                    command_to_send_g1 = b"G 1" # Removed newline
                    
                    try:
                        self.socket.sendall(command_to_send_g1)
                    except Exception as e:
                        logger.error(f"DirectWarehouseFeedReceiver: Error sending G 1: {e}")
                        self._handle_connection_error()
                        continue

                    frame_data = bytearray()
                    image_receive_start_time = time.time()
                    RECV_BUFFER_SIZE = 8192
                    MAX_IMAGE_BYTES = 5 * 1024 * 1024 
                    # Timeout for receiving a single frame
                    frame_receive_start_time = time.time()
                    # Socket timeout is 2s. Make frame timeout less than join() timeout (3s).
                    FRAME_TIMEOUT_SECONDS = self.socket.gettimeout() * 1.2 # e.g., 2.0s * 1.2 = 2.4s

                    try:
                        while True: # Inner loop for reading one frame data
                            if time.time() - image_receive_start_time > FRAME_TIMEOUT_SECONDS:
                                logger.warning(f"DirectWarehouseFeedReceiver: Timeout receiving image. Received {len(frame_data)} bytes.")
                                self._handle_connection_error() 
                                raise socket.timeout("Image receive overall timeout")

                            bytes_to_read = min(RECV_BUFFER_SIZE, MAX_IMAGE_BYTES - len(frame_data) + RECV_BUFFER_SIZE // 2)
                            if bytes_to_read <= 0 and len(frame_data) >= MAX_IMAGE_BYTES:
                               logger.warning(f"DirectWarehouseFeedReceiver: Exceeded max image bytes ({MAX_IMAGE_BYTES}) before EOI.")
                               self._handle_connection_error()
                               raise ConnectionError("Max image byte limit exceeded before EOI")

                            chunk = self.socket.recv(bytes_to_read if bytes_to_read > 0 else RECV_BUFFER_SIZE)
                            if not chunk:
                                logger.warning("DirectWarehouseFeedReceiver: Connection closed by server (recv empty).")
                                self._handle_connection_error()
                                raise ConnectionError("Connection closed by server reading image")
                            
                            frame_data.extend(chunk)
                            if frame_data.endswith(self._jpeg_eoi):
                                break # Full JPEG frame received
                            if len(frame_data) > MAX_IMAGE_BYTES:
                                logger.warning(f"DirectWarehouseFeedReceiver: Exceeded max image bytes ({MAX_IMAGE_BYTES}) after recv, no EOI.")
                                self._handle_connection_error()
                                raise ConnectionError("Max image byte limit, no EOI")
                        
                        # EOI found, now decode
                        soi_index = frame_data.rfind(self._jpeg_soi) # Find LAST SOI
                        if soi_index != -1:
                            jpeg_data_to_decode = frame_data[soi_index:]
                            frame_np = cv2.imdecode(np.frombuffer(jpeg_data_to_decode, dtype=np.uint8), cv2.IMREAD_COLOR)
                            if frame_np is not None:
                                if not self.video_queue.full():
                                    self.video_queue.put_nowait(frame_np)
                                else:
                                    self.video_queue.get_nowait() # Make space
                                    self.video_queue.put_nowait(frame_np)
                            else:
                                logger.warning(f"DirectWarehouseFeedReceiver: cv2.imdecode failed for {len(jpeg_data_to_decode)} bytes (SOI found). Original: {len(frame_data)} bytes.")
                        else:
                            logger.warning(f"DirectWarehouseFeedReceiver: EOI found but no SOI in {len(frame_data)} bytes.")
                    except (socket.timeout, ConnectionError, ConnectionResetError) as e: 
                        logger.warning(f"DirectWarehouseFeedReceiver: Network error receiving image: {e}")
                        # _handle_connection_error() should be called by the logic that raised or in this path
                        continue 
                    except Exception as e_img:
                        logger.error(f"DirectWarehouseFeedReceiver: Unexpected error processing image: {e_img}", exc_info=True)
                        self._handle_connection_error()
                        continue
                    last_video_request_time = time.time()

            except (socket.timeout, ConnectionError, ConnectionResetError) as e_outer_net:
                logger.warning(f"DirectWarehouseFeedReceiver: Outer loop net error: {e_outer_net}")
                self._handle_connection_error()
            except Exception as e_outer_generic:
                logger.error(f"DirectWarehouseFeedReceiver: Outer loop unexpected error: {e_outer_generic}", exc_info=True)
                self._handle_connection_error() # Ensure robust handling

        self._cleanup_socket()
        self.is_connected = False # Ensure state is false on exit
        logger.info(f"DirectWarehouseFeedReceiver: Thread {threading.current_thread().name} finished.")

    def _handle_connection_error(self, is_connect_phase: bool = False):
        # Set state before potential sleep
        self.is_connected = False 
        if self.socket: # Ensure socket is closed before potentially sleeping
            self._cleanup_socket() # Call the method that handles actual closing
        
        self.consecutive_failures +=1
        if self._stop_event.is_set():
            logger.info("DirectWarehouseFeedReceiver: Stop event set, not reconnecting.")
            return

        # Exponential backoff for reconnect delay
        delay_factor_exponent = min(self.consecutive_failures, 6) # Cap exponent to avoid overly long delays
        current_delay = self._reconnect_delay * (1.5 ** delay_factor_exponent)
        delay = min(current_delay, self._max_reconnect_delay)
        
        log_message = f"Failed to connect. Retrying in {delay:.1f}s..." if is_connect_phase else f"Connection lost. Reconnecting in {delay:.1f}s..."
        logger.info(f"DirectWarehouseFeedReceiver: {log_message}")
        time.sleep(delay)

    def _cleanup_socket(self):
        if self.socket:
            logger.debug("DirectWarehouseFeedReceiver: Cleaning up socket...")
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except (OSError, socket.error):
                pass 
            try:
                self.socket.close()
            except (OSError, socket.error):
                pass
            self.socket = None
            logger.debug("DirectWarehouseFeedReceiver: Socket cleaned up.") 