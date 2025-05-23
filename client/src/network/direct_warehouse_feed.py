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

        # JPEG Start of Image (SOI) and End of Image (EOI) markers
        self._jpeg_soi = b'\xff\xd8'
        self._jpeg_eoi = b'\xff\xd9'
        
    def connect(self):
        if self.is_connected and self.thread and self.thread.is_alive():
            logger.warning("DirectWarehouseFeedReceiver: Already connected and thread is running.")
            return
            
        self._stop_event.clear()
        # Ensure thread name is unique if multiple instances or similar threads run
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
            sock.settimeout(3.0) # Socket operations timeout
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
                
                current_time = time.time()
                
                if current_time - last_video_request_time >= video_request_interval_seconds:
                    command_to_send_g1 = b"G 1\n" # Ensure newline, some servers might need it
                    
                    try:
                        self.socket.sendall(command_to_send_g1)
                    except Exception as e:
                        logger.error(f"DirectWarehouseFeedReceiver: Error sending G 1 command: {e}")
                        self._handle_connection_error()
                        continue

                    last_video_request_time = current_time # Update time after successful send
                    
                    # Read JPEG stream
                    frame_buffer = bytearray()
                    found_soi = False
                    
                    # Timeout for receiving a single frame
                    frame_receive_start_time = time.time()
                    # Socket timeout is 3s, allow slightly more for full frame.
                    FRAME_TIMEOUT_SECONDS = self.socket.gettimeout() * 1.5 

                    while time.time() - frame_receive_start_time < FRAME_TIMEOUT_SECONDS:
                        try:
                            chunk = self.socket.recv(4096) # Read in chunks
                            if not chunk:
                                logger.warning("DirectWarehouseFeedReceiver: Connection closed by server while reading frame.")
                                self._handle_connection_error()
                                break 
                            
                            frame_buffer.extend(chunk)

                            if not found_soi:
                                soi_index = frame_buffer.find(self._jpeg_soi)
                                if soi_index != -1:
                                    frame_buffer = frame_buffer[soi_index:] # Start of a new frame
                                    found_soi = True
                            
                            if found_soi:
                                eoi_index = frame_buffer.find(self._jpeg_eoi)
                                if eoi_index != -1:
                                    jpeg_data = frame_buffer[:eoi_index + len(self._jpeg_eoi)]
                                    frame_buffer = frame_buffer[eoi_index + len(self._jpeg_eoi):] # Keep remainder for next frame
                                    
                                    try:
                                        frame = cv2.imdecode(np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                                        if frame is not None:
                                            if not self.video_queue.full():
                                                self.video_queue.put_nowait(frame)
                                            else:
                                                self.video_queue.get_nowait() # Make space
                                                self.video_queue.put_nowait(frame)
                                        else:
                                            logger.warning("DirectWarehouseFeedReceiver: Failed to decode JPEG frame.")
                                    except Exception as e_decode:
                                        logger.error(f"DirectWarehouseFeedReceiver: Error decoding frame: {e_decode}")
                                    found_soi = False # Look for next SOI
                                    break # Successfully processed one frame

                        except socket.timeout:
                            logger.debug("DirectWarehouseFeedReceiver: Socket timeout while reading frame chunk.")
                            break # Break from chunk reading loop, will try G1 again or reconnect
                        except Exception as e_recv:
                            logger.error(f"DirectWarehouseFeedReceiver: Error receiving frame chunk: {e_recv}")
                            self._handle_connection_error()
                            break 
                    else: # Outer while loop timed out without breaking from chunk loop
                        if not found_soi or not self._jpeg_eoi in frame_buffer:
                             logger.warning(f"DirectWarehouseFeedReceiver: Full frame timeout. Buffer size: {len(frame_buffer)}")


            except Exception as e_outer:
                logger.error(f"DirectWarehouseFeedReceiver: Unhandled exception in _run_client: {e_outer}", exc_info=True)
                self._handle_connection_error() # Generic error handling

        self._cleanup_socket()
        self.is_connected = False
        logger.info(f"DirectWarehouseFeedReceiver: Thread {threading.current_thread().name} finished.")

    def _handle_connection_error(self, is_connect_phase: bool = False):
        logger.warning(f"DirectWarehouseFeedReceiver: Connection error occurred.")
        self._cleanup_socket()
        self.is_connected = False
        
        if self._stop_event.is_set(): # If stopping, don't attempt to reconnect
            return

        logger.info(f"DirectWarehouseFeedReceiver: Will attempt to reconnect in {self._reconnect_delay:.1f}s...")
        time.sleep(self._reconnect_delay)
        self._reconnect_delay = min(self._reconnect_delay * 2, self._max_reconnect_delay)

    def _cleanup_socket(self):
        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass # Ignore errors on shutdown
            try:
                self.socket.close()
            except Exception:
                pass # Ignore errors on close
            self.socket = None
        logger.debug("DirectWarehouseFeedReceiver: Socket cleaned up.") 