import socket
import logging
import time
import re
import queue
import threading
import cv2
import numpy as np
from typing import Optional

logger = logging.getLogger(__name__)

class WarehouseCameraClient:
    def __init__(self, host: str = "192.168.0.100", port: int = 5001):
        self.host = host
        self.port = port
        self.socket: Optional[socket.socket] = None
        self.is_connected = False
        self.time_queue = queue.Queue(maxsize=1)  # Queue for warehouse time
        self.video_queue = queue.Queue(maxsize=10)  # Queue for video frames
        self.thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._reconnect_delay = 1.0  # Start with 1 second delay
        self._max_reconnect_delay = 30.0  # Maximum delay between reconnection attempts
        self.consecutive_failures = 0
        
    def connect(self):
        if self.is_connected and self.thread and self.thread.is_alive():
            logger.warning("Already connected and thread is running for warehouse camera")
            return
            
        self._stop_event.clear()
        self.thread = threading.Thread(target=self._run_client, name="WarehouseCameraThread")
        self.thread.daemon = True
        self.thread.start()
        
    def disconnect(self):
        if not self.thread and not self.is_connected:
            logger.info("Warehouse camera client already disconnected or not started.")
            return
            
        logger.info("Disconnecting from warehouse camera...")
        self._stop_event.set()
        
        if self.thread:
            self.thread.join(timeout=5.0)
            if self.thread.is_alive():
                logger.warning("Warehouse camera thread did not terminate cleanly")
        
        self._cleanup_socket()
        self.is_connected = False
        self.thread = None
        
    def get_warehouse_time(self) -> Optional[float]:
        try:
            return self.time_queue.get_nowait()
        except queue.Empty:
            return None
            
    def get_video_frame(self) -> Optional[np.ndarray]:
        try:
            return self.video_queue.get_nowait()
        except queue.Empty:
            return None
            
    def _parse_time_response(self, response: str) -> Optional[float]:
        match = re.search(r'<WAREHOUSE TIME="([\d.]+)" />', response)
        if match:
            try:
                return float(match.group(1))
            except ValueError:
                logger.warning(f"Invalid time value in response: {response}")
        return None
            
    def _create_socket(self) -> Optional[socket.socket]:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) # Optional
            sock.settimeout(5.0)
            return sock
        except Exception as e:
            logger.error(f"Error creating socket: {e}")
            return None
            
    def _run_client(self):
        last_time_request = 0
        time_request_interval = 1.0
        last_video_request = 0
        video_request_interval = 0.033 # Target ~30 FPS

        while not self._stop_event.is_set():
            try:
                if self.socket is None:
                    self.socket = self._create_socket()
                    if self.socket is None:
                        self._handle_connection_error(is_connect_phase=True)
                        continue
                        
                    logger.info(f"Attempting to connect to warehouse camera at {self.host}:{self.port}")
                    self.socket.connect((self.host, self.port))
                    self.is_connected = True
                    logger.info(f"Connected to warehouse camera at {self.host}:{self.port}")
                    
                    self._reconnect_delay = 1.0
                    self.consecutive_failures = 0
                    self.socket.settimeout(2.0)
                
                current_time = time.time()
                
                # Request time (G 0)
                if current_time - last_time_request >= time_request_interval:
                    command_to_send = b"G 0"
                    logger.debug(f"Attempting to send command: {command_to_send!r}")
                    try:
                        self.socket.sendall(command_to_send)
                        logger.debug(f"Successfully sent command: {command_to_send!r}")
                    except Exception as e:
                        logger.error(f"Error sending command {command_to_send!r}: {e}")
                        self._handle_connection_error()
                        continue # Skip to next iteration of the main loop
                    
                    response_bytes = bytearray()
                    read_start_time = time.time()
                    received_anything = False
                    
                    while time.time() - read_start_time < self.socket.gettimeout():
                        try:
                            char = self.socket.recv(1)
                            if not char:
                                logger.warning("Connection closed by server while reading time (recv(1) returned empty).")
                                raise ConnectionError("Connection closed by server while reading time")
                            
                            received_anything = True
                            logger.debug(f"Received byte: {char!r}")
                            response_bytes.extend(char)
                            if char == b'\n':
                                logger.debug("Newline received, breaking from recv loop.")
                                break
                        except socket.timeout:
                            if received_anything:
                                logger.debug(f"Socket timeout after receiving some data for G 0: {response_bytes!r}")
                            else:
                                logger.debug("Socket timeout while waiting for G 0 response char (no data received).")
                            break # Break from this inner while loop for recv
                        except Exception as e:
                            logger.error(f"Error during recv for G 0: {e}")
                            # Marking as connection error to trigger reconnect logic
                            self._handle_connection_error() 
                            # Need to break out of the recv loop, outer loop will handle reconnect
                            response_bytes = bytearray() # Clear potentially partial data
                            break 
                    
                    if response_bytes: # Process if we broke due to newline or error after some data
                        try:
                            time_str = response_bytes.decode('utf-8').strip()
                            logger.debug(f"Received time response: '{time_str}'")
                            warehouse_time = self._parse_time_response(time_str)
                            if warehouse_time is not None:
                                try:
                                    while not self.time_queue.empty():
                                        self.time_queue.get_nowait()
                                    self.time_queue.put_nowait(warehouse_time)
                                except queue.Full:
                                    pass
                            else:
                                logger.warning(f"Could not parse time from: '{time_str}'")
                        except UnicodeDecodeError:
                            logger.warning(f"Failed to decode time response: {response_bytes}")
                    else:
                        logger.debug("No complete time response received for G 0.")
                    last_time_request = current_time
                
                # Request video frame (G 1)
                if current_time - last_video_request >= video_request_interval:
                    command_to_send_g1 = b"G 1"

                    logger.debug(f"Attempting to send command: {command_to_send_g1!r}")
                    try:
                        self.socket.sendall(command_to_send_g1)
                        logger.debug(f"Successfully sent command: {command_to_send_g1!r}")
                    except Exception as e:
                        logger.error(f"Error sending command {command_to_send_g1!r}: {e}")
                        self._handle_connection_error()
                        continue

                    # 1. Read the image size (string ending with newline)
                    img_size_str_bytes = bytearray()
                    received_anything_size = False
                    read_size_start_time = time.time()
                    try:
                        while time.time() - read_size_start_time < self.socket.gettimeout():
                            char = self.socket.recv(1)
                            if not char:
                                logger.warning("Connection closed by server while reading image size (recv(1) returned empty).")
                                raise ConnectionError("Connection closed by server while reading image size")
                            received_anything_size = True
                            # logger.debug(f"Recv image size byte: {char!r}")
                            if char == b'\n':
                                break
                            img_size_str_bytes.extend(char)
                        else: # Timeout occurred
                            if received_anything_size:
                                logger.warning(f"Socket timeout after receiving partial image size: {img_size_str_bytes!r}")
                            else:
                                logger.warning("Socket timeout while waiting for image size (no data received).")
                            self._handle_connection_error() # Or just continue if timeout is expected
                            continue

                        img_size_str = img_size_str_bytes.decode('ascii').strip()
                        img_size = int(img_size_str)
                        logger.debug(f"Received image size: {img_size}")

                    except socket.timeout:
                        if received_anything_size:
                             logger.warning(f"Socket timeout after receiving partial image size: {img_size_str_bytes!r}")
                        else:
                            logger.warning(f"Socket timeout reading image size for G 1 (no data received).")
                        self._handle_connection_error() # Or just continue if timeout is expected
                        continue
                    except (ValueError, UnicodeDecodeError) as e:
                        logger.error(f"Error parsing image size '{img_size_str_bytes.decode(errors="ignore")}': {e}")
                        self._handle_connection_error()
                        continue
                    except ConnectionError as e: # From explicit raise
                        logger.error(f"Connection error while reading image size: {e}")
                        self._handle_connection_error()
                        continue
                    except Exception as e:
                        logger.error(f"Unexpected error reading image size: {e}")
                        self._handle_connection_error()
                        continue
                    
                    # 2. Read the image data
                    frame_data = bytearray()
                    bytes_received = 0
                    image_read_start_time = time.time() # Reset timer for image data read

                    try:
                        while bytes_received < img_size and time.time() - image_read_start_time < self.socket.gettimeout():
                            chunk_size = min(8192, img_size - bytes_received)
                            chunk = self.socket.recv(chunk_size)
                            if not chunk:
                                logger.warning("Connection closed by server while reading image data (recv returned empty).")
                                raise ConnectionError("Connection closed by server while reading image data")
                            frame_data.extend(chunk)
                            bytes_received += len(chunk)
                            # logger.debug(f"Received image chunk: {len(chunk)} bytes, total: {bytes_received}/{img_size}")

                        if bytes_received < img_size:
                            logger.warning(f"Incomplete image data received: {bytes_received}/{img_size} bytes before timeout or error.")
                            # Don't necessarily treat as a full connection error, maybe just a bad frame
                            # But for now, let's be strict to see if it works.
                            self._handle_connection_error()
                            continue
                        
                        logger.debug(f"Successfully received {bytes_received} bytes for image.")

                        # 3. Decode the image
                        # Assuming JPEG format based on typical arena server behavior
                        frame_np = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                        
                        if frame_np is not None:
                            # logger.debug(f"Successfully decoded frame: {frame_np.shape}")
                            try:
                                # Clear old frames if any to keep the queue fresh
                                while not self.video_queue.empty():
                                    self.video_queue.get_nowait()
                                self.video_queue.put_nowait(frame_np)
                                # logger.debug(f"Put frame in video_queue. Size: {self.video_queue.qsize()}")
                            except queue.Full:
                                logger.warning("Video queue full, dropping frame.")
                                pass # Frame dropped
                        else:
                            logger.warning(f"Failed to decode image from {len(frame_data)} bytes of data.")
                            # Consider this a recoverable error for now, don't disconnect.

                    except socket.timeout:
                        logger.warning(f"Socket timeout while reading image data. Received {bytes_received}/{img_size} bytes.")
                        self._handle_connection_error() 
                        continue
                    except ConnectionError as e: # From explicit raise
                        logger.error(f"Connection error while reading image data: {e}")
                        self._handle_connection_error()
                        continue
                    except Exception as e:
                        logger.error(f"Error processing image data: {e}")
                        # Depending on the error, might want to self._handle_connection_error()
                        # For now, log and continue to allow recovery for next frame.
                        continue # Skip to next iteration

                    last_video_request = time.time() # Update only on successful send AND attempt to process
            
            except (socket.timeout, ConnectionError, ConnectionResetError) as e:
                logger.warning(f"Connection error in _run_client: {e}")
                self._handle_connection_error()
                
            except Exception as e:
                logger.error(f"Unexpected error in warehouse client loop: {e}", exc_info=True)
                self._handle_connection_error()

        self._cleanup_socket()
        logger.info("Warehouse camera client thread finished.")
            
    def _handle_connection_error(self, is_connect_phase: bool = False):
        self.is_connected = False
        if self.socket:
            try:
                self.socket.close()
            except Exception as e:
                logger.debug(f"Error closing socket during error handling: {e}")
            self.socket = None
        
        self.consecutive_failures +=1
            
        if self._stop_event.is_set():
            logger.info("Stop event set, not attempting reconnection.")
            return

        delay_factor_exponent = min(self.consecutive_failures, 8)
        current_delay = self._reconnect_delay * (1.5 ** delay_factor_exponent)
        delay = min(current_delay, self._max_reconnect_delay)
        
        if is_connect_phase:
            logger.info(f"Failed to connect. Retrying in {delay:.1f} seconds...")
        else:
            logger.info(f"Connection lost. Reconnecting in {delay:.1f} seconds...")
        
        time.sleep(delay)
            
    def _cleanup_socket(self):
        if self.socket:
            logger.info("Cleaning up warehouse camera socket...")
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except (OSError, socket.error) as e:
                logger.debug(f"Error during socket shutdown (ignorable if already closed): {e}")
            finally:
                try:
                    self.socket.close()
                except (OSError, socket.error) as e:
                    logger.debug(f"Error during socket close (ignorable if already closed): {e}")
                self.socket = None
        logger.info("Warehouse camera socket cleaned up.") 