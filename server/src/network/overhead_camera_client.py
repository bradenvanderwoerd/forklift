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
                    self.socket.sendall(b"G 0\\n")
                    logger.debug("Sent G 0 command")
                    
                    response_bytes = bytearray()
                    read_start_time = time.time()
                    
                    while time.time() - read_start_time < self.socket.gettimeout():
                        try:
                            char = self.socket.recv(1)
                            if not char:
                                raise ConnectionError("Connection closed by server while reading time")
                            response_bytes.extend(char)
                            if char == b'\\n':
                                break
                        except socket.timeout:
                            logger.debug("Socket timeout while waiting for G 0 response char.")
                            break
                    
                    if response_bytes:
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
                    self.socket.sendall(b"G 1\\n")
                    logger.debug("Sent G 1 command")
                    
                    frame_data = bytearray()
                    image_read_start_time = time.time()
                    buffer_size = 8192
                    
                    while time.time() - image_read_start_time < self.socket.gettimeout():
                        try:
                            chunk = self.socket.recv(buffer_size)
                            if not chunk:
                                logger.info("Server closed connection while reading image data.")
                                raise ConnectionError("Connection closed by server while reading image")
                            frame_data.extend(chunk)
                            if b'\\xff\\xd9' in frame_data: # Check for EOI marker for JPEG
                                logger.debug("JPEG EOI marker found.")
                                break
                        except socket.timeout:
                            logger.debug("Socket timeout while reading image chunk for G 1.")
                            break
                    
                    if frame_data:
                        soi = frame_data.rfind(b'\\xff\\xd8')
                        eoi = frame_data.rfind(b'\\xff\\xd9')

                        if soi != -1 and eoi != -1 and eoi > soi:
                            jpeg_data = frame_data[soi:eoi+2]
                            logger.debug(f"Extracted JPEG data: {len(jpeg_data)} bytes")
                            try:
                                nparr = np.frombuffer(jpeg_data, np.uint8)
                                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                                if frame is not None:
                                    try:
                                        while not self.video_queue.empty():
                                            self.video_queue.get_nowait()
                                        self.video_queue.put_nowait(frame)
                                    except queue.Full:
                                        pass
                                else:
                                    logger.warning("cv2.imdecode returned None for video frame.")
                            except Exception as e:
                                logger.warning(f"Failed to decode video frame: {e}. Data length: {len(jpeg_data)}")
                        else:
                            logger.warning(f"Could not find valid SOI/EOI in received G 1 data. Length: {len(frame_data)}")
                    else:
                        logger.debug("No frame data received for G 1.")
                    last_video_request = current_time
            
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