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
        
    def connect(self):
        if self.is_connected:
            logger.warning("Already connected to warehouse camera")
            return
            
        self.thread = threading.Thread(target=self._run_client, name="WarehouseCameraThread")
        self.thread.daemon = True
        self.thread.start()
        
    def disconnect(self):
        if not self.is_connected:
            return
            
        logger.info("Disconnecting from warehouse camera...")
        self._stop_event.set()
        
        if self.thread:
            self.thread.join(timeout=5.0)
            if self.thread.is_alive():
                logger.warning("Warehouse camera thread did not terminate cleanly")
                
        self.is_connected = False
        self.thread = None
        self._stop_event.clear()
        
    def get_warehouse_time(self) -> Optional[float]:
        """Get the latest warehouse time"""
        try:
            return self.time_queue.get_nowait()
        except queue.Empty:
            return None
            
    def get_video_frame(self) -> Optional[np.ndarray]:
        """Get the latest video frame from the warehouse camera"""
        try:
            return self.video_queue.get_nowait()
        except queue.Empty:
            return None
            
    def _parse_time_response(self, response: str) -> Optional[float]:
        """Parse the warehouse time response"""
        match = re.search(r'<WAREHOUSE TIME="([\d.]+)" />', response)
        if match:
            try:
                return float(match.group(1))
            except ValueError:
                logger.warning(f"Invalid time value in response: {response}")
        return None
            
    def _create_socket(self) -> Optional[socket.socket]:
        """Create and configure a new socket"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(5.0)  # Initial connection timeout
            return sock
        except Exception as e:
            logger.error(f"Error creating socket: {e}")
            return None
            
    def _run_client(self):
        consecutive_failures = 0
        last_time_request = 0
        time_request_interval = 1.0  # Request time every second
        last_video_request = 0
        video_request_interval = 0.016  # Target 60 FPS
        
        while not self._stop_event.is_set():
            try:
                if self.socket is None:
                    self.socket = self._create_socket()
                    if self.socket is None:
                        raise ConnectionError("Failed to create socket")
                        
                    # Connect to server
                    logger.info(f"Attempting to connect to warehouse camera at {self.host}:{self.port}")
                    self.socket.connect((self.host, self.port))
                    self.is_connected = True
                    logger.info(f"Connected to warehouse camera at {self.host}:{self.port}")
                    
                    # Reset reconnection delay on successful connection
                    self._reconnect_delay = 1.0
                    consecutive_failures = 0
                    
                    # Set a shorter timeout for data operations
                    self.socket.settimeout(0.1)  # Reduced timeout for faster response
                
                current_time = time.time()
                
                # Request time every second
                if current_time - last_time_request >= time_request_interval:
                    self.socket.sendall(b"G 0")
                    logger.debug("Sent G 0 command")
                    
                    # Read time response
                    response = bytearray()
                    start_time = time.time()
                    
                    while time.time() - start_time < 0.1:  # 100ms timeout for reading
                        try:
                            char = self.socket.recv(1)
                            if not char:
                                raise ConnectionError("Connection closed by server")
                            response.extend(char)
                            if char == b'\n':
                                break
                        except socket.timeout:
                            continue  # Continue trying to read
                    
                    if response:
                        try:
                            time_str = response.decode('utf-8').strip()
                            logger.debug(f"Received time response: {time_str}")
                            warehouse_time = self._parse_time_response(time_str)
                            if warehouse_time is not None:
                                try:
                                    self.time_queue.put_nowait(warehouse_time)
                                    consecutive_failures = 0  # Reset failure count on successful read
                                except queue.Full:
                                    try:
                                        self.time_queue.get_nowait()
                                        self.time_queue.put_nowait(warehouse_time)
                                    except queue.Empty:
                                        pass
                        except UnicodeDecodeError:
                            logger.warning("Failed to decode time response")
                    
                    last_time_request = current_time
                
                # Request video frame at specified interval
                if current_time - last_video_request >= video_request_interval:
                    self.socket.sendall(b"G 1")
                    logger.debug("Sent G 1 command")
                    
                    # Read the image data directly
                    frame_data = bytearray()
                    start_time = time.time()
                    
                    while time.time() - start_time < 0.1:  # 100ms timeout for reading frame
                        try:
                            chunk = self.socket.recv(4096)
                            if not chunk:
                                break
                            frame_data.extend(chunk)
                        except socket.timeout:
                            # If we get a timeout, assume we have the complete frame
                            break
                    
                    if frame_data:
                        try:
                            # Decode frame
                            nparr = np.frombuffer(frame_data, np.uint8)
                            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                            if frame is not None:
                                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                try:
                                    self.video_queue.put_nowait(frame)
                                except queue.Full:
                                    try:
                                        self.video_queue.get_nowait()
                                        self.video_queue.put_nowait(frame)
                                    except queue.Empty:
                                        pass
                        except Exception as e:
                            logger.warning(f"Failed to decode video frame: {e}")
                    
                    last_video_request = current_time
                
            except (socket.timeout, ConnectionError, ConnectionResetError) as e:
                logger.warning(f"Connection error: {e}")
                consecutive_failures += 1
                self._handle_connection_error()
                
            except Exception as e:
                logger.error(f"Unexpected error in warehouse client: {e}")
                consecutive_failures += 1
                self._handle_connection_error()
                
    def _handle_connection_error(self):
        """Handle connection errors and implement exponential backoff"""
        self.is_connected = False
        if self.socket:
            try:
                self.socket.close()
            except Exception as e:
                logger.debug(f"Error closing socket: {e}")
            self.socket = None
            
        # Calculate delay with exponential backoff
        delay = min(self._reconnect_delay * (1.5 ** min(consecutive_failures, 5)), self._max_reconnect_delay)
        logger.info(f"Reconnecting in {delay:.1f} seconds...")
        time.sleep(delay)
        self._reconnect_delay = delay  # Update for next time
            
    def _cleanup_socket(self):
        """Clean up socket resources"""
        if self.socket:
            try:
                # Shutdown the socket before closing
                self.socket.shutdown(socket.SHUT_RDWR)
            except Exception as e:
                logger.debug(f"Error during socket shutdown: {e}")
            finally:
                try:
                    self.socket.close()
                except Exception as e:
                    logger.debug(f"Error during socket close: {e}")
                self.socket = None
        self.is_connected = False
        logger.info("Warehouse camera client disconnected")