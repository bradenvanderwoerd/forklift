import socket
import cv2
import numpy as np
import threading
import queue
import logging
import time
from typing import Optional

logger = logging.getLogger(__name__)

class WarehouseCameraClient:
    def __init__(self, host: str = "192.168.0.100", port: int = 4001):
        self.host = host
        self.port = port
        self.socket: Optional[socket.socket] = None
        self.is_connected = False
        self.video_queue = queue.Queue(maxsize=10)
        self.thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        
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
        
    def get_video_frame(self) -> Optional[np.ndarray]:
        """Get the latest video frame from the warehouse camera"""
        try:
            return self.video_queue.get_nowait()
        except queue.Empty:
            return None
            
    def _run_client(self):
        try:
            # Create socket with proper options
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.socket.settimeout(5.0)  # Set a reasonable timeout
            
            # Connect to server
            self.socket.connect((self.host, self.port))
            self.is_connected = True
            logger.info(f"Connected to warehouse camera at {self.host}:{self.port}")
            
            while not self._stop_event.is_set():
                try:
                    # Send request for frame with newline
                    self.socket.sendall(b"G 1\n")
                    
                    # Receive frame size (4 bytes)
                    size_data = self.socket.recv(4)
                    if not size_data:
                        logger.warning("No size data received, connection may be closed")
                        break
                        
                    frame_size = int.from_bytes(size_data, byteorder='big')
                    
                    # Receive frame data
                    frame_data = bytearray()
                    while len(frame_data) < frame_size:
                        chunk = self.socket.recv(min(frame_size - len(frame_data), 4096))
                        if not chunk:
                            logger.warning("Connection closed while receiving frame data")
                            break
                        frame_data.extend(chunk)
                    
                    if len(frame_data) == frame_size:
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
                    
                    # Small delay to control frame rate
                    time.sleep(0.033)  # ~30 FPS
                    
                except socket.timeout:
                    logger.warning("Socket timeout while receiving data")
                    break
                except ConnectionResetError:
                    logger.error("Connection reset by peer")
                    break
                except Exception as e:
                    logger.error(f"Error receiving warehouse camera frame: {e}")
                    break
                    
        except Exception as e:
            logger.error(f"Error in warehouse camera client: {e}")
        finally:
            self._cleanup_socket()
            
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