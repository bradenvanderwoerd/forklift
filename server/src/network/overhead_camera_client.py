import socket
import logging
import time
import re
import queue
import threading
import cv2
import numpy as np
from typing import Optional, Union

logger = logging.getLogger(__name__)

class WarehouseCameraClient:
    """Connects to an external "Warehouse Camera Server" to fetch synchronized time
    and JPEG video frames.

    The client runs in a separate thread, continuously attempting to connect to the
    specified server. Once connected, it periodically sends commands:
    - "G 0": To request the warehouse server's current time (expects XML-like response).
    - "G 1": To request a JPEG video frame.

    Received time and decoded video frames (as NumPy arrays) are made available
    through thread-safe queues (`time_queue` and `video_queue`).
    Includes automatic reconnection logic with exponential backoff.
    """
    def __init__(self, host: str = "192.168.0.100", port: int = 4001):
        """Initializes the WarehouseCameraClient.

        Args:
            host: IP address of the warehouse camera server.
            port: Port number of the warehouse camera server.
        """
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
        """Starts the client thread to connect to the warehouse camera server.
        If already connected and thread is running, this method does nothing.
        """
        if self.is_connected and self.thread and self.thread.is_alive():
            logger.warning("Already connected and thread is running for warehouse camera")
            return
            
        self._stop_event.clear()
        self.thread = threading.Thread(target=self._run_client, name="WarehouseCameraThread")
        self.thread.daemon = True
        self.thread.start()
        
    def disconnect(self):
        """Stops the client thread and disconnects from the server.
        Waits for the client thread to terminate.
        """
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
        """Retrieves the latest warehouse time from the queue (non-blocking).

        Returns:
            The warehouse time as a float, or None if the queue is empty.
        """
        try:
            return self.time_queue.get_nowait()
        except queue.Empty:
            return None
            
    def get_video_frame(self) -> Optional[np.ndarray]:
        """Retrieves the latest video frame from the queue (non-blocking).

        Returns:
            The video frame as a NumPy ndarray, or None if the queue is empty.
        """
        try:
            return self.video_queue.get_nowait()
        except queue.Empty:
            return None
            
    def _parse_time_response(self, response: str) -> Optional[float]:
        """Parses the warehouse time from the server's XML-like response string.
        Expected format: '<WAREHOUSE TIME="12345.678" />'

        Args:
            response: The string response from the server for a time request.

        Returns:
            The parsed time as a float, or None if parsing fails.
        """
        match = re.search(r'<WAREHOUSE TIME="([\d.]+)" />', response)
        if match:
            try:
                return float(match.group(1))
            except ValueError:
                logger.warning(f"Invalid time value in response: {response}")
        return None
            
    def _create_socket(self) -> Optional[socket.socket]:
        """Creates a new socket and attempts to connect to the server.

        Returns:
            A connected socket object, or None if socket creation or connection fails.
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) # Optional
            sock.settimeout(5.0)
            logger.info(f"Attempting to connect to warehouse camera at {self.host}:{self.port}")
            sock.connect((self.host, self.port))
            self.is_connected = True
            logger.info(f"Connected to warehouse camera at {self.host}:{self.port}")
            
            self._reconnect_delay = 1.0
            self.consecutive_failures = 0
            sock.settimeout(2.0)
            return sock
        except socket.timeout:
            logger.warning(f"Timeout connecting to warehouse camera at {self.host}:{self.port}.")
        except ConnectionRefusedError:
            logger.warning(f"Connection refused by warehouse camera at {self.host}:{self.port}.")
        except OSError as e: # Catches other address-related errors, e.g., Host not found
            logger.error(f"OS error connecting to warehouse camera at {self.host}:{self.port}: {e}")
        except Exception as e:
            logger.error(f"Unexpected error creating/connecting socket: {e}", exc_info=True)
        return None
            
    def _run_client(self):
        """Main loop for the client thread. Handles connection, requests, and data processing."""
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
                        
                current_time = time.time()
                
                # Request time (G 0)
                if current_time - last_time_request >= time_request_interval:
                    self._send_command_and_receive_time()
                    last_time_request = current_time
                
                # Request video frame (G 1)
                if current_time - last_video_request >= video_request_interval:
                    self._send_command_and_receive_video()
                    last_video_request = current_time

            except (socket.timeout, ConnectionError, ConnectionResetError) as e:
                logger.warning(f"Connection error in _run_client: {e}")
                self._handle_connection_error()
                
            except Exception as e:
                logger.error(f"Unexpected error in warehouse client loop: {e}", exc_info=True)
                self._handle_connection_error()

        self._cleanup_socket()
        logger.info("Warehouse camera client thread finished.")
            
    def _send_command_and_receive_time(self):
        """Sends 'G 0' command and processes the time response."""
        if not self.socket: return
        command_to_send = b"G 0\n" # Ensure newline as per DirectWarehouseFeed
        logger.debug(f"Sending time command: {command_to_send!r}")
        try:
            self.socket.sendall(command_to_send)
            logger.debug(f"Successfully sent command: {command_to_send!r}")
        except Exception as e:
            logger.error(f"Error sending command {command_to_send!r}: {e}")
            self._handle_connection_error()
            return
        
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
                time_str = response_bytes.decode('utf-8', errors='ignore').strip()
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

    def _send_command_and_receive_video(self):
        """Sends 'G 1' command and processes the JPEG video frame response."""
        if not self.socket: return
        command_to_send = b"G 1\n" # Ensure newline
        logger.debug(f"Sending video command: {command_to_send!r}")
        try:
            self.socket.sendall(command_to_send)
            logger.debug(f"Successfully sent command: {command_to_send!r}")
        except Exception as e:
            logger.error(f"Error sending command {command_to_send!r}: {e}")
            self._handle_connection_error()
            return

        # Read JPEG stream until EOI marker
        frame_data = bytearray()
        image_receive_start_time = time.time()
        RECV_BUFFER_SIZE = 8192  # Buffer size for socket recv
        # Max expected image size (e.g., 5MB) to prevent runaway allocation
        MAX_IMAGE_BYTES = 5 * 1024 * 1024 
        # Overall timeout for receiving a single frame (e.g., 2x socket's own timeout)
        # self.socket.gettimeout() is 2.0s after connect.
        FRAME_RECEIVE_TIMEOUT = self.socket.gettimeout() * 2.5 # e.g., 5 seconds total for a frame

        try:
            while True:
                if time.time() - image_receive_start_time > FRAME_RECEIVE_TIMEOUT:
                    logger.warning(f"Timeout receiving complete image data. Received {len(frame_data)} bytes in {FRAME_RECEIVE_TIMEOUT:.1f}s.")
                    self._handle_connection_error() # Mark connection as bad
                    raise socket.timeout("Image receive overall timeout")

                # How much more data can we read up to RECV_BUFFER_SIZE without overshooting MAX_IMAGE_BYTES too much
                bytes_to_read = min(RECV_BUFFER_SIZE, MAX_IMAGE_BYTES - len(frame_data) + RECV_BUFFER_SIZE//2) # Allow slight overshoot for check
                
                if bytes_to_read <= 0 and len(frame_data) >= MAX_IMAGE_BYTES : # Already over limit, check before recv
                   logger.warning(f"Exceeded max image byte limit ({MAX_IMAGE_BYTES}) before EOI. Frame data {len(frame_data)} bytes.")
                   self._handle_connection_error()
                   raise ConnectionError("Max image byte limit exceeded before EOI")

                chunk = self.socket.recv(bytes_to_read if bytes_to_read > 0 else RECV_BUFFER_SIZE)
                if not chunk:
                    logger.warning("Connection closed by server while reading image data (recv returned empty).")
                    self._handle_connection_error()
                    raise ConnectionError("Connection closed by server while reading image data")
                
                frame_data.extend(chunk)
                # logger.debug(f"Received image chunk: {len(chunk)} bytes, total: {len(frame_data)}")

                # Check for EOI marker (0xFFD9)
                # Using endswith is efficient if EOI is indeed the last part of the chunk or accumulated data.
                if frame_data.endswith(b'\xff\xd9'):
                    logger.debug(f"EOI marker (FFD9) found. Total image data bytes: {len(frame_data)}")
                    break # Successfully received what seems to be a full JPEG frame

                if len(frame_data) > MAX_IMAGE_BYTES:
                    logger.warning(f"Exceeded max image byte limit ({MAX_IMAGE_BYTES}) after recv. Frame data {len(frame_data)} bytes without EOI. Discarding frame.")
                    self._handle_connection_error()
                    raise ConnectionError("Max image byte limit exceeded after recv, no EOI")
            
            # If loop exited due to break (EOI found)
            # Try to find the Start of Image (SOI) marker 0xFFD8, usually at the beginning.
            soi_index = frame_data.rfind(b'\xff\xd8')
            if soi_index != -1:
                # Slice from the last SOI marker to the end (which includes EOI)
                jpeg_data_to_decode = frame_data[soi_index:]
                logger.debug(f"Attempting to decode JPEG data from last SOI. Length: {len(jpeg_data_to_decode)} bytes.")
                
                frame_np = cv2.imdecode(np.frombuffer(jpeg_data_to_decode, dtype=np.uint8), cv2.IMREAD_COLOR)
                
                if frame_np is not None:
                    logger.debug(f"Successfully decoded frame: {frame_np.shape}")
                    try:
                        while not self.video_queue.empty(): # Clear old frames
                            self.video_queue.get_nowait()
                        self.video_queue.put_nowait(frame_np)
                    except queue.Full:
                        logger.warning("Video queue full, dropping frame.")
                else:
                    logger.warning(f"cv2.imdecode failed for {len(jpeg_data_to_decode)} bytes (SOI found). Original data size: {len(frame_data)} bytes.")
            else:
                logger.warning(f"EOI marker found but no SOI marker in {len(frame_data)} bytes of data. Cannot decode.")

        except (socket.timeout, ConnectionError, ConnectionResetError) as e: 
            logger.error(f"Network error during image data reception: {e}")
            # _handle_connection_error() should have been called by the raiser or logic path leading here.
            # If not, it's called in the main exception handler for the client loop.
            # We 'continue' to allow the main client loop to attempt reconnection.
            return 
        except Exception as e:
            logger.error(f"Unexpected error processing image data: {e}", exc_info=True)
            self._handle_connection_error() # Ensure connection reset for unexpected issues
            return

    def _handle_connection_error(self, is_connect_phase: bool = False):
        """Handles cleanup and backoff delay calculation after a connection failure."""
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
        """Closes the socket if it is open."""
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