import socket
import logging
import time
import queue
import threading
import cv2
import numpy as np
from typing import Optional
import os # For path manipulation if needed for config imports
import sys

# --- Python Path Modification & Config Import ---
# Add the project\'s root directory to sys.path to allow absolute imports like 'from client.src...'
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
if project_root not in sys.path:
    sys.path.append(project_root)

from client.src.config import (
    DIRECT_WAREHOUSE_VIDEO_HOST, 
    DIRECT_WAREHOUSE_VIDEO_PORT,
    DIRECT_WAREHOUSE_CONNECTION_TIMEOUT,
    DIRECT_WAREHOUSE_FRAME_TIMEOUT
)

# Get a logger for this module. Assumes logging is configured in client/src/main.py.
logger = logging.getLogger(__name__) # Use module name for logger

class DirectWarehouseFeedReceiver:
    """
    Connects to an external "Warehouse Camera Server" via a direct TCP socket
    to fetch JPEG video frames. This is typically used for a lower-latency raw feed
    from an IP camera or similar source, bypassing the Raspberry Pi for this specific stream.

    The client runs in a separate thread and continuously attempts to connect. Once connected,
    it sends a "G 1" command to request a JPEG video frame. The server is expected to
    respond with raw JPEG data terminated by an End Of Image (EOI) marker (0xFFD9).

    Received video frames (decoded as NumPy arrays) are made available through a
    thread-safe queue. Includes automatic reconnection logic with exponential backoff.

    Attributes:
        host (str): IP address of the direct warehouse camera source.
        port (int): Port number of the direct warehouse camera source.
        socket (Optional[socket.socket]): The TCP socket connection.
        is_connected (bool): True if the socket is currently connected.
        video_queue (queue.Queue): Thread-safe queue for received video frames (np.ndarray).
        thread (Optional[threading.Thread]): The dedicated thread running the client loop.
        _stop_event (threading.Event): Event to signal the client thread to stop.
        _jpeg_soi (bytes): JPEG Start of Image marker (0xFFD8).
        _jpeg_eoi (bytes): JPEG End of Image marker (0xFFD9).
    """
    def __init__(self, 
                 host: str = DIRECT_WAREHOUSE_VIDEO_HOST, 
                 port: int = DIRECT_WAREHOUSE_VIDEO_PORT):
        """
        Initializes the DirectWarehouseFeedReceiver.

        Args:
            host: IP address of the direct warehouse camera source.
            port: Port number of the direct warehouse camera source.
        """
        self.host = host
        self.port = port
        self.socket: Optional[socket.socket] = None
        self.is_connected = False
        self.video_queue: queue.Queue[Optional[np.ndarray]] = queue.Queue(maxsize=2)  # Small queue for latest frames
        self.thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        
        # Reconnection parameters
        self._initial_reconnect_delay_sec = 1.0
        self._max_reconnect_delay_sec = 30.0
        self._current_reconnect_delay_sec = self._initial_reconnect_delay_sec
        self.consecutive_failures = 0

        # JPEG Start of Image (SOI) and End of Image (EOI) markers
        self._jpeg_soi = b'\xff\xd8'
        self._jpeg_eoi = b'\xff\xd9'
        
        logger.info(f"DirectWarehouseFeedReceiver initialized for {self.host}:{self.port}")

    def connect(self):
        """
        Starts the client thread to connect to the direct warehouse camera feed.
        If already connected and the thread is running, this method does nothing.
        The thread handles connection, sending "G 1" requests, and receiving frames.
        """
        if self.thread and self.thread.is_alive():
            logger.warning("DirectWarehouseFeedReceiver: Connect called, but thread is already running.")
            return
        if self.is_connected:
            logger.warning("DirectWarehouseFeedReceiver: Connect called, but already reported as connected.")
            # This state might be inconsistent, proceed to start thread to ensure it runs or resets.

        self._stop_event.clear() # Ensure stop event is clear before starting
        self.consecutive_failures = 0 # Reset failure count for a new connection sequence
        self._current_reconnect_delay_sec = self._initial_reconnect_delay_sec # Reset delay
        
        self.thread = threading.Thread(target=self._run_client_loop, name="DirectWarehouseFeedThread")
        self.thread.daemon = True # Allows main program to exit even if this thread is running.
        self.thread.start()
        logger.info(f"DirectWarehouseFeedReceiver: Connection process started in thread {self.thread.name}.")
        
    def disconnect(self):
        """
        Stops the client thread and disconnects the socket.
        Waits for the client thread to terminate.
        """
        logger.info("DirectWarehouseFeedReceiver: Disconnect requested.")
        if not (self.thread and self.thread.is_alive()) and not self.is_connected:
            logger.info("DirectWarehouseFeedReceiver: Already disconnected or thread not running.")
            self._cleanup_socket_and_state() # Ensure state is clean
            return
            
        self._stop_event.set() # Signal the thread to stop
        
        if self.thread:
            logger.debug("DirectWarehouseFeedReceiver: Waiting for client thread to join...")
            self.thread.join(timeout=DIRECT_WAREHOUSE_CONNECTION_TIMEOUT + DIRECT_WAREHOUSE_FRAME_TIMEOUT + 0.5) # Wait a bit longer than operations
            if self.thread.is_alive():
                logger.warning("DirectWarehouseFeedReceiver: Client thread did not terminate cleanly within timeout.")
            else:
                logger.info("DirectWarehouseFeedReceiver: Client thread joined successfully.")
        
        self._cleanup_socket_and_state() # Final cleanup after thread attempts to stop
        logger.info("DirectWarehouseFeedReceiver: Disconnect process finished.")

    def _cleanup_socket_and_state(self):
        """Internal helper to close socket and reset connection state variables."""
        if self.socket:
            logger.debug("DirectWarehouseFeedReceiver: Cleaning up socket...")
            try: # Attempt graceful shutdown
                self.socket.shutdown(socket.SHUT_RDWR)
            except (OSError, socket.error):
                pass # Ignore errors if already closed or not connected
            try:
                self.socket.close()
            except (OSError, socket.error):
                pass # Ignore errors if already closed
            self.socket = None
            logger.debug("DirectWarehouseFeedReceiver: Socket closed and cleaned up.")
        self.is_connected = False
        self.thread = None # Clear thread reference after it has been joined
        # Clear the video queue on disconnect
        while not self.video_queue.empty():
            try: self.video_queue.get_nowait()
            except queue.Empty: break

    def get_video_frame(self) -> Optional[np.ndarray]:
        """
        Retrieves the latest video frame from the queue (non-blocking).

        Returns:
            The video frame as a NumPy ndarray (RGB format), or None if the queue is empty.
        """
        try:
            return self.video_queue.get_nowait()
        except queue.Empty:
            return None
            
    def _attempt_socket_connection(self) -> Optional[socket.socket]:
        """Creates a new socket and attempts to connect to the server."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Generally not needed for client sockets
            sock.settimeout(DIRECT_WAREHOUSE_CONNECTION_TIMEOUT) # Timeout for the connect() call
            
            logger.info(f"DirectWarehouseFeedReceiver: Attempting to connect to {self.host}:{self.port} (timeout: {DIRECT_WAREHOUSE_CONNECTION_TIMEOUT}s)")
            sock.connect((self.host, self.port))
            
            # After connection, set a shorter timeout for send/recv operations
            sock.settimeout(DIRECT_WAREHOUSE_FRAME_TIMEOUT)
            self.is_connected = True
            logger.info(f"DirectWarehouseFeedReceiver: Successfully connected to {self.host}:{self.port}. Frame recv timeout: {DIRECT_WAREHOUSE_FRAME_TIMEOUT}s.")
            
            # Reset reconnection logic on successful connection
            self.consecutive_failures = 0
            self._current_reconnect_delay_sec = self._initial_reconnect_delay_sec
            return sock
        except socket.timeout:
            logger.warning(f"DirectWarehouseFeedReceiver: Timeout connecting to {self.host}:{self.port}.")
        except ConnectionRefusedError:
            logger.warning(f"DirectWarehouseFeedReceiver: Connection refused by {self.host}:{self.port}.")
        except OSError as e:
            logger.error(f"DirectWarehouseFeedReceiver: OS error connecting to {self.host}:{self.port}: {e}")
        except Exception as e:
            logger.error(f"DirectWarehouseFeedReceiver: Unexpected error creating/connecting socket: {e}", exc_info=True)
        
        self.is_connected = False # Ensure disconnected state on any failure
        return None
            
    def _run_client_loop(self):
        """
        Main loop for the client thread. Handles connection, sending "G 1" requests,
        receiving JPEG frame data, decoding frames, and putting them into the queue.
        Includes reconnection logic with exponential backoff.
        """
        last_frame_request_time = 0
        # Target ~30 FPS for requests, actual rate depends on server & network.
        # Interval slightly less than 1/FPS to account for processing time.
        frame_request_interval_sec = 1.0 / 30.0 

        while not self._stop_event.is_set():
            try:
                if self.socket is None or not self.is_connected:
                    self.socket = self._attempt_socket_connection()
                    if self.socket is None:
                        self._handle_connection_failure()
                        continue # Retry connection after delay
                
                # If connected, proceed to request and process frames
                current_time = time.time()
                if current_time - last_frame_request_time >= frame_request_interval_sec:
                    command_g1 = b"G 1" # Server expects "G 1" for a frame, no newline needed by some servers.
                                       # If server needs newline: b"G 1\n"
                    try:
                        self.socket.sendall(command_g1)
                        logger.debug(f"DirectWarehouseFeedReceiver: Sent command '{command_g1.decode()}'.")
                    except (socket.error, socket.timeout) as e_send:
                        logger.error(f"DirectWarehouseFeedReceiver: Error sending command '{command_g1.decode()}': {e_send}")
                        self._handle_connection_failure()
                        continue # Retry connection

                    # --- Receive JPEG frame data --- 
                    frame_data_buffer = bytearray()
                    receive_start_time = time.time()
                    # Use configured frame timeout for the entire frame reception process.
                    # Socket's internal timeout (DIRECT_WAREHOUSE_FRAME_TIMEOUT) applies per recv() call.
                    # This outer timeout ensures the whole frame process doesn't hang indefinitely.
                    overall_frame_receive_timeout = DIRECT_WAREHOUSE_FRAME_TIMEOUT * 1.5 # e.g., 2s * 1.5 = 3s for whole frame
                    RECV_CHUNK_SIZE = 4096 # Size of chunks to read from socket
                    MAX_FRAME_BYTES = 10 * 1024 * 1024 # 10MB sanity limit for a frame

                    try:
                        while True: # Loop to read chunks until EOI or timeout
                            if time.time() - receive_start_time > overall_frame_receive_timeout:
                                logger.warning(f"DirectWarehouseFeedReceiver: Overall timeout ({overall_frame_receive_timeout:.1f}s) receiving frame. Buffer size: {len(frame_data_buffer)} bytes.")
                                raise socket.timeout("Overall frame receive timeout")
                            
                            if len(frame_data_buffer) > MAX_FRAME_BYTES:
                                logger.warning(f"DirectWarehouseFeedReceiver: Frame data exceeded max limit ({MAX_FRAME_BYTES} bytes) before EOI. Discarding.")
                                raise ConnectionError("Frame data limit exceeded")

                            # Read a chunk from the socket
                            # self.socket.settimeout is already DIRECT_WAREHOUSE_FRAME_TIMEOUT
                            chunk = self.socket.recv(RECV_CHUNK_SIZE)
                            if not chunk: # Connection closed by peer
                                logger.warning("DirectWarehouseFeedReceiver: Connection closed by server while receiving frame data (recv returned empty).")
                                raise ConnectionAbortedError("Socket recv returned empty, connection closed by peer")
                            
                            frame_data_buffer.extend(chunk)
                            
                            # Check if the EOI marker is present in the received data
                            if self._jpeg_eoi in frame_data_buffer: # More robust check than endswith for streamed data
                                logger.debug(f"DirectWarehouseFeedReceiver: EOI marker found. Total frame data: {len(frame_data_buffer)} bytes.")
                                break # Full JPEG frame likely received
                        
                        # --- Process received frame data --- 
                        # Find the last SOI marker to handle cases where multiple frames might be buffered or partial data exists.
                        soi_index = frame_data_buffer.rfind(self._jpeg_soi)
                        eoi_index = frame_data_buffer.rfind(self._jpeg_eoi, soi_index if soi_index !=-1 else 0) # Search EOI after last SOI

                        if soi_index != -1 and eoi_index != -1 and eoi_index > soi_index:
                            # Extract the valid JPEG data from the last SOI to its corresponding EOI + 2 bytes for EOI marker length
                            jpeg_to_decode = frame_data_buffer[soi_index : eoi_index + len(self._jpeg_eoi)]
                            
                            decoded_frame_bgr = cv2.imdecode(np.frombuffer(jpeg_to_decode, dtype=np.uint8), cv2.IMREAD_COLOR)
                            if decoded_frame_bgr is not None:
                                decoded_frame_rgb = cv2.cvtColor(decoded_frame_bgr, cv2.COLOR_BGR2RGB)
                                # Put frame in queue, making space if full
                                if self.video_queue.full():
                                    try: self.video_queue.get_nowait() # Discard oldest
                                    except queue.Empty: pass # Should not happen with .full() check
                                self.video_queue.put_nowait(decoded_frame_rgb)
                                logger.debug(f"DirectWarehouseFeedReceiver: Frame decoded ({decoded_frame_rgb.shape}) and queued.")
                            else:
                                logger.warning(f"DirectWarehouseFeedReceiver: cv2.imdecode failed. Data length: {len(jpeg_to_decode)}. SOI at {soi_index}, EOI at {eoi_index}.")
                        else:
                            logger.warning(f"DirectWarehouseFeedReceiver: Valid SOI/EOI pair not found in {len(frame_data_buffer)} bytes. SOI: {soi_index}, EOI: {eoi_index}.")

                    except (socket.timeout, ConnectionError, ConnectionAbortedError, ConnectionResetError) as e_recv:
                        logger.warning(f"DirectWarehouseFeedReceiver: Network error during frame reception: {e_recv}")
                        self._handle_connection_failure()
                        continue # Retry connection
                    except Exception as e_process_frame:
                        logger.error(f"DirectWarehouseFeedReceiver: Unexpected error processing frame data: {e_process_frame}", exc_info=True)
                        self._handle_connection_failure() # Assume connection is bad
                        continue # Retry connection

                    last_frame_request_time = current_time # Update time only after successful send/attempted recv
                else:
                    # Not time to request a frame yet, sleep briefly to prevent busy-waiting.
                    # This sleep also yields control, allowing the _stop_event to be checked.
                    time.sleep(0.005) # Small sleep if no request sent

            except Exception as e_outer_loop:
                logger.error(f"DirectWarehouseFeedReceiver: Unhandled exception in main client loop: {e_outer_loop}", exc_info=True)
                self._handle_connection_failure() # Attempt to reset and reconnect

        # --- Loop exited (due to _stop_event) --- 
        self._cleanup_socket_and_state() # Ensure final cleanup when loop terminates
        logger.info(f"DirectWarehouseFeedReceiver: Client loop in thread {threading.current_thread().name} has finished.")

    def _handle_connection_failure(self):
        """Handles connection failure, closes socket, and schedules reconnection attempt with backoff."""
        self.is_connected = False # Ensure connection status is False
        if self.socket:
            self._cleanup_socket_and_state() # Close existing socket and reset related state
        
        self.consecutive_failures += 1
        
        if self._stop_event.is_set(): # Check if we should stop trying to reconnect
            logger.info("DirectWarehouseFeedReceiver: Stop event is set, not attempting reconnection.")
            return

        # Calculate exponential backoff delay
        delay_exponent = min(self.consecutive_failures, 6) # Cap exponent to prevent excessively long waits (e.g., 1.5^6 ~ 11s)
        self._current_reconnect_delay_sec = self._initial_reconnect_delay_sec * (1.5 ** delay_exponent)
        actual_delay = min(self._current_reconnect_delay_sec, self._max_reconnect_delay_sec)
        
        logger.info(f"DirectWarehouseFeedReceiver: Connection failed/lost. Retrying in {actual_delay:.1f} seconds... (Failures: {self.consecutive_failures})")
        time.sleep(actual_delay) # Wait before the next connection attempt 