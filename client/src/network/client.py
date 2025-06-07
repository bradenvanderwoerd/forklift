import asyncio
import websockets
from websockets.connection import State
import json
import cv2
import numpy as np
from typing import Optional, Tuple, Dict, Any, Union
import threading
import queue
import logging
import os
import sys
import time # Added for sleep in cleanup
from colorama import Fore, Style, init as colorama_init

# Add the src directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from src.config import SERVER_HOST, COMMAND_PORT, VIDEO_PORT, OVERHEAD_VIDEO_PORT

colorama_init(autoreset=True)

class ColorFormatter(logging.Formatter):
    COLORS = {
        logging.DEBUG: Fore.CYAN,
        logging.INFO: Fore.GREEN,
        logging.WARNING: Fore.YELLOW,
        logging.ERROR: Fore.RED,
        logging.CRITICAL: Fore.MAGENTA + Style.BRIGHT,
    }
    def format(self, record):
        color = self.COLORS.get(record.levelno, "")
        message = super().format(record)
        return f"{color}{message}{Style.RESET_ALL}"

handler = logging.StreamHandler()
handler.setFormatter(ColorFormatter("%(levelname)s: %(message)s"))
logging.getLogger().handlers = [handler]
logging.getLogger().setLevel(logging.INFO)

logger = logging.getLogger(__name__)

class RobotClient:
    """
    Manages WebSocket connections to the Forklift robot server for sending commands
    and receiving video streams (onboard and Pi-processed overhead).

    This client runs an asyncio event loop in a dedicated thread to handle all
    network operations asynchronously. It provides methods to connect, disconnect,
    send commands, and retrieve video frames from queues. It also includes
    automatic reconnection logic.

    Attributes:
        host (str): IP address or hostname of the robot server.
        command_port (int): Port for the command WebSocket.
        onboard_video_port (int): Port for the onboard camera video WebSocket.
        overhead_video_port (int): Port for the Pi-processed overhead camera video WebSocket.
        command_ws (Optional[websockets.WebSocketClientProtocol]): WebSocket for commands.
        onboard_video_ws (Optional[websockets.WebSocketClientProtocol]): WebSocket for onboard video.
        overhead_video_ws (Optional[websockets.WebSocketClientProtocol]): WebSocket for overhead video.
        is_connected (bool): True if all critical connections are active.
        command_connected (bool): True if the command WebSocket is connected.
        onboard_video_connected (bool): True if the onboard video WebSocket is connected.
        overhead_video_connected (bool): True if the overhead video WebSocket is connected.
        onboard_video_queue (queue.Queue): Queue for received onboard video frames (np.ndarray).
        overhead_video_queue (queue.Queue): Queue for received overhead video frames (np.ndarray).
        loop (Optional[asyncio.AbstractEventLoop]): The asyncio event loop for this client.
        thread (Optional[threading.Thread]): The dedicated thread running the asyncio loop.
        _stop_event (Optional[asyncio.Event]): Asyncio event to signal the client loop to stop.
    """
    def __init__(self, host: str = SERVER_HOST, 
                 command_port: int = COMMAND_PORT, 
                 onboard_video_port: int = VIDEO_PORT, 
                 overhead_video_port: int = OVERHEAD_VIDEO_PORT):
        """
        Initializes the RobotClient.

        Args:
            host: The server's hostname or IP address.
            command_port: Port for the command WebSocket server.
            onboard_video_port: Port for the onboard video WebSocket server.
            overhead_video_port: Port for the Pi-processed overhead video WebSocket server.
        """
        self.host = host
        self.command_port = command_port
        self.onboard_video_port = onboard_video_port
        self.overhead_video_port = overhead_video_port
        
        self.command_ws: Optional[websockets.WebSocketClientProtocol] = None
        self.onboard_video_ws: Optional[websockets.WebSocketClientProtocol] = None
        self.overhead_video_ws: Optional[websockets.WebSocketClientProtocol] = None
        
        self.is_connected = False # Overall connection status
        self.command_connected = False
        self.onboard_video_connected = False
        self.overhead_video_connected = False

        # Thread-safe queues for passing video frames to the UI thread.
        # maxsize=1 ensures the UI gets the latest frame, dropping older ones if not consumed.
        self.onboard_video_queue: queue.Queue[Optional[np.ndarray]] = queue.Queue(maxsize=1)
        self.overhead_video_queue: queue.Queue[Optional[np.ndarray]] = queue.Queue(maxsize=1)
        
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.thread: Optional[threading.Thread] = None
        self._stop_event: Optional[asyncio.Event] = None # For signaling the async loop to stop
        
        # Asyncio tasks for video stream receivers
        self.onboard_video_receive_task: Optional[asyncio.Task] = None
        self.overhead_video_receive_task: Optional[asyncio.Task] = None
        
        logger.info(
            f"RobotClient initialized. Target Server: ws://{self.host}:"
            f"Cmd({self.command_port}), OnboardVid({self.onboard_video_port}), OverheadVid({self.overhead_video_port})"
        )
        
    def connect(self):
        """
        Starts the client's network operations in a dedicated thread.
        If already connected or a connection attempt is in progress, this method returns early.
        The dedicated thread will run an asyncio event loop to manage WebSocket connections.
        """
        if self.thread and self.thread.is_alive():
            logger.warning("RobotClient: Connect called, but client thread is already running.")
            return
        if self.is_connected:
            logger.warning("RobotClient: Connect called, but already connected.")
            return

        logger.info("RobotClient: Starting connection thread...")
        # Create a new thread for the asyncio event loop.
        # This prevents blocking the main UI thread.
        self.thread = threading.Thread(target=self._run_client_event_loop, name="RobotClientAsyncThread")
        self.thread.daemon = True # Allow main program to exit even if this thread is running.
        self.thread.start()
            
    def disconnect(self):
        """
        Signals the client to disconnect all WebSockets and stop its network thread.
        This method is thread-safe and waits for the network thread to terminate.
        """
        logger.info("RobotClient: Disconnect requested.")
        if not (self.thread and self.thread.is_alive()) and not self.is_connected:
            logger.warning("RobotClient: Disconnect called, but client thread not running or not connected.")
            # Perform cleanup just in case, though ideally, resources are tied to the loop/thread.
            self._cleanup_resources_immediate()
            return
            
        # Signal the asyncio loop to stop its operations.
        if self.loop and self._stop_event and not self._stop_event.is_set():
            logger.info("RobotClient: Signaling asyncio stop event.")
            self.loop.call_soon_threadsafe(self._stop_event.set)
        else:
            logger.warning("RobotClient: Cannot signal stop event: loop or event not initialized or already set.")
        
        # Wait for the network thread to finish.
        if self.thread:
            logger.info("RobotClient: Waiting for client thread to join...")
            self.thread.join(timeout=7.0) # Increased timeout for graceful shutdown
            if self.thread.is_alive():
                logger.warning("RobotClient: Client thread did not terminate cleanly within timeout.")
            else:
                logger.info("RobotClient: Client thread joined successfully.")

        # Final state reset (some of this might be redundant if _run_client_event_loop handles it well)
        self._cleanup_resources_immediate()
        logger.info("RobotClient: Disconnect process finished.")

    def _cleanup_resources_immediate(self):
        """
        Resets client state and resources. Called after the asyncio loop has stopped.
        This method should not perform asyncio operations.
        """
        self.is_connected = False
        self.command_connected = False
        self.onboard_video_connected = False
        self.overhead_video_connected = False

        self.command_ws = None
        self.onboard_video_ws = None
        self.overhead_video_ws = None
        
        self.onboard_video_receive_task = None
        self.overhead_video_receive_task = None
        
        self.loop = None
        self.thread = None
        # Clear queues
        while not self.onboard_video_queue.empty(): self.onboard_video_queue.get_nowait()
        while not self.overhead_video_queue.empty(): self.overhead_video_queue.get_nowait()
        logger.debug("RobotClient: Immediate resource cleanup performed.")

            
    def send_command(self, command_type: str, data: Optional[Dict[str, Any]] = None):
        """
        Sends a command to the robot server via the command WebSocket.
        The command is sent as a JSON message: {"command": command_type, "data": data}.
        This method is thread-safe.

        Args:
            command_type: The type of command (e.g., "drive", "servo").
            data: A dictionary containing data for the command. Defaults to None (empty dict).
        """
        if not self.command_connected or not self.loop or not self.command_ws:
            logger.warning(f"RobotClient: Command WebSocket not connected. Cannot send command '{command_type}'.")
            return

        if data is None:
            data = {}
        message = {"command": command_type, "data": data}
        try:
            # Schedule the asynchronous send operation in the client's asyncio loop.
            json_message = json.dumps(message)
            asyncio.run_coroutine_threadsafe(self._send_command_async(json_message), self.loop)
            logger.debug(f"RobotClient: Scheduled command: {json_message}")
        except Exception as e:
            logger.error(f"RobotClient: Error scheduling send_command '{command_type}': {e}")

    async def _send_command_async(self, json_message: str):
        """
        Asynchronously sends a JSON message over the command WebSocket.
        This coroutine runs in the client's asyncio event loop.

        Args:
            json_message: The JSON string to send.
        """
        if self.command_ws and self.command_connected and self.command_ws.open:
            try:
                await self.command_ws.send(json_message)
                logger.debug(f"RobotClient: Sent command: {json_message}")
            except websockets.exceptions.ConnectionClosed:
                logger.warning("RobotClient: Command WebSocket closed while trying to send. Marking as disconnected.")
                self.command_connected = False
                self.is_connected = False
            except Exception as e:
                logger.error(f"RobotClient: Error sending command async: {e}")
        else:
            logger.warning("RobotClient: Command WebSocket not available or closed when _send_command_async was called.")
            
    def get_onboard_video_frame(self) -> Optional[np.ndarray]:
        """
        Retrieves the latest onboard video frame from its queue. Non-blocking.

        Returns:
            A NumPy ndarray representing the RGB video frame, or None if the queue is empty.
        """
        try:
            return self.onboard_video_queue.get_nowait()
        except queue.Empty:
            return None

    def get_overhead_video_frame(self) -> Optional[np.ndarray]:
        """
        Retrieves the latest Pi-processed overhead video frame from its queue. Non-blocking.

        Returns:
            A NumPy ndarray representing the RGB video frame, or None if the queue is empty.
        """
        try:
            return self.overhead_video_queue.get_nowait()
        except queue.Empty:
            return None
            
    async def _connect_individual_ws(self, url: str, description: str) -> Optional[websockets.WebSocketClientProtocol]:
        """Helper to connect to an individual WebSocket with timeout and error handling."""
        logger.info(f"RobotClient: Attempting to connect to {description} at {url}...")
        try:
            # Use open_connection for more control over timeout
            ws = await asyncio.wait_for(
                websockets.connect(url, ping_interval=20, ping_timeout=20, max_size=None), # max_size=None for video
                timeout=5.0  # Connection timeout
            )
            logger.info(f"RobotClient: Successfully connected to {description}.")
            return ws
        except asyncio.TimeoutError:
            logger.warning(f"RobotClient: Timeout connecting to {description} at {url}.")
        except websockets.exceptions.InvalidURI:
            logger.error(f"RobotClient: Invalid URI for {description}: {url}")
        except websockets.exceptions.ConnectionClosedError as e:
            logger.error(f"RobotClient: Connection to {description} closed during handshake: {e}")
        except ConnectionRefusedError:
            logger.warning(f"RobotClient: Connection refused for {description} at {url}.")
        except OSError as e: # Catches errors like [Errno 61] Connection refused or host down
             logger.warning(f"RobotClient: OS error connecting to {description} at {url}: {e}")
        except Exception as e:
            logger.error(f"RobotClient: Unexpected error connecting to {description} at {url}: {e}", exc_info=True)
        return None

    async def _connect_all_websockets(self):
        """
        Attempts to establish all WebSocket connections (command, onboard video, overhead video).
        This coroutine runs in the client's asyncio event loop.
        It sets the respective `_connected` flags and updates `is_connected`.
        """
        # Attempt to connect command WebSocket
        if not self.command_connected:
            cmd_ws_url = f"ws://{self.host}:{self.command_port}"
            self.command_ws = await self._connect_individual_ws(cmd_ws_url, "Command Server")
            self.command_connected = self.command_ws is not None and self.command_ws.open

        # Attempt to connect onboard video WebSocket
        if not self.onboard_video_connected:
            onboard_vid_ws_url = f"ws://{self.host}:{self.onboard_video_port}"
            self.onboard_video_ws = await self._connect_individual_ws(onboard_vid_ws_url, "Onboard Video Server")
            if self.onboard_video_ws and self.onboard_video_ws.open:
            self.onboard_video_connected = True
                if self.onboard_video_receive_task and not self.onboard_video_receive_task.done():
                    self.onboard_video_receive_task.cancel() # Cancel old task if any
                self.onboard_video_receive_task = asyncio.create_task(
                    self._receive_video_stream(self.onboard_video_ws, self.onboard_video_queue, "Onboard"),
                    name="OnboardVideoReceiveTask"
                )
            else:
                self.onboard_video_connected = False


        # Attempt to connect overhead video WebSocket
        if not self.overhead_video_connected:
            overhead_vid_ws_url = f"ws://{self.host}:{self.overhead_video_port}"
            self.overhead_video_ws = await self._connect_individual_ws(overhead_vid_ws_url, "Overhead Video Server")
            if self.overhead_video_ws and self.overhead_video_ws.open:
            self.overhead_video_connected = True
                if self.overhead_video_receive_task and not self.overhead_video_receive_task.done():
                    self.overhead_video_receive_task.cancel() # Cancel old task if any
                self.overhead_video_receive_task = asyncio.create_task(
                    self._receive_video_stream(self.overhead_video_ws, self.overhead_video_queue, "Overhead"),
                    name="OverheadVideoReceiveTask"
                )
            else:
                self.overhead_video_connected = False

        # Update overall connection status
        # For this client, command and onboard video are considered essential. Overhead is optional.
        self.is_connected = self.command_connected and self.onboard_video_connected
            if self.is_connected:
            status_msg = "RobotClient: Connected to Command and Onboard Video."
            if self.overhead_video_connected:
                status_msg += " Overhead Video also connected."
            else:
                status_msg += " Overhead Video FAILED to connect."
            logger.info(status_msg)
        else:
            logger.warning(
                "RobotClient: Failed to establish all essential connections. "
                f"Command: {self.command_connected}, OnboardVideo: {self.onboard_video_connected}, "
                f"OverheadVideo: {self.overhead_video_connected}."
            )
            
    async def _shutdown_all_client_resources(self):
        """
        Gracefully closes all WebSocket connections and cancels running asyncio tasks.
        This coroutine runs in the client's asyncio event loop.
        """
        logger.info("RobotClient: Shutting down all client resources (async)...")

        # Cancel video receiving tasks
        tasks_to_cancel = []
        if self.onboard_video_receive_task and not self.onboard_video_receive_task.done():
            tasks_to_cancel.append(self.onboard_video_receive_task)
        if self.overhead_video_receive_task and not self.overhead_video_receive_task.done():
            tasks_to_cancel.append(self.overhead_video_receive_task)

        if tasks_to_cancel:
            logger.debug(f"RobotClient: Cancelling {len(tasks_to_cancel)} video tasks...")
        for task in tasks_to_cancel:
                task_name = task.get_name() if hasattr(task, 'get_name') else 'UnknownVideoTask'
            task.cancel()
                try:
                    await task # Allow task to process cancellation
                except asyncio.CancelledError:
                    logger.info(f"RobotClient: Video task '{task_name}' cancelled successfully.")
                except Exception as e:
                    logger.error(f"RobotClient: Error during cancellation of task '{task_name}': {e}")
        
        self.onboard_video_receive_task = None
        self.overhead_video_receive_task = None

        # Close WebSocket connections
        ws_connections_map = {
            "Command": self.command_ws,
            "OnboardVideo": self.onboard_video_ws,
            "OverheadVideo": self.overhead_video_ws
        }
        close_tasks = []
        for name, ws in ws_connections_map.items():
            if ws and ws.open: # Check if ws exists and is open
                logger.debug(f"RobotClient: Closing {name} WebSocket...")
                close_tasks.append(asyncio.wait_for(ws.close(), timeout=2.0))
            # Reset connection state flags
            if name == "Command": self.command_connected = False
            elif name == "OnboardVideo": self.onboard_video_connected = False
            elif name == "OverheadVideo": self.overhead_video_connected = False

        if close_tasks:
            results = await asyncio.gather(*close_tasks, return_exceptions=True)
            for i, result in enumerate(results): # Assuming order matches append order
                # Find the name corresponding to this result (a bit clunky but works for few items)
                closed_ws_name = [n for n, w in ws_connections_map.items() if w and w.id == results[i]._protocol.id][0] if isinstance(result, websockets.WebSocketClientProtocol) else f"WS_{i+1}"
                if isinstance(result, asyncio.TimeoutError):
                    logger.warning(f"RobotClient: Timeout closing WebSocket for {closed_ws_name}.")
                elif isinstance(result, Exception):
                    logger.error(f"RobotClient: Error closing WebSocket for {closed_ws_name}: {result}")
                else:
                    logger.info(f"RobotClient: WebSocket for {closed_ws_name} closed.")
        
        # Reset WebSocket attributes
        self.command_ws = None
        self.onboard_video_ws = None
        self.overhead_video_ws = None
        
        self.is_connected = False # Overall connection status
        logger.info("RobotClient: All async client resources have been processed for shutdown.")
            
    async def _receive_video_stream(self, ws: websockets.WebSocketClientProtocol,
                                    video_frame_queue: queue.Queue, stream_name: str):
        """
        Receives video frames from a WebSocket, decodes them, and puts them into a queue.
        This coroutine runs in the client's asyncio event loop for each video stream.

        Args:
            ws: The WebSocket connection to receive frames from.
            video_frame_queue: The queue to put decoded frames into.
            stream_name: A descriptive name for the stream (e.g., "Onboard", "Overhead").
        """
        logger.info(f"RobotClient: {stream_name} video receiver task started for {ws.remote_address}.")
        try:
            while ws.open and not (self._stop_event and self._stop_event.is_set()):
                try:
                    # Wait for a message with a timeout
                    data_bytes: Union[bytes, str] = await asyncio.wait_for(ws.recv(), timeout=1.0)
                    if not isinstance(data_bytes, bytes):
                        logger.warning(f"RobotClient: {stream_name} received non-bytes data: {type(data_bytes)}. Skipping.")
                        continue

                    # Decode JPEG bytes to a NumPy array
                    nparr = np.frombuffer(data_bytes, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                    if frame is not None:
                        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) # Convert BGR (OpenCV default) to RGB
                        # Put the frame into the queue (non-blocking, drops if full)
                        try: 
                            if video_frame_queue.full(): # If queue is full, discard the oldest frame
                                video_frame_queue.get_nowait()
                            video_frame_queue.put_nowait(frame_rgb)
                        except queue.Full:
                             # Should not happen if we check .full() and get_nowait() first, but as safeguard:
                            logger.debug(f"RobotClient: {stream_name} video queue was unexpectedly full. Frame dropped.")
                        except Exception as q_e:
                            logger.error(f"RobotClient: Error putting frame to {stream_name} queue: {q_e}")
                    else:
                        logger.warning(f"RobotClient: {stream_name} failed to decode frame (imdecode returned None).")

                except asyncio.TimeoutError:
                    # No message received within the timeout, continue to check ws.open and stop_event
                    if not ws.open: # Double check if ws closed during timeout
                        logger.warning(f"RobotClient: {stream_name} WebSocket found closed after recv timeout.")
                        break
                    continue
                except websockets.exceptions.ConnectionClosedOK:
                    logger.info(f"RobotClient: {stream_name} video WebSocket closed cleanly by server.")
                    break
                except websockets.exceptions.ConnectionClosedError as e:
                    logger.warning(f"RobotClient: {stream_name} video WebSocket closed with error: {e}")
                    break
                except Exception as e:
                    logger.error(f"RobotClient: {stream_name} video receive error: {e}", exc_info=True)
                    break # Exit loop on other errors
        except asyncio.CancelledError:
            logger.info(f"RobotClient: {stream_name} video receive task explicitly cancelled.")
        finally:
            logger.info(f"RobotClient: {stream_name} video receiver task finished for {ws.remote_address}.")
            # Update connection status for this specific stream
            if stream_name == "Onboard": self.onboard_video_connected = False
            elif stream_name == "Overhead": self.overhead_video_connected = False
            
            # Re-evaluate overall connection status if an essential stream dropped
            if (stream_name == "Onboard" and not self.onboard_video_connected) or \
               (stream_name == "Command" and not self.command_connected): # Though command isn't handled here
                self.is_connected = self.command_connected and self.onboard_video_connected

                
    def _run_client_event_loop(self):
        """
        Target method for the client's dedicated network thread.
        Sets up a new asyncio event loop and runs the main client logic (`_run_main_management_loop_async`).
        Ensures cleanup of asyncio resources upon completion or error.
        """
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self._stop_event = asyncio.Event() # Asyncio event, tied to this loop

            logger.info("RobotClient: Asyncio event loop started in dedicated thread.")
            self.loop.run_until_complete(self._run_main_management_loop_async())

        except Exception as e:
            logger.error(f"RobotClient: Unhandled error in client event loop manager: {e}", exc_info=True)
        finally:
            logger.info("RobotClient: Client event loop manager finishing. Ensuring async resources are shut down.")
            if self.loop and self.loop.is_running():
                # Ensure all tasks and connections are shut down before closing the loop
                logger.debug("RobotClient: Scheduling final shutdown of async resources before closing loop.")
                shutdown_future = asyncio.run_coroutine_threadsafe(self._shutdown_all_client_resources(), self.loop)
                try:
                    shutdown_future.result(timeout=5.0) # Wait for shutdown to complete
                    logger.info("RobotClient: Async resources successfully shut down via _run_client_event_loop finally block.")
                except TimeoutError:
                    logger.error("RobotClient: Timeout waiting for _shutdown_all_client_resources in _run_client_event_loop.")
                except Exception as e_shutdown:
                    logger.error(f"RobotClient: Exception during _shutdown_all_client_resources in _run_client_event_loop: {e_shutdown}")
            
            # Close the asyncio loop
            if self.loop and not self.loop.is_closed():
                # Cancel all remaining tasks in the loop
                for task in asyncio.all_tasks(self.loop):
                    task.cancel()
                # Run loop one last time to process cancellations
                try:
                    self.loop.run_until_complete(asyncio.gather(*asyncio.all_tasks(self.loop), return_exceptions=True))
                except Exception as e_gather:
                     logger.error(f"RobotClient: Error during final task gathering: {e_gather}")
                self.loop.close()
                logger.info("RobotClient: Asyncio event loop closed.")
            
            # Reset flags as the loop is now fully stopped
            self._cleanup_resources_immediate() # Perform final non-async cleanup
            logger.info("RobotClient: Dedicated client thread _run_client_event_loop finished.")

    async def _run_main_management_loop_async(self):
        """
        The main asynchronous management loop for the client.
        Handles connection attempts, monitors connection health, and manages reconnection logic.
        This coroutine runs in the client's asyncio event loop until `_stop_event` is set.
        """
        retry_delay_seconds = 5 # Initial delay for reconnection attempts
        max_retry_delay_seconds = 60

        while not (self._stop_event and self._stop_event.is_set()):
                try:
                # If not connected, attempt to connect all WebSockets
                if not self.is_connected: # is_connected reflects essential connections
                    await self._connect_all_websockets() # This updates self.is_connected

                    if not self.is_connected:
                        logger.warning(f"RobotClient: Failed to establish all essential connections. Retrying in {retry_delay_seconds}s...")
                        await asyncio.sleep(retry_delay_seconds)
                        # Exponential backoff for retry delay
                        retry_delay_seconds = min(retry_delay_seconds * 1.5, max_retry_delay_seconds)
                        continue # Restart loop to try connecting again
                    else:
                        retry_delay_seconds = 5 # Reset retry delay on successful connection

                # If connected, monitor health and wait for stop signal
            if self.is_connected:
                try:
                        # Wait for the stop event with a timeout, allowing periodic checks
                    await asyncio.wait_for(self._stop_event.wait(), timeout=1.0) 
                        # If wait() returns, it means _stop_event was set.
                        logger.info("RobotClient: Stop event detected in main management loop.")
                        break # Exit the while loop
                except asyncio.TimeoutError:
                        # Timeout occurred, _stop_event not set. Check WebSocket health.
                        any_essential_ws_closed = False
                        if self.command_ws and self.command_ws.state != State.OPEN:
                            logger.warning("RobotClient: Command WebSocket found not OPEN. State: {self.command_ws.state}")
                        self.command_connected = False
                            any_essential_ws_closed = True
                        if self.onboard_video_ws and self.onboard_video_ws.state != State.OPEN:
                            logger.warning("RobotClient: Onboard Video WebSocket found not OPEN. State: {self.onboard_video_ws.state}")
                        self.onboard_video_connected = False
                            any_essential_ws_closed = True
                        # Non-essential, but log if it's closed
                        if self.overhead_video_ws and self.overhead_video_ws.state != State.OPEN:
                             logger.debug("RobotClient: Overhead Video WebSocket found not OPEN. State: {self.overhead_video_ws.state}")
                        self.overhead_video_connected = False


                        if any_essential_ws_closed:
                            logger.warning("RobotClient: One or more essential WebSockets found closed. Triggering reconnect logic.")
                            self.is_connected = False # This will trigger _connect_all_websockets in the next iteration
                            # Close remaining connections before attempting to reconnect all
                            await self._shutdown_all_client_resources() # Cleans up existing connections
                        continue # Continue monitoring

            except asyncio.CancelledError:
                logger.info("RobotClient: Main management loop task cancelled.")
                 break
            except Exception as e:
                logger.error(f"RobotClient: Unhandled exception in main management loop: {e}. Retrying connection sequence.", exc_info=True)
                self.is_connected = False # Assume connections are bad
                await self._shutdown_all_client_resources() # Attempt to clean up before retrying
                await asyncio.sleep(retry_delay_seconds) # Wait before retrying
                retry_delay_seconds = min(retry_delay_seconds * 1.5, max_retry_delay_seconds)


        logger.info("RobotClient: Main async management loop finished. Initiating final resource shutdown.")
        await self._shutdown_all_client_resources() # Ensure everything is closed on exit 