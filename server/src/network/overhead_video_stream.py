import asyncio
import logging
import threading
from typing import Optional, Set, Any
import numpy as np
import cv2
from websockets.server import serve as ws_serve
from websockets.exceptions import ConnectionClosed

logger = logging.getLogger(__name__)

class OverheadStreamer:
    """Streams video frames received from an external source (e.g., processed
    overhead camera feed) to connected WebSocket clients.

    This class acts as a WebSocket server. It receives NumPy array frames via
    its `set_frame` method. Each connected client has its own queue, and frames
    are pushed to these queues. A dedicated asyncio task for each client then
    retrieves frames from its queue, encodes them as JPEG, and sends them over
    the WebSocket connection.
    """
    def __init__(self, host: str, port: int):
        """Initializes the OverheadStreamer.

        Args:
            host: The network host to bind the WebSocket server to.
            port: The port for the WebSocket video streaming server.
        """
        self.host = host
        self.port = port
        self.server: Optional[Any] = None
        self.clients: Set[asyncio.Queue[Optional[np.ndarray]]] = set()
        self._running_WebSocket_server = False
        self._stop_event: Optional[asyncio.Event] = None
        self.async_loop: Optional[asyncio.AbstractEventLoop] = None
        self.server_thread: Optional[threading.Thread] = None
        self.external_frame: Optional[np.ndarray] = None
        self.external_frame_lock = threading.Lock()
        self.jpeg_quality = 50

    def set_frame(self, frame: np.ndarray):
        """Accepts a new video frame and distributes it to all client queues.

        This method is thread-safe. It should be called by the source providing
        the overhead video frames (e.g., after processing by OverheadLocalizer).

        Args:
            frame: The video frame (as a NumPy ndarray) to be streamed.
        """
        with self.external_frame_lock:
            self.external_frame = frame.copy()
        for client_q in list(self.clients):
            try:
                client_q.put_nowait(frame.copy())
            except asyncio.QueueFull:
                logger.warning(f"OverheadStreamer: Client queue full for a client. Frame dropped for this client.")
            except Exception as e:
                logger.error(f"OverheadStreamer: Error putting frame to a client queue: {e}")

    async def _handle_client_websocket(self, websocket: Any, path: str):
        """Handles an individual WebSocket client connection.

        A new instance of this coroutine is run for each client that connects.
        It creates a dedicated asyncio.Queue for the client, adds it to `self.clients`,
        and then enters a loop to get frames from this queue, encode them, and send them.

        Args:
            websocket: The WebSocket connection object for this client.
            path: The path of the WebSocket connection (unused).
        """
        remote_addr = websocket.remote_address
        logger.info(f"OverheadStreamer: New client connected from {remote_addr} on path '{path}'.")
        client_queue: asyncio.Queue[Optional[np.ndarray]] = asyncio.Queue(maxsize=10)
        self.clients.add(client_queue)
        try:
            while self._running_WebSocket_server:
                frame_to_send = None
                try:
                    frame_to_send = await asyncio.wait_for(client_queue.get(), timeout=1.0)
                except asyncio.TimeoutError:
                    if not websocket.open:
                        logger.info(f"OverheadStreamer: Client {remote_addr} websocket closed during queue timeout.")
                        break
                        continue
                
                if frame_to_send is None:
                    logger.info(f"OverheadStreamer: Received None from queue for client {remote_addr}. Ending handler.")
                    break

                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                    result, encoded_jpeg = cv2.imencode('.jpg', frame_to_send, encode_param)
                    
                    if not result:
                    logger.error("OverheadStreamer: Failed to encode frame to JPEG. Skipping send.")
                    client_queue.task_done()
                        continue
                    
                    await websocket.send(encoded_jpeg.tobytes())
                    client_queue.task_done()

                except ConnectionClosed:
            logger.info(f"OverheadStreamer: Client {remote_addr} connection closed gracefully.")
                except Exception as e:
            logger.error(f"OverheadStreamer: Error in client handler for {remote_addr}: {e}", exc_info=True)
        finally:
            logger.info(f"OverheadStreamer: Client {remote_addr} disconnecting. Removing its queue.")
            self.clients.discard(client_queue)
            while not client_queue.empty():
                try:
                    client_queue.get_nowait()
                    client_queue.task_done()
                except asyncio.QueueEmpty:
                    break
                except Exception:
                    pass

    async def _run_server_main_task(self):
        """The main asynchronous task that runs the WebSocket server using `ws_serve`."""
        self._stop_event = asyncio.Event()
        
        async with ws_serve(
            self._handle_client_websocket,
            self.host,
            self.port,
            ping_interval=20,
            ping_timeout=20,
            reuse_address=True
        ) as server_instance:
            self.server = server_instance
            self._running_WebSocket_server = True
            logger.info(f"OverheadStreamer: WebSocket server started and listening on {self.host}:{self.port}")
            
            await self._stop_event.wait()
            
            logger.info("OverheadStreamer: Stop event received. Shutting down WebSocket server.")

        self._running_WebSocket_server = False
        logger.info("OverheadStreamer: WebSocket server has shut down.")
            
    def start(self):
        """Starts the OverheadStreamer in a new thread with its own asyncio event loop.
        Initializes and runs the WebSocket server.
        """
        if self.server_thread and self.server_thread.is_alive():
            logger.warning("OverheadStreamer: Start called, but server thread is already running.")
            return

        logger.info("OverheadStreamer: Starting server thread and asyncio event loop...")
        self.async_loop = asyncio.new_event_loop()
        
        def loop_runner():
            asyncio.set_event_loop(self.async_loop)
            try:
                self.async_loop.run_until_complete(self._run_server_main_task())
            except KeyboardInterrupt:
                logger.info("OverheadStreamer: KeyboardInterrupt in server loop.")
            except Exception as e:
                logger.error(f"OverheadStreamer: Exception in server_thread's run_loop: {e}", exc_info=True)
            finally:
                logger.info("OverheadStreamer: Asyncio loop in server_thread ended.")
                if self.async_loop.is_running():
                    self.async_loop.call_soon_threadsafe(self.async_loop.stop)
        
        self.server_thread = threading.Thread(target=loop_runner, daemon=True, name="OverheadStreamerThread")
        self.server_thread.start()
        logger.info("OverheadStreamer: Server thread started.")

    def stop(self):
        """Signals the OverheadStreamer to stop its operations gracefully."""
        logger.info("OverheadStreamer: Stop requested.")
        if not self._running_WebSocket_server and not (self.server_thread and self.server_thread.is_alive()):
            logger.info("OverheadStreamer: Stop called but streamer not considered active.")
            self.cleanup()
            return

        self._running_WebSocket_server = False
        
        if self.async_loop and self._stop_event and not self._stop_event.is_set():
            logger.info("OverheadStreamer: Setting stop_event for the server's asyncio loop.")
            self.async_loop.call_soon_threadsafe(self._stop_event.set)
        else:
            logger.warning("OverheadStreamer: Cannot signal stop_event (loop or event not available/already set).")
        
        if self.server_thread and self.server_thread.is_alive():
            logger.info("OverheadStreamer: Waiting for server thread to join...")
            self.server_thread.join(timeout=7.0)
            if self.server_thread.is_alive():
                logger.warning("OverheadStreamer: Server thread did not join in time. It might be stuck.")
            else:
                logger.info("OverheadStreamer: Server thread joined.")
        self.server_thread = None
        
        self.cleanup()
        logger.info("OverheadStreamer: Stop sequence complete.")

    def cleanup(self):
        """Cleans up any held resources. Primarily ensures the asyncio loop is closed if still open.
        This is usually called as part of the stop() sequence.
        """
        logger.info(f"OverheadStreamer: Cleanup initiated.")
        self._running_WebSocket_server = False

        self.clients.clear()

        if self.async_loop:
            if self.async_loop.is_running():
                logger.info("OverheadStreamer cleanup: Asyncio loop is still running. Requesting stop.")
                self.async_loop.call_soon_threadsafe(self.async_loop.stop)
            
            if not self.async_loop.is_closed():
                logger.info("OverheadStreamer cleanup: Closing asyncio loop.")
            else:
                logger.info("OverheadStreamer cleanup: Asyncio loop was already closed.")
        self.async_loop = None
        logger.info(f"OverheadStreamer: Cleanup finished.") 