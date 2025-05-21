import asyncio
import logging
import threading
from typing import Optional, Set
import numpy as np
import cv2
from websockets.server import serve as ws_serve
from websockets.exceptions import ConnectionClosed

logger = logging.getLogger(__name__)

class OverheadStreamer:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.server = None
        self.clients: Set[asyncio.Queue] = set()
        self.running = False
        self._stop_event: Optional[asyncio.Event] = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.external_frame: Optional[np.ndarray] = None
        self.external_frame_lock = threading.Lock()
        self.jpeg_quality = 50 # Reduced JPEG quality from 75 to 50

    def set_frame(self, frame: np.ndarray):
        """Sets the current frame to be streamed."""
        with self.external_frame_lock:
            self.external_frame = frame.copy()
        # Notify worker to send the frame
        for q in self.clients:
            try:
                # Non-blocking put, or handle queue full if it can happen
                # For simplicity, assuming queue won't be full if clients are responsive
                q.put_nowait(self.external_frame) 
            except asyncio.QueueFull:
                logger.warning(f"OverheadStreamer: Client queue (id: {id(q)}, maxsize: {q.maxsize}) full. Frame dropped for this client.")
            except Exception as e:
                logger.error(f"OverheadStreamer: Error putting frame to queue for client {q}: {e}")


    async def _handle_client(self, websocket, path):
        logger.info(f"OverheadStreamer: New client connected from {websocket.remote_address} to {self.host}:{self.port}")
        client_queue = asyncio.Queue(maxsize=15) # Increased buffer size from 5 to 15
        self.clients.add(client_queue)
        try:
            while self.running:
                try:
                    frame_to_send = await asyncio.wait_for(client_queue.get(), timeout=1.0)
                    if frame_to_send is None: # Could be a signal to stop, though not used here
                        continue

                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                    result, encoded_jpeg = cv2.imencode('.jpg', frame_to_send, encode_param)
                    
                    if not result:
                        logger.error("OverheadStreamer: Failed to encode frame to JPEG")
                        continue
                    
                    await websocket.send(encoded_jpeg.tobytes())
                    client_queue.task_done()

                except asyncio.TimeoutError:
                    # Check if websocket is still alive or if we should break
                    if not websocket.open:
                        logger.info("OverheadStreamer: Client websocket closed during timeout check.")
                        break
                    continue # Normal if no new frame within timeout
                except ConnectionClosed:
                    logger.info(f"OverheadStreamer: Client {websocket.remote_address} connection closed.")
                    break
                except Exception as e:
                    logger.error(f"OverheadStreamer: Error in client handler for {websocket.remote_address}: {e}", exc_info=True)
                    break
        finally:
            logger.info(f"OverheadStreamer: Client {websocket.remote_address} disconnected.")
            self.clients.remove(client_queue)


    async def start_server_async(self):
        """Start the WebSocket server (async part)"""
        self._stop_event = asyncio.Event()
        async with ws_serve(
            self._handle_client,
            self.host,
            self.port,
            ping_interval=20,
            ping_timeout=20,
            max_size=None 
        ) as server:
            self.server = server
            self.running = True
            logger.info(f"OverheadStreamer WebSocket server started on {self.host}:{self.port}")
            await self._stop_event.wait()
            logger.info("OverheadStreamer stop event received, proceeding to resource shutdown.")
            await self._shutdown_server_resources()
            
    def start(self):
        """Start the streamer (called by ForkliftServer thread)"""
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self.start_server_async())
        except Exception as e:
            logger.error(f"OverheadStreamer: Error starting: {e}", exc_info=True)
            # self.cleanup() # Ensure cleanup if start fails
        finally:
            if self.loop and not self.loop.is_closed():
                logger.info("OverheadStreamer: Closing event loop in start() finally.")
                self.loop.close()
            logger.info("OverheadStreamer: start() method finished.")

    async def _shutdown_server_resources(self):
        logger.info(f"OverheadStreamer ({self.host}:{self.port}) shutting down resources...")
        # Clients are identified by their queues; just clearing the set
        self.clients.clear()

        if self.server:
            self.server.close()
            try:
                await asyncio.wait_for(self.server.wait_closed(), timeout=5.0)
                logger.info(f"OverheadStreamer ({self.host}:{self.port}) WebSocket server has been closed.")
            except asyncio.TimeoutError:
                logger.warning(f"OverheadStreamer ({self.host}:{self.port}) timeout waiting for WebSocket server to close.")
        

    def stop(self):
        """Stop the streamer (called by ForkliftServer)"""
        if not self.running and not (self.loop and self.loop.is_running()):
            logger.info(f"OverheadStreamer: stop() called, but server or loop not active.")
            return

        logger.info(f"OverheadStreamer: stop() called. Signaling stop event.")
        self.running = False
        
        if self.loop and self._stop_event and not self._stop_event.is_set():
            self.loop.call_soon_threadsafe(self._stop_event.set)
        else:
            logger.info("OverheadStreamer: Loop or _stop_event not available or already set.")

    def cleanup(self):
        """Clean up resources (called by ForkliftServer)"""
        logger.info(f"OverheadStreamer: Cleaning up...")
        # Loop cleanup is handled in start() finally or stop() indirectly
        if self.loop and not self.loop.is_closed():
             logger.warning("OverheadStreamer: Event loop was not closed; attempting close in cleanup.")
             if self.loop.is_running():
                 self.loop.call_soon_threadsafe(self.loop.stop)
                 # Give it a moment to process the stop
                 # This is tricky from a different thread; direct closing might be abrupt
                 # Consider joining the thread that runs self.loop if possible from ForkliftServer
             time.sleep(0.2) # Short delay
             if not self.loop.is_closed(): # Check again
                self.loop.close()
             logger.info("OverheadStreamer: Event loop closed in cleanup.")
        self.loop = None
        logger.info(f"OverheadStreamer: Cleanup complete.") 