import asyncio
import websockets
import json
import cv2
import numpy as np
from typing import Optional, Tuple, Dict, Any
import threading
import queue
import logging
import os
import sys
import time # Added for sleep in cleanup
from colorama import Fore, Style, init as colorama_init

# Add the src directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from src.config import SERVER_HOST, COMMAND_PORT, VIDEO_PORT

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
    def __init__(self, host: str = SERVER_HOST, command_port: int = COMMAND_PORT, video_port: int = VIDEO_PORT):
        self.host = host
        self.command_port = command_port
        self.video_port = video_port
        self.command_ws: Optional[websockets.WebSocketClientProtocol] = None
        self.video_ws: Optional[websockets.WebSocketClientProtocol] = None
        self.is_connected = False
        self.video_queue = queue.Queue(maxsize=10)
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.thread: Optional[threading.Thread] = None
        self._stop_event: Optional[asyncio.Event] = None # Will be asyncio.Event
        self.video_receive_task: Optional[asyncio.Task] = None # To store the video task
        
        logger.info(f"Initializing RobotClient with: HOST={self.host}, COMMAND_PORT={self.command_port}, VIDEO_PORT={self.video_port}")
        
    def connect(self):
        if self.is_connected:
            logger.warning("Already connected")
            return
        # Ensure stop event is cleared if reusing client instance (though usually new instance is created)
        # self._stop_event is created within the loop now
        self.thread = threading.Thread(target=self._run_client, name="RobotClientThread")
        self.thread.daemon = True # Allow main thread to exit even if this fails (though cleanup is better)
        self.thread.start()
        
    def disconnect(self):
        if not self.is_connected and not (self.thread and self.thread.is_alive()):
            logger.warning("Not connected or thread not running")
            return
            
        logger.info("Disconnecting... Signaling stop event.")
        if self.loop and self._stop_event and not self._stop_event.is_set():
            self.loop.call_soon_threadsafe(self._stop_event.set)
        else:
            logger.warning("Cannot signal stop event: loop or event not initialized or already set.")
        
        # Wait for the thread to finish its cleanup
        if self.thread:
            self.thread.join(timeout=5.0)
            if self.thread.is_alive():
                logger.warning("Client thread did not terminate cleanly within timeout.")
            else:
                logger.info("Client thread terminated cleanly.")
        
        # Ensure flags are reset even if thread join timed out
        self.is_connected = False 
        self.loop = None
        self.thread = None
        self.video_receive_task = None
        logger.info("Client disconnect process complete.")
            
    def send_command(self, command: str, value: Optional[Any] = None):
        if not self.is_connected or not self.loop or not self.command_ws:
            logger.warning("Not connected, cannot send command")
            return
            
        message = {
            "command": command,
            "value": value
        }
        try:
            # Schedule the send operation on the client's event loop
            asyncio.run_coroutine_threadsafe(
                self._send_command(json.dumps(message)),
                self.loop
            )
        except Exception as e:
            logger.error(f"Error scheduling send_command: {e}")
        
    def get_video_frame(self) -> Optional[np.ndarray]:
        """Get the latest video frame"""
        try:
            return self.video_queue.get_nowait()
        except queue.Empty:
            return None
            
    async def _connect(self):
        try:
            logger.info(f"Connecting to command server at ws://{self.host}:{self.command_port}")
            self.command_ws = await websockets.connect(
                f"ws://{self.host}:{self.command_port}",
                ping_interval=20, ping_timeout=20
            )
            logger.info(f"Connecting to video server at ws://{self.host}:{self.video_port}")
            self.video_ws = await websockets.connect(
                f"ws://{self.host}:{self.video_port}",
                ping_interval=20, ping_timeout=20, max_size=None # Allow larger frames
            )
            self.is_connected = True
            logger.info("Successfully connected to both servers")
            # Start video receiver task and store it
            self.video_receive_task = asyncio.create_task(self._receive_video())
        except Exception as e:
            logger.error(f"Connection error: {e}", exc_info=True)
            self.is_connected = False
            # Ensure cleanup if connection fails partially
            await self._shutdown_client_resources()
            raise # Re-raise exception to stop the _run_client loop
            
    async def _shutdown_client_resources(self):
        logger.info("Shutting down client resources...")
        # Cancel video task
        if self.video_receive_task and not self.video_receive_task.done():
            self.video_receive_task.cancel()
            try:
                await self.video_receive_task # Wait for cancellation
            except asyncio.CancelledError:
                logger.info("Video receive task cancelled.")
            except Exception as e:
                logger.error(f"Error during video task cancellation/await: {e}")
        self.video_receive_task = None

        # Close websockets
        tasks = []
        if self.command_ws and self.command_ws.open:
            tasks.append(asyncio.wait_for(self.command_ws.close(), timeout=2.0))
        if self.video_ws and self.video_ws.open:
            tasks.append(asyncio.wait_for(self.video_ws.close(), timeout=2.0))
        
        if tasks:
            results = await asyncio.gather(*tasks, return_exceptions=True)
            for i, result in enumerate(results):
                if isinstance(result, asyncio.TimeoutError):
                    logger.warning(f"Timeout closing websocket {i+1}.")
                elif isinstance(result, Exception):
                    logger.error(f"Error closing websocket {i+1}: {result}")
                else:
                    logger.info(f"Websocket {i+1} closed gracefully.")
        
        self.command_ws = None
        self.video_ws = None
        self.is_connected = False # Ensure connection status is updated
        logger.info("Client resources shut down.")
            
    async def _send_command(self, message: str):
        """Send a command through the command websocket"""
        if self.command_ws:
            try:
                await self.command_ws.send(message)
            except Exception as e:
                logger.error(f"Error sending command: {e}")
                raise
            
    async def _receive_video(self):
        logger.info("Video receiver task started.")
        try:
            while self.is_connected and not (self._stop_event and self._stop_event.is_set()):
                try:
                    if not self.video_ws or not self.video_ws.open:
                        logger.warning("Video websocket not available or closed, stopping video reception.")
                        break
                    # Use wait_for to add a timeout, making it responsive to cancellation
                    data = await asyncio.wait_for(self.video_ws.recv(), timeout=1.0)
                    nparr = np.frombuffer(data, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    if frame is not None:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        try: self.video_queue.put_nowait(frame)
                        except queue.Full:
                            try: 
                                self.video_queue.get_nowait()
                                self.video_queue.put_nowait(frame)
                            except queue.Empty: pass
                except asyncio.TimeoutError: # Timeout on recv is normal, just continue loop check
                    continue 
                except websockets.exceptions.ConnectionClosed:
                    logger.warning("Video connection closed by server.")
                    break
                except Exception as e:
                    logger.error(f"Video receive error: {e}", exc_info=True)
                    break
        except asyncio.CancelledError:
            logger.info("Video receive task explicitly cancelled.")
        finally:
             logger.info("Video receiver task finished.")
             self.is_connected = False # Ensure status reflects reality
                
    def _run_client(self):
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self._stop_event = asyncio.Event() # Create event within the loop
            # Run connect and wait for stop signal
            self.loop.run_until_complete(self._run_main_async())
        except Exception as e:
            logger.error(f"Error in client thread main execution: {e}", exc_info=True)
        finally:
            logger.info("Client thread's run_client finishing. Closing loop.")
            if self.loop:
                # Ensure all tasks are complete before closing loop
                # This requires running the loop a bit more to process cancellations/closures
                try:
                    all_tasks = asyncio.all_tasks(self.loop)
                    tasks_to_wait_for = [t for t in all_tasks if not t.done()]
                    if tasks_to_wait_for:
                        self.loop.run_until_complete(asyncio.gather(*tasks_to_wait_for, return_exceptions=True))
                except Exception as e:
                    logger.error(f"Error during final task gathering in _run_client: {e}")
                
                if not self.loop.is_closed():
                    self.loop.close()
                    logger.info("Client asyncio event loop closed.")
            self.is_connected = False # Final state update
            logger.info("Client thread _run_client finished.")

    async def _run_main_async(self):
        """Main async part run by the client thread's loop"""
        try:
            await self._connect()
            if self.is_connected:
                await self._stop_event.wait()
                logger.info("Client stop event received.")
            else:
                logger.warning("Connection failed, not waiting for stop event.")
        except Exception as e:
            logger.error(f"Exception during client main async execution: {e}", exc_info=True)
        finally:
            logger.info("Client main async execution finished, shutting down resources.")
            await self._shutdown_client_resources() 