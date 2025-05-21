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
    def __init__(self, host: str = SERVER_HOST, 
                 command_port: int = COMMAND_PORT, 
                 onboard_video_port: int = VIDEO_PORT, 
                 overhead_video_port: int = OVERHEAD_VIDEO_PORT):
        self.host = host
        self.command_port = command_port
        self.onboard_video_port = onboard_video_port
        self.overhead_video_port = overhead_video_port
        
        self.command_ws: Optional[websockets.WebSocketClientProtocol] = None
        self.onboard_video_ws: Optional[websockets.WebSocketClientProtocol] = None
        self.overhead_video_ws: Optional[websockets.WebSocketClientProtocol] = None
        
        self.is_connected = False
        self.command_connected = False
        self.onboard_video_connected = False
        self.overhead_video_connected = False

        self.onboard_video_queue = queue.Queue(maxsize=10)
        self.overhead_video_queue = queue.Queue(maxsize=10)
        
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.thread: Optional[threading.Thread] = None
        self._stop_event: Optional[asyncio.Event] = None
        
        self.onboard_video_receive_task: Optional[asyncio.Task] = None
        self.overhead_video_receive_task: Optional[asyncio.Task] = None
        
        logger.info(f"Initializing RobotClient with: HOST={self.host}, COMMAND_PORT={self.command_port}, ONBOARD_VIDEO_PORT={self.onboard_video_port}, OVERHEAD_VIDEO_PORT={self.overhead_video_port}")
        
    def connect(self):
        if self.is_connected:
            logger.warning("Already connected or connection attempt in progress.")
            return
        self.thread = threading.Thread(target=self._run_client, name="RobotClientThread")
        self.thread.daemon = True
        self.thread.start()
            
    def disconnect(self):
        if not (self.thread and self.thread.is_alive()):
            logger.warning("Not connected or thread not running.")
            return
            
        logger.info("Disconnecting... Signaling stop event.")
        if self.loop and self._stop_event and not self._stop_event.is_set():
            self.loop.call_soon_threadsafe(self._stop_event.set)
        else:
            logger.warning("Cannot signal stop event: loop or event not initialized or already set.")
        
        if self.thread:
            self.thread.join(timeout=7.0)
            if self.thread.is_alive():
                logger.warning("Client thread did not terminate cleanly within timeout.")
        
        self.is_connected = False
        self.command_connected = False
        self.onboard_video_connected = False
        self.overhead_video_connected = False
        self.loop = None
        self.thread = None
        self.onboard_video_receive_task = None
        self.overhead_video_receive_task = None
        logger.info("Client disconnect process finished.")
            
    def send_command(self, command: str, value: Optional[Any] = None):
        if not self.command_connected or not self.loop or not self.command_ws:
            logger.warning("Command WS not connected, cannot send command.")
            return
        message = {"command": command, "value": value}
        try:
            asyncio.run_coroutine_threadsafe(self._send_command_async(json.dumps(message)), self.loop)
        except Exception as e:
            logger.error(f"Error scheduling send_command: {e}")

    async def _send_command_async(self, message: str):
        if self.command_ws and self.command_connected:
            try: await self.command_ws.send(message)
            except Exception as e: logger.error(f"Error sending command: {e}")
        else: logger.warning("Command WS not available for sending.")
            
    def get_onboard_video_frame(self) -> Optional[np.ndarray]:
        try: return self.onboard_video_queue.get_nowait()
        except queue.Empty: return None

    def get_overhead_video_frame(self) -> Optional[np.ndarray]:
        try: return self.overhead_video_queue.get_nowait()
        except queue.Empty: return None
            
    async def _connect_all(self):
        try:
            logger.info(f"Connecting to command server at ws://{self.host}:{self.command_port}")
            self.command_ws = await websockets.connect(f"ws://{self.host}:{self.command_port}", ping_interval=20, ping_timeout=20)
            self.command_connected = True
            logger.info("Connected to command server.")

            logger.info(f"Connecting to onboard video server at ws://{self.host}:{self.onboard_video_port}")
            self.onboard_video_ws = await websockets.connect(f"ws://{self.host}:{self.onboard_video_port}", ping_interval=20, ping_timeout=20, max_size=None)
            self.onboard_video_connected = True
            logger.info("Connected to onboard video server.")
            self.onboard_video_receive_task = asyncio.create_task(self._receive_video_stream(self.onboard_video_ws, self.onboard_video_queue, "Onboard"), name="OnboardVideoReceiveTask")

            logger.info(f"Connecting to overhead video server at ws://{self.host}:{self.overhead_video_port}")
            self.overhead_video_ws = await websockets.connect(f"ws://{self.host}:{self.overhead_video_port}", ping_interval=20, ping_timeout=20, max_size=None)
            self.overhead_video_connected = True
            logger.info("Connected to overhead video server.")
            self.overhead_video_receive_task = asyncio.create_task(self._receive_video_stream(self.overhead_video_ws, self.overhead_video_queue, "Overhead"), name="OverheadVideoReceiveTask")
            
            self.is_connected = self.command_connected and self.onboard_video_connected and self.overhead_video_connected
            if self.is_connected:
                logger.info("Successfully connected to all servers (Command, Onboard Video, Overhead Video).")
            else:
                logger.warning("Partial connection achieved. Check logs for details.")

        except Exception as e:
            logger.error(f"Connection error during _connect_all: {e}", exc_info=True)
            await self._shutdown_client_resources()
            
    async def _shutdown_client_resources(self):
        logger.info("Shutting down client resources...")
        tasks_to_cancel = []
        if self.onboard_video_receive_task and not self.onboard_video_receive_task.done():
            tasks_to_cancel.append(self.onboard_video_receive_task)
        if self.overhead_video_receive_task and not self.overhead_video_receive_task.done():
            tasks_to_cancel.append(self.overhead_video_receive_task)

        for task in tasks_to_cancel:
            task_name = task.get_name() if hasattr(task, 'get_name') else 'UnknownTask'
            task.cancel()
            try: await task
            except asyncio.CancelledError: logger.info(f"Video task {task_name} cancelled.")
            except Exception as e: logger.error(f"Error cancelling task {task_name}: {e}")
        
        self.onboard_video_receive_task = None
        self.overhead_video_receive_task = None

        ws_connections = []
        if self.command_ws: ws_connections.append(self.command_ws)
        if self.onboard_video_ws: ws_connections.append(self.onboard_video_ws)
        if self.overhead_video_ws: ws_connections.append(self.overhead_video_ws)

        close_tasks = [asyncio.wait_for(ws.close(), timeout=2.0) for ws in ws_connections if ws and not ws.closed]
        if close_tasks:
            results = await asyncio.gather(*close_tasks, return_exceptions=True)
            for i, result in enumerate(results):
                if isinstance(result, asyncio.TimeoutError): logger.warning(f"Timeout closing websocket {i+1}.")
                elif isinstance(result, Exception): logger.error(f"Error closing websocket {i+1}: {result}")
                else: logger.info(f"Websocket {i+1} closed.")
        
        self.command_ws = None
        self.onboard_video_ws = None
        self.overhead_video_ws = None
        self.is_connected = False
        self.command_connected = False
        self.onboard_video_connected = False
        self.overhead_video_connected = False
        logger.info("Client resources shut down.")
            
    async def _receive_video_stream(self, ws: websockets.WebSocketClientProtocol, video_q: queue.Queue, stream_name: str):
        logger.info(f"{stream_name} video receiver task started.")
        try:
            while not (self._stop_event and self._stop_event.is_set()):
                try:
                    if not ws or ws.closed:
                        logger.warning(f"{stream_name} websocket is None or closed, stopping video reception.")
                        break
                    data = await asyncio.wait_for(ws.recv(), timeout=1.0)
                    nparr = np.frombuffer(data, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    if frame is not None:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        try: 
                            if video_q.full(): video_q.get_nowait()
                            video_q.put_nowait(frame)
                        except queue.Full: pass
                except asyncio.TimeoutError: continue 
                except websockets.exceptions.ConnectionClosed:
                    logger.warning(f"{stream_name} video connection closed during recv.")
                    break
                except Exception as e:
                    logger.error(f"{stream_name} video receive error: {e}", exc_info=True)
                    break
        except asyncio.CancelledError:
            logger.info(f"{stream_name} video receive task explicitly cancelled.")
        finally:
            logger.info(f"{stream_name} video receiver task finished.")
            if stream_name == "Onboard": self.onboard_video_connected = False
            elif stream_name == "Overhead": self.overhead_video_connected = False
            if not self.onboard_video_connected and not self.overhead_video_connected and not self.command_connected:
                self.is_connected = False
                
    def _run_client(self):
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self._stop_event = asyncio.Event()
            self.loop.run_until_complete(self._run_main_async())
        except Exception as e:
            logger.error(f"Error in client thread main execution: {e}", exc_info=True)
        finally:
            logger.info("Client thread's run_client finishing. Ensuring resources are shut down before closing loop.")
            if self.loop and self.loop.is_running():
                shutdown_future = asyncio.run_coroutine_threadsafe(self._shutdown_client_resources(), self.loop)
                try:
                    shutdown_future.result(timeout=5.0)
                    logger.info("Client resources successfully shut down via _run_client finally.")
                except TimeoutError:
                    logger.error("Timeout waiting for _shutdown_client_resources to complete in _run_client.")
                except Exception as e:
                    logger.error(f"Exception during _shutdown_client_resources in _run_client: {e}")
            
            if self.loop and not self.loop.is_closed():
                self.loop.close()
                logger.info("Client asyncio event loop closed.")
            self.is_connected = False
            self.command_connected = False
            self.onboard_video_connected = False
            self.overhead_video_connected = False
            logger.info("Client thread _run_client finished.")

    async def _run_main_async(self):
        retry_delay = 5
        while not (self._stop_event and self._stop_event.is_set()):
            if not self.is_connected:
                try:
                    await self._connect_all()
                    if not self.is_connected:
                        logger.warning(f"Failed to establish all connections. Retrying in {retry_delay}s...")
                        await asyncio.sleep(retry_delay)
                        continue
                except Exception as e:
                    logger.error(f"Unhandled exception during _connect_all: {e}. Retrying in {retry_delay}s...")
                    await self._shutdown_client_resources()
                    await asyncio.sleep(retry_delay)
                    continue
            
            if self.is_connected:
                try:
                    await asyncio.wait_for(self._stop_event.wait(), timeout=1.0) 
                except asyncio.TimeoutError:
                    any_ws_closed = False
                    if self.command_ws and self.command_ws.closed:
                        logger.warning("Command WebSocket found closed.")
                        self.command_connected = False
                        any_ws_closed = True
                    if self.onboard_video_ws and self.onboard_video_ws.closed:
                        logger.warning("Onboard Video WebSocket found closed.")
                        self.onboard_video_connected = False
                        any_ws_closed = True
                    if self.overhead_video_ws and self.overhead_video_ws.closed:
                        logger.warning("Overhead Video WebSocket found closed.")
                        self.overhead_video_connected = False
                        any_ws_closed = True
                    
                    if any_ws_closed:
                        logger.warning("One or more WebSockets found closed. Triggering full reconnect logic.")
                        self.is_connected = False
                    continue
            
            if self._stop_event.is_set():
                 logger.info("Client stop event received in _run_main_async.")
                 break
        
        logger.info("Client main async execution finished, initiating final resource shutdown.")
        await self._shutdown_client_resources() 