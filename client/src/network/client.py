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

# Add the src directory to the Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from src.config import SERVER_HOST, COMMAND_PORT, VIDEO_PORT

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RobotClient:
    def __init__(self, host: str = SERVER_HOST, command_port: int = COMMAND_PORT, video_port: int = VIDEO_PORT):
        self.host = host
        self.command_port = command_port
        self.video_port = video_port
        self.command_ws = None
        self.video_ws = None
        self.is_connected = False
        self.video_queue = queue.Queue(maxsize=10)
        self.loop = None
        self.thread = None
        self._stop_event = threading.Event()
        
    def connect(self):
        """Start the client connection in a separate thread"""
        if self.is_connected:
            logger.warning("Already connected")
            return
            
        self._stop_event.clear()
        self.thread = threading.Thread(target=self._run_client)
        self.thread.daemon = True
        self.thread.start()
        
    def disconnect(self):
        """Disconnect from the server"""
        if not self.is_connected:
            logger.warning("Not connected")
            return
            
        logger.info("Disconnecting...")
        self._stop_event.set()
        
        if self.loop:
            try:
                asyncio.run_coroutine_threadsafe(self._disconnect(), self.loop)
                self.thread.join(timeout=5.0)  # Wait up to 5 seconds for clean shutdown
                if self.thread.is_alive():
                    logger.warning("Thread did not terminate cleanly")
            except Exception as e:
                logger.error(f"Error during disconnect: {e}")
        
        self.is_connected = False
        self.loop = None
        self.thread = None
            
    def send_command(self, command: str, value: Optional[Any] = None):
        """Send a command to the robot"""
        if not self.is_connected:
            logger.warning("Not connected, cannot send command")
            return
            
        message = {
            "command": command,
            "value": value
        }
        try:
            asyncio.run_coroutine_threadsafe(
                self._send_command(json.dumps(message)),
                self.loop
            )
        except Exception as e:
            logger.error(f"Error sending command: {e}")
        
    def get_video_frame(self) -> Optional[np.ndarray]:
        """Get the latest video frame"""
        try:
            return self.video_queue.get_nowait()
        except queue.Empty:
            return None
            
    async def _connect(self):
        """Connect to both command and video websockets"""
        try:
            logger.info(f"Connecting to command server at ws://{self.host}:{self.command_port}")
            self.command_ws = await websockets.connect(
                f"ws://{self.host}:{self.command_port}",
                ping_interval=20,
                ping_timeout=20
            )
            
            logger.info(f"Connecting to video server at ws://{self.host}:{self.video_port}")
            self.video_ws = await websockets.connect(
                f"ws://{self.host}:{self.video_port}",
                ping_interval=20,
                ping_timeout=20
            )
            
            self.is_connected = True
            logger.info("Successfully connected to both servers")
            
            # Start video receiver
            asyncio.create_task(self._receive_video())
            
        except Exception as e:
            logger.error(f"Connection error: {e}")
            self.is_connected = False
            raise
            
    async def _disconnect(self):
        """Close both websocket connections"""
        try:
            if self.command_ws:
                await self.command_ws.close()
                logger.info("Command websocket closed")
            if self.video_ws:
                await self.video_ws.close()
                logger.info("Video websocket closed")
        except Exception as e:
            logger.error(f"Error during websocket closure: {e}")
            
    async def _send_command(self, message: str):
        """Send a command through the command websocket"""
        if self.command_ws:
            try:
                await self.command_ws.send(message)
            except Exception as e:
                logger.error(f"Error sending command: {e}")
                raise
            
    async def _receive_video(self):
        """Receive and process video frames"""
        while self.is_connected and not self._stop_event.is_set():
            try:
                if self.video_ws:
                    data = await self.video_ws.recv()
                    # Convert received bytes to numpy array
                    nparr = np.frombuffer(data, np.uint8)
                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    if frame is not None:
                        # Convert BGR to RGB
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        # Update video queue
                        try:
                            self.video_queue.put_nowait(frame)
                        except queue.Full:
                            # Remove oldest frame if queue is full
                            try:
                                self.video_queue.get_nowait()
                                self.video_queue.put_nowait(frame)
                            except queue.Empty:
                                pass
            except websockets.exceptions.ConnectionClosed:
                logger.warning("Video connection closed")
                break
            except Exception as e:
                logger.error(f"Video receive error: {e}")
                break
                
    def _run_client(self):
        """Run the asyncio event loop in a separate thread"""
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self._connect())
            self.loop.run_forever()
        except Exception as e:
            logger.error(f"Error in client thread: {e}")
        finally:
            self.is_connected = False
            if self.loop:
                self.loop.close() 