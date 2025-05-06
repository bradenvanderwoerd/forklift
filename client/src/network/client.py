import asyncio
import websockets
import json
import cv2
import numpy as np
from typing import Optional, Tuple, Dict, Any
import threading
import queue

class RobotClient:
    def __init__(self, host: str = "localhost", command_port: int = 8765, video_port: int = 8766):
        self.host = host
        self.command_port = command_port
        self.video_port = video_port
        self.command_ws = None
        self.video_ws = None
        self.is_connected = False
        self.video_queue = queue.Queue(maxsize=10)
        self.loop = None
        self.thread = None
        
    def connect(self):
        """Start the client connection in a separate thread"""
        self.thread = threading.Thread(target=self._run_client)
        self.thread.daemon = True
        self.thread.start()
        
    def disconnect(self):
        """Disconnect from the server"""
        if self.loop:
            asyncio.run_coroutine_threadsafe(self._disconnect(), self.loop)
            self.thread.join()
            self.is_connected = False
            
    def send_command(self, command: str, value: Optional[Any] = None):
        """Send a command to the robot"""
        if not self.is_connected:
            return
            
        message = {
            "command": command,
            "value": value
        }
        asyncio.run_coroutine_threadsafe(
            self._send_command(json.dumps(message)),
            self.loop
        )
        
    def get_video_frame(self) -> Optional[np.ndarray]:
        """Get the latest video frame"""
        try:
            return self.video_queue.get_nowait()
        except queue.Empty:
            return None
            
    async def _connect(self):
        """Connect to both command and video websockets"""
        try:
            self.command_ws = await websockets.connect(
                f"ws://{self.host}:{self.command_port}"
            )
            self.video_ws = await websockets.connect(
                f"ws://{self.host}:{self.video_port}"
            )
            self.is_connected = True
            
            # Start video receiver
            asyncio.create_task(self._receive_video())
            
        except Exception as e:
            print(f"Connection error: {e}")
            self.is_connected = False
            
    async def _disconnect(self):
        """Close both websocket connections"""
        if self.command_ws:
            await self.command_ws.close()
        if self.video_ws:
            await self.video_ws.close()
            
    async def _send_command(self, message: str):
        """Send a command through the command websocket"""
        if self.command_ws:
            await self.command_ws.send(message)
            
    async def _receive_video(self):
        """Receive and process video frames"""
        while self.is_connected:
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
            except Exception as e:
                print(f"Video receive error: {e}")
                break
                
    def _run_client(self):
        """Run the asyncio event loop in a separate thread"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._connect())
        self.loop.run_forever() 