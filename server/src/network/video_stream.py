import cv2
import numpy as np
import asyncio
import logging
import threading
from typing import Optional
import signal
from websockets.server import serve as ws_serve
from websockets.exceptions import ConnectionClosed
import picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
import io

logger = logging.getLogger(__name__)

class AdaptiveQualityController:
    def __init__(self, initial_quality: int = 80):
        """Initialize quality controller
        
        Args:
            initial_quality: Initial JPEG quality (0-100)
        """
        self.quality = initial_quality
        self.min_quality = 20
        self.max_quality = 95
        self.step_size = 5
        self.last_adjustment = time.time()
        self.adjustment_interval = 1.0  # seconds
    
    def adjust_quality(self, frame_time: float, target_time: float = 0.033):
        """Adjust quality based on frame processing time
        
        Args:
            frame_time: Time taken to process last frame
            target_time: Target frame processing time (default: 30 FPS)
        """
        current_time = time.time()
        if current_time - self.last_adjustment < self.adjustment_interval:
            return
        
        if frame_time > target_time * 1.1:  # Too slow
            self.quality = max(self.min_quality, self.quality - self.step_size)
        elif frame_time < target_time * 0.9:  # Too fast
            self.quality = min(self.max_quality, self.quality + self.step_size)
        
        self.last_adjustment = current_time
    
    def get_quality(self) -> int:
        """Get current quality setting
        
        Returns:
            Current JPEG quality (0-100)
        """
        return self.quality

class FrameSequencer:
    def __init__(self):
        """Initialize frame sequencer"""
        self.sequence = 0
    
    def next(self) -> int:
        """Get next frame sequence number
        
        Returns:
            Next sequence number
        """
        self.sequence = (self.sequence + 1) % 65536
        return self.sequence

class VideoStreamer:
    def __init__(self, host: str = "0.0.0.0", port: int = 3457):
        self.host = host
        self.port = port
        self.server = None
        self.clients = set()
        self.running = False
        self._stop_event = None
        self.camera = None
        self.loop = None
        
    def _initialize_camera(self):
        """Initialize the camera"""
        try:
            # Initialize the camera
            self.camera = picamera2.Picamera2()
            
            # Configure the camera
            config = self.camera.create_video_configuration(
                main={"size": (640, 480), "format": "RGB888"},
                lores={"size": (640, 480), "format": "YUV420"}
            )
            self.camera.configure(config)
            
            # Start the camera
            self.camera.start()
            logger.info("Camera initialized successfully")
        except Exception as e:
            logger.error(f"Error initializing camera: {e}")
            raise
            
    async def _handle_client(self, websocket, path):
        """Handle a client connection
        
        Args:
            websocket: WebSocket connection
            path: Request path
        """
        self.clients.add(websocket)
        logger.info(f"New video client connected. Total clients: {len(self.clients)}")
        
        try:
            while self.running:
                if not self.camera:
                    break
                    
                # Capture frame
                frame = self.camera.capture_array()
                
                # Convert to JPEG
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                await websocket.send(buffer.tobytes())
                
                # Control frame rate
                await asyncio.sleep(1/30)  # ~30 FPS
                
        except ConnectionClosed:
            logger.info("Video client connection closed")
        except Exception as e:
            logger.error(f"Error in video stream: {e}")
        finally:
            self.clients.remove(websocket)
            logger.info(f"Video client disconnected. Remaining clients: {len(self.clients)}")
            
    async def start_server(self):
        """Start the WebSocket server"""
        async with ws_serve(
            self._handle_client,
            self.host,
            self.port,
            ping_interval=20,
            ping_timeout=20
        ) as server:
            self.server = server
            self.running = True
            logger.info(f"Video server started on {self.host}:{self.port}")
            
            # Wait for stop event
            await self._stop_event.wait()
            
    def start(self):
        """Start the video streamer"""
        try:
            self._initialize_camera()
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self._stop_event = asyncio.Event()
            self.loop.run_until_complete(self.start_server())
        except Exception as e:
            logger.error(f"Error starting video streamer: {e}")
            self.cleanup()
            raise
            
    def stop(self):
        """Stop the video streamer"""
        if not self.running:
            return
            
        logger.info("Stopping video streamer...")
        self.running = False
        
        # Set stop event
        if self.loop and self._stop_event:
            self.loop.call_soon_threadsafe(self._stop_event.set)
        
        # Close all client connections
        for client in self.clients:
            self.loop.call_soon_threadsafe(client.close)
        self.clients.clear()
        
        # Stop the server
        if self.server:
            self.server.close()
            
        # Stop the event loop
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
            
    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up video streamer...")
        self.stop()
        
        # Release camera
        if self.camera:
            self.camera.stop()
            self.camera = None
            logger.info("Camera released")
        
        # Wait for the event loop to stop
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.loop.close()
            
        logger.info("Video streamer cleanup complete") 