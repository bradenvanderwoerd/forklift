import socket
import threading
import time
import json
from typing import Dict, Optional, Callable
from queue import Queue
from dataclasses import dataclass
from datetime import datetime
import asyncio
import websockets
import logging
import signal

logger = logging.getLogger(__name__)

@dataclass
class Command:
    type: str
    data: Dict
    timestamp: float = 0.0
    
    def __post_init__(self):
        if self.timestamp == 0.0:
            self.timestamp = time.time()

class CommandQueue:
    def __init__(self, max_size: int = 100):
        """Initialize command queue
        
        Args:
            max_size: Maximum queue size
        """
        self.queue = Queue(maxsize=max_size)
        self.processing = False
    
    def add(self, command: Command) -> bool:
        """Add command to queue
        
        Args:
            command: Command to add
            
        Returns:
            True if command was added, False if queue is full
        """
        try:
            self.queue.put(command, block=False)
            return True
        except Queue.Full:
            return False
    
    def get(self) -> Optional[Command]:
        """Get next command from queue
        
        Returns:
            Next command or None if queue is empty
        """
        try:
            return self.queue.get(block=False)
        except Queue.Empty:
            return None
    
    def clear(self):
        """Clear all commands from queue"""
        while not self.queue.empty():
            try:
                self.queue.get_nowait()
            except Queue.Empty:
                break

class HeartbeatTimer:
    def __init__(self, interval: float = 1.0):
        """Initialize heartbeat timer
        
        Args:
            interval: Heartbeat interval in seconds
        """
        self.interval = interval
        self.last_heartbeat = time.time()
    
    def update(self):
        """Update last heartbeat time"""
        self.last_heartbeat = time.time()
    
    def is_alive(self) -> bool:
        """Check if connection is alive
        
        Returns:
            True if last heartbeat was within interval
        """
        return (time.time() - self.last_heartbeat) < self.interval * 2

class CommandServer:
    def __init__(self, host: str = "0.0.0.0", port: int = 3456):
        self.host = host
        self.port = port
        self.server = None
        self.clients = set()
        self.handlers = {}
        self.running = False
        self._stop_event = asyncio.Event()
        
    def register_handler(self, command: str, handler: Callable):
        """Register a handler for a specific command
        
        Args:
            command: Command name
            handler: Handler function
        """
        self.handlers[command] = handler
        
    async def _handle_client(self, websocket, path):
        """Handle a client connection
        
        Args:
            websocket: WebSocket connection
            path: Request path
        """
        self.clients.add(websocket)
        logger.info(f"New client connected. Total clients: {len(self.clients)}")
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    command = data.get('command')
                    if command in self.handlers:
                        self.handlers[command](data)
                    else:
                        logger.warning(f"Unknown command: {command}")
                except json.JSONDecodeError:
                    logger.error("Invalid JSON received")
                except Exception as e:
                    logger.error(f"Error handling message: {e}")
        except websockets.exceptions.ConnectionClosed:
            logger.info("Client connection closed")
        finally:
            self.clients.remove(websocket)
            logger.info(f"Client disconnected. Remaining clients: {len(self.clients)}")
            
    async def start_server(self):
        """Start the WebSocket server"""
        self.server = await websockets.serve(
            self._handle_client,
            self.host,
            self.port,
            ping_interval=20,
            ping_timeout=20
        )
        self.running = True
        logger.info(f"Command server started on {self.host}:{self.port}")
        
        # Wait for stop event
        await self._stop_event.wait()
        
    def start(self):
        """Start the server in a new event loop"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.start_server())
        
    def stop(self):
        """Stop the server"""
        if not self.running:
            return
            
        logger.info("Stopping command server...")
        self.running = False
        
        # Set stop event
        if self.loop and self._stop_event:
            asyncio.run_coroutine_threadsafe(self._stop_event.set(), self.loop)
        
        # Close all client connections
        for client in self.clients:
            asyncio.run_coroutine_threadsafe(client.close(), self.loop)
        self.clients.clear()
        
        # Stop the server
        if self.server:
            self.server.close()
            asyncio.run_coroutine_threadsafe(self.server.wait_closed(), self.loop)
            
        # Stop the event loop
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)
            
    def cleanup(self):
        """Clean up resources"""
        logger.info("Cleaning up command server...")
        self.stop()
        
        # Wait for the event loop to stop
        if self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.loop.close()
            
        logger.info("Command server cleanup complete") 