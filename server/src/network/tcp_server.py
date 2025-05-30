import socket
import threading
import time
import json
from typing import Dict, Optional, Callable
from queue import Queue
from dataclasses import dataclass
from datetime import datetime
import asyncio
import logging
import signal
from websockets.server import serve as ws_serve
from websockets.exceptions import ConnectionClosed

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
        self._stop_event = None
        self.loop = None
        
    def register_handler(self, command: str, handler: callable):
        self.handlers[command] = handler
        
    async def _handle_client(self, websocket, path):
        self.clients.add(websocket)
        logger.info(f"New client connected to CommandServer. Total clients: {len(self.clients)}")
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    command_name = data.get('command')
                    if command_name in self.handlers:
                        self.handlers[command_name](data)
                    else:
                        logger.warning(f"Unknown command: {command_name}")
                except json.JSONDecodeError:
                    logger.error("Invalid JSON received in CommandServer")
                except Exception as e:
                    logger.error(f"Error handling message in CommandServer: {e}")
        except ConnectionClosed:
            logger.info("Client connection closed in CommandServer")
        finally:
            self.clients.remove(websocket)
            logger.info(f"Client disconnected from CommandServer. Remaining clients: {len(self.clients)}")
            
    async def _shutdown_server_resources(self):
        logger.info(f"CommandServer ({self.host}:{self.port}) shutting down resources...")
        closing_tasks = []
        for client_ws in list(self.clients):
            if client_ws.open:
                task = asyncio.wait_for(client_ws.close(), timeout=2.0)
                closing_tasks.append(task)
        
        if closing_tasks:
            results = await asyncio.gather(*closing_tasks, return_exceptions=True)
            for i, result in enumerate(results):
                if isinstance(result, asyncio.TimeoutError):
                    logger.warning(f"CommandServer: Timeout closing client connection {i+1}.")
                elif isinstance(result, Exception):
                    logger.error(f"CommandServer: Error closing client connection {i+1}: {result}")
        self.clients.clear()

        if self.server:
            self.server.close()
            await self.server.wait_closed()
            logger.info(f"CommandServer ({self.host}:{self.port}) has been closed.")

    async def start_server_async(self):
        """Start the WebSocket server (async part)"""
        self._stop_event = asyncio.Event()
        async with ws_serve(
            self._handle_client,
            self.host,
            self.port,
            ping_interval=20,
            ping_timeout=20,
            reuse_address=True 
        ) as server:
            self.server = server
            self.running = True
            logger.info(f"Command server started on {self.host}:{self.port}")
            await self._stop_event.wait()
            logger.info("CommandServer stop event received, proceeding to resource shutdown.")
            await self._shutdown_server_resources()
            
    def start(self):
        """Start the server in a new event loop (called by ForkliftServer thread)"""
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self.start_server_async())
        except Exception as e:
            logger.error(f"Exception in CommandServer start: {e}", exc_info=True)
        finally:
            if self.loop and not self.loop.is_closed():
                logger.info("Closing CommandServer event loop in start() finally.")
                self.loop.close()
            logger.info("CommandServer start() method finished.")
        
    def stop(self):
        """Stop the server (called by ForkliftServer)"""
        if not self.running and not (self.loop and self.loop.is_running()):
            logger.info(f"CommandServer stop() called, but server or loop not active.")
            return
            
        logger.info(f"CommandServer stop() called. Signaling stop event.")
        self.running = False
        
        if self.loop and self._stop_event and not self._stop_event.is_set():
            self.loop.call_soon_threadsafe(self._stop_event.set)
        else:
            logger.info("CommandServer: Loop or _stop_event not available or already set.")
            
    def cleanup(self):
        """Clean up resources (called by ForkliftServer)"""
        logger.info(f"CommandServer cleanup() called.")
        if self.loop and not self.loop.is_closed():
            logger.warning("CommandServer event loop was not closed by start() method; closing in cleanup.")
            self.loop.call_soon_threadsafe(self.loop.stop)
            current_thread = threading.current_thread()
            if self.loop._thread_id != current_thread.ident:
                time.sleep(1.0)

            if self.loop.is_running():
                self.loop.call_soon_threadsafe(self.loop.stop)
                time.sleep(0.1)
            self.loop.close()
            logger.info("CommandServer event loop closed in cleanup.")
        self.loop = None
        logger.info(f"CommandServer cleanup complete.") 