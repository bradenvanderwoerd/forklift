import socket
import threading
import time
import json
from typing import Dict, Optional, Callable, Any, Set
from queue import Queue
from dataclasses import dataclass, field
from datetime import datetime
import asyncio
import logging
import signal
from websockets.server import serve as ws_serve
from websockets.exceptions import ConnectionClosed

logger = logging.getLogger(__name__)

@dataclass
class Command:
    """Represents a command received by the server.

    Attributes:
        type (str): The type or name of the command (e.g., "DRIVE", "SERVO_CONTROL").
        data (Dict): A dictionary containing the payload or parameters for the command.
        timestamp (float): The time the command object was created, defaults to current time.
    """
    type: str
    data: Dict[str, Any]
    timestamp: float = field(default_factory=time.time)

class CommandQueue:
    """A simple wrapper around Python's standard queue.Queue for commands.
    
    NOTE: This class is defined here but does not appear to be actively used by 
    the `CommandServer` itself, which processes messages directly. It might be intended
    for other parts of the application or future use (e.g., decoupling command 
    reception from processing).
    """
    def __init__(self, max_size: int = 100):
        """Initializes the command queue.
        
        Args:
            max_size: The maximum number of commands the queue can hold.
        """
        self.queue: Queue[Command] = Queue(maxsize=max_size)
    
    def add(self, command: Command) -> bool:
        """Adds a command to the queue in a non-blocking way.
        
        Args:
            command: The `Command` object to add.
            
        Returns:
            True if the command was added, False if the queue was full.
        """
        try:
            self.queue.put(command, block=False)
            return True
        except Queue.Full:
            logger.warning("CommandQueue is full. Command not added.")
            return False
    
    def get(self) -> Optional[Command]:
        """Retrieves the next command from the queue in a non-blocking way.
        
        Returns:
            The next `Command` object, or None if the queue is empty.
        """
        try:
            return self.queue.get(block=False)
        except Queue.Empty:
            return None
    
    def clear(self):
        """Removes all commands currently in the queue."""
        with self.queue.mutex:
            self.queue.queue.clear()
            self.queue.all_tasks_done.notify_all()
            self.queue.unfinished_tasks = 0

class HeartbeatTimer:
    """Utility class to track heartbeats or connection liveness.

    NOTE: This class is defined here but does not appear to be actively used by
    the `CommandServer`. It could be used to monitor client connections if a
    heartbeat mechanism were implemented.
    """
    def __init__(self, timeout_seconds: float = 5.0):
        """Initializes the heartbeat timer.
        
        Args:
            timeout_seconds: The duration after which a connection might be considered stale
                             if no heartbeat is received (e.g., interval * 2).
        """
        self.timeout_seconds = timeout_seconds
        self.last_heartbeat_time = time.time()
    
    def update(self):
        """Updates the last heartbeat time to the current time."""
        self.last_heartbeat_time = time.time()
    
    def is_alive(self) -> bool:
        """Checks if the connection is considered alive based on the last heartbeat.
        
        Returns:
            True if the time since the last heartbeat is less than `timeout_seconds`.
        """
        return (time.time() - self.last_heartbeat_time) < self.timeout_seconds

class CommandServer:
    """A WebSocket server for receiving and dispatching commands from clients.

    This server listens for incoming WebSocket connections. When a client sends a
    JSON message, the server parses it, identifies the command type, and calls a
    registered handler function for that command type. It manages its own asyncio
    event loop running in a separate thread.
    """
    def __init__(self, host: str = "0.0.0.0", port: int = 3456):
        """Initializes the CommandServer.

        Args:
            host: The network host to bind the WebSocket server to.
            port: The port for the WebSocket command server.
        """
        self.host = host
        self.port = port
        self.websocket_server_instance: Optional[Any] = None
        self.clients: Set[Any] = set()
        self.handlers: Dict[str, Callable[[Dict[str, Any]], None]] = {}
        self._running_server_flag = False
        self._stop_event: Optional[asyncio.Event] = None
        self.async_loop: Optional[asyncio.AbstractEventLoop] = None
        self.server_thread: Optional[threading.Thread] = None
        
    def register_handler(self, command_type: str, handler: Callable[[Dict[str, Any]], None]):
        """Registers a handler function for a specific command type.

        Args:
            command_type: The string identifier of the command (e.g., "DRIVE").
            handler: A callable that takes a dictionary (command data) as an argument.
        """
        self.handlers[command_type] = handler
        logger.info(f"CommandServer: Handler registered for command type '{command_type}'.")
        
    async def _handle_client_websocket(self, websocket: Any, path: str):
        """Handles an individual client's WebSocket connection.

        Receives messages, parses them as JSON, and dispatches to registered handlers.

        Args:
            websocket: The WebSocket connection object for this client.
            path: The path of the WebSocket connection (unused).
        """
        self.clients.add(websocket)
        remote_addr = websocket.remote_address
        logger.info(f"CommandServer: New client connected from {remote_addr}. Total clients: {len(self.clients)}")
        try:
            async for message in websocket:
                if not isinstance(message, str):
                    logger.warning(f"CommandServer: Received non-string message from {remote_addr}: {type(message)}. Ignoring.")
                    continue
                try:
                    parsed_data = json.loads(message)
                    command_name = parsed_data.get('command')
                    command_payload = parsed_data.get('data', {})

                    if command_name in self.handlers:
                        self.handlers[command_name](command_payload) 
                    else:
                        logger.warning(f"CommandServer: Unknown command '{command_name}' from {remote_addr}. Payload: {command_payload}")
                except json.JSONDecodeError:
                    logger.error(f"CommandServer: Invalid JSON received from {remote_addr}: {message}")
                except Exception as e:
                    logger.error(f"CommandServer: Error processing message from {remote_addr}: {e}. Message: '{message}'", exc_info=True)
        except ConnectionClosed:
            logger.info(f"CommandServer: Client {remote_addr} connection closed.")
        except Exception as e:
            logger.error(f"CommandServer: Unhandled error in WebSocket handler for {remote_addr}: {e}", exc_info=True)
        finally:
            self.clients.discard(websocket)
            logger.info(f"CommandServer: Client {remote_addr} disconnected. Remaining clients: {len(self.clients)}")
            
    async def _run_websocket_server_task(self):
        """The main asynchronous task that runs the WebSocket command server."""
        self._stop_event = asyncio.Event()
        
        async with ws_serve(
            self._handle_client_websocket,
            self.host,
            self.port,
            ping_interval=20,
            ping_timeout=20,
            reuse_address=True
        ) as server_obj:
            self.websocket_server_instance = server_obj
            self._running_server_flag = True
            logger.info(f"CommandServer: WebSocket server started and listening on {self.host}:{self.port}")
            await self._stop_event.wait()
        
        self._running_server_flag = False
        logger.info("CommandServer: WebSocket server has shut down.")
            
    def start(self):
        """Starts the CommandServer in a new thread with its own asyncio event loop."""
        if self.server_thread and self.server_thread.is_alive():
            logger.warning("CommandServer: Start called, but server thread is already running.")
            return

        logger.info("CommandServer: Starting server thread and asyncio event loop...")
        self.async_loop = asyncio.new_event_loop()
        
        def loop_runner():
            asyncio.set_event_loop(self.async_loop)
            try:
                self.async_loop.run_until_complete(self._run_websocket_server_task())
            except KeyboardInterrupt:
                logger.info("CommandServer: KeyboardInterrupt in server loop.")
            except Exception as e:
                logger.error(f"CommandServer: Exception in server_thread's run_loop: {e}", exc_info=True)
            finally:
                logger.info("CommandServer: Asyncio loop in server_thread ended.")
                if self.async_loop.is_running():
                    self.async_loop.call_soon_threadsafe(self.async_loop.stop) 
        
        self.server_thread = threading.Thread(target=loop_runner, daemon=True, name="CommandServerThread")
        self.server_thread.start()
        logger.info("CommandServer: Server thread started.")
        
    def stop(self):
        """Signals the CommandServer to stop its operations gracefully."""
        logger.info("CommandServer: Stop requested.")
        if not self._running_server_flag and not (self.server_thread and self.server_thread.is_alive()):
            logger.info("CommandServer: Stop called but server not considered active.")
            self.cleanup()
            return
            
        self._running_server_flag = False
        
        if self.async_loop and self._stop_event and not self._stop_event.is_set():
            logger.info("CommandServer: Setting stop_event for the server's asyncio loop.")
            self.async_loop.call_soon_threadsafe(self._stop_event.set)
        else:
            logger.warning("CommandServer: Cannot signal stop_event (loop/event not available or already set).")
            
        if self.server_thread and self.server_thread.is_alive():
            logger.info("CommandServer: Waiting for server thread to join...")
            self.server_thread.join(timeout=7.0)
            if self.server_thread.is_alive():
                logger.warning("CommandServer: Server thread did not join in time.")
            else:
                logger.info("CommandServer: Server thread joined.")
        self.server_thread = None
        
        self.cleanup()
        logger.info("CommandServer: Stop sequence complete.")
            
    def cleanup(self):
        """Cleans up resources, primarily ensuring the asyncio loop is handled correctly.
        Called as part of the stop sequence.
        """
        logger.info(f"CommandServer: Cleanup initiated.")
        self._running_server_flag = False
        self.clients.clear()

        if self.async_loop:
            if self.async_loop.is_running():
                logger.info("CommandServer cleanup: Asyncio loop is still running. Requesting stop.")
                self.async_loop.call_soon_threadsafe(self.async_loop.stop)
        self.async_loop = None
        logger.info(f"CommandServer: Cleanup finished.")

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    test_server = CommandServer(host='0.0.0.0', port=3456)

    def handle_drive_command(data: Dict[str, Any]):
        logger.info(f"Test Handler: Received DRIVE command with data: {data}")

    def handle_servo_command(data: Dict[str, Any]):
        logger.info(f"Test Handler: Received SERVO_CONTROL command with data: {data}")

    test_server.register_handler("DRIVE", handle_drive_command)
    test_server.register_handler("SERVO_CONTROL", handle_servo_command)

    def signal_handler(sig, frame):
        logger.info(f"Signal {sig} received, stopping CommandServer...")
        test_server.stop()
        time.sleep(1)
        logger.info("Exiting CommandServer test.")
        sys.exit(0)

    import sys
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        test_server.start()
        while test_server.server_thread and test_server.server_thread.is_alive():
            time.sleep(0.5)
    except Exception as e:
        logger.error(f"Unhandled exception in CommandServer test main: {e}", exc_info=True)
    finally:
        logger.info("CommandServer test main finally block. Ensuring server is stopped.")
        if test_server:
            test_server.stop()
    logger.info("CommandServer test finished.") 