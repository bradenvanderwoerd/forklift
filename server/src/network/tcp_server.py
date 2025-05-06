import socket
import threading
import time
import json
from typing import Dict, Optional, Callable
from queue import Queue
from dataclasses import dataclass
from datetime import datetime

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
    def __init__(self, host: str = '0.0.0.0', port: int = 4001):
        """Initialize command server
        
        Args:
            host: Host address to bind to
            port: TCP port for command server
        """
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, port))
        
        self.command_queue = CommandQueue()
        self.heartbeat_timer = HeartbeatTimer()
        self.running = False
        self.server_thread: Optional[threading.Thread] = None
        self.command_handlers: Dict[str, Callable] = {}
    
    def register_handler(self, command_type: str, handler: Callable):
        """Register command handler
        
        Args:
            command_type: Type of command to handle
            handler: Handler function
        """
        self.command_handlers[command_type] = handler
    
    def start(self):
        """Start command server"""
        if self.server_thread is not None:
            return
        
        self.running = True
        self.server_thread = threading.Thread(target=self._server_loop)
        self.server_thread.start()
    
    def stop(self):
        """Stop command server"""
        self.running = False
        if self.server_thread is not None:
            self.server_thread.join()
            self.server_thread = None
    
    def _server_loop(self):
        """Main server loop"""
        self.socket.listen(1)
        
        while self.running:
            try:
                client_socket, address = self.socket.accept()
                self._handle_client(client_socket, address)
            except socket.error:
                continue
    
    def _handle_client(self, client_socket: socket.socket, address: tuple):
        """Handle client connection
        
        Args:
            client_socket: Client socket
            address: Client address
        """
        try:
            while self.running:
                # Receive command
                data = client_socket.recv(1024)
                if not data:
                    break
                
                # Parse command
                try:
                    command_data = json.loads(data.decode())
                    command = Command(
                        type=command_data['type'],
                        data=command_data['data']
                    )
                except (json.JSONDecodeError, KeyError):
                    continue
                
                # Handle heartbeat
                if command.type == 'heartbeat':
                    self.heartbeat_timer.update()
                    continue
                
                # Add command to queue
                if not self.command_queue.add(command):
                    # Queue is full, send error response
                    response = {
                        'status': 'error',
                        'message': 'Command queue is full'
                    }
                    client_socket.send(json.dumps(response).encode())
                    continue
                
                # Process command
                self._process_command(command)
                
                # Send acknowledgment
                response = {
                    'status': 'ok',
                    'command_id': command.timestamp
                }
                client_socket.send(json.dumps(response).encode())
        
        except socket.error:
            pass
        finally:
            client_socket.close()
    
    def _process_command(self, command: Command):
        """Process command from queue
        
        Args:
            command: Command to process
        """
        handler = self.command_handlers.get(command.type)
        if handler is not None:
            try:
                handler(command.data)
            except Exception as e:
                print(f"Error processing command: {e}")
    
    def cleanup(self):
        """Clean up resources"""
        self.stop()
        self.socket.close() 