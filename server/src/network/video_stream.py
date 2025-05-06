import cv2
import numpy as np
import socket
import struct
import time
from typing import Tuple, Optional
from threading import Thread, Event

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
    def __init__(self, host: str = '0.0.0.0', port: int = 5001):
        """Initialize video streamer
        
        Args:
            host: Host address to bind to
            port: UDP port for video streaming
        """
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, port))
        
        self.quality_controller = AdaptiveQualityController()
        self.frame_sequencer = FrameSequencer()
        self.running = Event()
        self.stream_thread: Optional[Thread] = None
        
        # Camera setup
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
    
    def start(self):
        """Start video streaming"""
        if self.stream_thread is not None:
            return
        
        self.running.set()
        self.stream_thread = Thread(target=self._stream_loop)
        self.stream_thread.start()
    
    def stop(self):
        """Stop video streaming"""
        self.running.clear()
        if self.stream_thread is not None:
            self.stream_thread.join()
            self.stream_thread = None
    
    def _stream_loop(self):
        """Main streaming loop"""
        while self.running.is_set():
            start_time = time.time()
            
            # Capture frame
            ret, frame = self.camera.read()
            if not ret:
                continue
            
            # Process frame
            frame = self._process_frame(frame)
            
            # Get current quality
            quality = self.quality_controller.get_quality()
            
            # Compress frame
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            _, frame_data = cv2.imencode('.jpg', frame, encode_param)
            
            # Get sequence number
            sequence = self.frame_sequencer.next()
            
            # Prepare packet
            packet = self._prepare_packet(sequence, frame_data)
            
            # Send packet
            self.socket.sendto(packet, (self.host, self.port))
            
            # Adjust quality based on processing time
            frame_time = time.time() - start_time
            self.quality_controller.adjust_quality(frame_time)
    
    def _process_frame(self, frame: np.ndarray) -> np.ndarray:
        """Process video frame
        
        Args:
            frame: Input frame
            
        Returns:
            Processed frame
        """
        # Add any frame processing here (e.g., resizing, filtering)
        return frame
    
    def _prepare_packet(self, sequence: int, frame_data: np.ndarray) -> bytes:
        """Prepare frame packet for transmission
        
        Args:
            sequence: Frame sequence number
            frame_data: Compressed frame data
            
        Returns:
            Packet data
        """
        # Packet format: [sequence(2)][size(4)][data(n)]
        header = struct.pack('!HI', sequence, len(frame_data))
        return header + frame_data.tobytes()
    
    def cleanup(self):
        """Clean up resources"""
        self.stop()
        self.camera.release()
        self.socket.close() 