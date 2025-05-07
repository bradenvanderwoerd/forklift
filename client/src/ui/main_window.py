from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QSlider)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap
import cv2
import numpy as np
from src.network.client import RobotClient

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Forklift Control")
        self.setMinimumSize(1280, 720)
        
        # Initialize robot client
        self.robot_client = RobotClient()
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # Video display
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setMinimumHeight(480)
        layout.addWidget(self.video_label, stretch=2)
        
        # Control panel
        control_panel = QWidget()
        control_layout = QHBoxLayout(control_panel)
        layout.addWidget(control_panel, stretch=1)
        
        # Speed control
        speed_layout = QVBoxLayout()
        self.speed_slider = QSlider(Qt.Orientation.Vertical)
        self.speed_slider.setRange(-100, 100)
        self.speed_slider.setValue(0)
        self.speed_slider.valueChanged.connect(self.on_speed_change)
        speed_layout.addWidget(QLabel("Speed"))
        speed_layout.addWidget(self.speed_slider)
        control_layout.addLayout(speed_layout)
        
        # Steering control
        steering_layout = QVBoxLayout()
        self.steering_slider = QSlider(Qt.Orientation.Horizontal)
        self.steering_slider.setRange(-100, 100)
        self.steering_slider.setValue(0)
        self.steering_slider.valueChanged.connect(self.on_steering_change)
        steering_layout.addWidget(QLabel("Steering"))
        steering_layout.addWidget(self.steering_slider)
        control_layout.addLayout(steering_layout)
        
        # Control buttons
        button_layout = QVBoxLayout()
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        button_layout.addWidget(self.connect_button)
        
        self.emergency_button = QPushButton("Emergency Stop")
        self.emergency_button.setStyleSheet("background-color: red; color: white;")
        self.emergency_button.clicked.connect(self.emergency_stop)
        button_layout.addWidget(self.emergency_button)
        
        control_layout.addLayout(button_layout)
        layout.addWidget(control_panel)
        
        # Video update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_video)
        self.timer.start(33)  # ~30 FPS
        
        self.is_connected = False
        
    def toggle_connection(self):
        if not self.is_connected:
            self.robot_client.connect()
            self.connect_button.setText("Disconnect")
            self.is_connected = True
        else:
            self.robot_client.disconnect()
            self.connect_button.setText("Connect")
            self.is_connected = False
            
    def emergency_stop(self):
        self.speed_slider.setValue(0)
        self.steering_slider.setValue(0)
        self.robot_client.send_command("emergency_stop")
        
    def on_speed_change(self, value):
        if self.is_connected:
            self.robot_client.send_command("set_speed", value)
            
    def on_steering_change(self, value):
        if self.is_connected:
            self.robot_client.send_command("set_steering", value)
            
    def update_video(self):
        if self.is_connected:
            frame = self.robot_client.get_video_frame()
            if frame is not None:
                # Convert frame to QImage
                height, width, channel = frame.shape
                bytes_per_line = 3 * width
                q_image = QImage(frame.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)
                pixmap = QPixmap.fromImage(q_image)
                
                # Scale pixmap to fit label while maintaining aspect ratio
                scaled_pixmap = pixmap.scaled(self.video_label.size(), 
                                            Qt.AspectRatioMode.KeepAspectRatio,
                                            Qt.TransformationMode.SmoothTransformation)
                self.video_label.setPixmap(scaled_pixmap) 