from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QSlider, QGridLayout)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap, QKeyEvent
import cv2
import numpy as np
from src.network.client import RobotClient

class KeyDisplay(QLabel):
    def __init__(self, key_text, width=35, height=35):
        super().__init__(key_text)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFixedSize(width, height)
        self.setStyleSheet("""
            QLabel {
                background-color: #2c2c2c;
                color: white;
                border: 2px solid #3c3c3c;
                border-radius: 4px;
                font-size: 14px;
                font-weight: bold;
            }
        """)
        self.active = False
        
    def set_active(self, active):
        self.active = active
        if active:
            self.setStyleSheet("""
                QLabel {
                    background-color: #4CAF50;
                    color: white;
                    border: 2px solid #45a049;
                    border-radius: 4px;
                    font-size: 14px;
                    font-weight: bold;
                }
            """)
        else:
            self.setStyleSheet("""
                QLabel {
                    background-color: #2c2c2c;
                    color: white;
                    border: 2px solid #3c3c3c;
                    border-radius: 4px;
                    font-size: 14px;
                    font-weight: bold;
                }
            """)

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
        self.speed_slider.setRange(0, 100)
        self.speed_slider.setValue(50)  # Start at 50%
        self.speed_slider.valueChanged.connect(self.on_speed_change)
        speed_layout.addWidget(QLabel("Speed"))
        speed_layout.addWidget(self.speed_slider)
        control_layout.addLayout(speed_layout)
        
        # WASD Controls
        wasd_layout = QGridLayout()
        wasd_layout.setSpacing(10)
        
        # Create key displays
        self.w_key = KeyDisplay("W")
        self.a_key = KeyDisplay("A")
        self.s_key = KeyDisplay("S")
        self.d_key = KeyDisplay("D")
        self.space_key = KeyDisplay("SPACE", width=120, height=35)  # Wide space bar
        
        # Add keys to grid
        wasd_layout.addWidget(self.w_key, 0, 1)
        wasd_layout.addWidget(self.a_key, 1, 0)
        wasd_layout.addWidget(self.s_key, 1, 1)
        wasd_layout.addWidget(self.d_key, 1, 2)
        wasd_layout.addWidget(self.space_key, 2, 0, 1, 3)
        
        # Center the WASD grid in a QWidget
        wasd_widget = QWidget()
        wasd_widget.setLayout(wasd_layout)
        wasd_outer_layout = QHBoxLayout()
        wasd_outer_layout.addStretch(1)
        wasd_outer_layout.addWidget(wasd_widget)
        wasd_outer_layout.addStretch(1)
        control_layout.addLayout(wasd_outer_layout)
        
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
        
        # Video update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_video)
        self.timer.start(33)  # ~30 FPS
        
        self.is_connected = False
        self.current_speed = 50
        
    def keyPressEvent(self, event: QKeyEvent):
        if not self.is_connected:
            return
            
        key = event.key()
        if key == Qt.Key.Key_W:
            self.w_key.set_active(True)
            self.robot_client.send_command("drive", {"direction": "FORWARD", "speed": self.current_speed})
        elif key == Qt.Key.Key_A:
            self.a_key.set_active(True)
            self.robot_client.send_command("drive", {"direction": "LEFT", "speed": self.current_speed})
        elif key == Qt.Key.Key_S:
            self.s_key.set_active(True)
            self.robot_client.send_command("drive", {"direction": "BACKWARD", "speed": self.current_speed})
        elif key == Qt.Key.Key_D:
            self.d_key.set_active(True)
            self.robot_client.send_command("drive", {"direction": "RIGHT", "speed": self.current_speed})
        elif key == Qt.Key.Key_Space:
            self.space_key.set_active(True)
            self.emergency_stop()
            
    def keyReleaseEvent(self, event: QKeyEvent):
        key = event.key()
        if key == Qt.Key.Key_W:
            self.w_key.set_active(False)
        elif key == Qt.Key.Key_A:
            self.a_key.set_active(False)
        elif key == Qt.Key.Key_S:
            self.s_key.set_active(False)
        elif key == Qt.Key.Key_D:
            self.d_key.set_active(False)
        elif key == Qt.Key.Key_Space:
            self.space_key.set_active(False)
            
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
        self.robot_client.send_command("stop")
        
    def on_speed_change(self, value):
        self.current_speed = value
        if self.is_connected:
            self.robot_client.send_command("set_speed", value)
            
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