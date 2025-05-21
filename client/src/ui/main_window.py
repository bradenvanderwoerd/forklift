from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QSlider, QGridLayout, QSizePolicy,
                             QFormLayout, QGroupBox, QLineEdit)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap, QKeyEvent, QDoubleValidator
import cv2
import numpy as np
from src.network.client import RobotClient
import logging
from typing import Optional

logger = logging.getLogger(__name__)

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
        self.setWindowTitle("Forklift Control - Dual View")
        self.setMinimumSize(1200, 700) # Adjusted for two feeds + controls
        # self.resize(1200, 700) # Let minimum size dictate initial or use layout policy
        
        self.robot_client = RobotClient()
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # --- Video Display Area (Horizontal Layout for two feeds) ---
        video_area_layout = QHBoxLayout()
        
        # Onboard Video display
        onboard_video_group = QGroupBox("Onboard Camera (Pi)")
        onboard_video_layout = QVBoxLayout(onboard_video_group)
        self.onboard_video_label = QLabel("Waiting for Onboard Stream...")
        self.onboard_video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.onboard_video_label.setMinimumSize(320, 240) # Smaller minimum
        self.onboard_video_label.setStyleSheet("background-color: black; color: grey;")
        onboard_video_layout.addWidget(self.onboard_video_label)
        video_area_layout.addWidget(onboard_video_group, stretch=1)
        
        # Overhead Video display
        overhead_video_group = QGroupBox("Overhead Camera (Warehouse)")
        overhead_video_layout = QVBoxLayout(overhead_video_group)
        self.overhead_video_label = QLabel("Waiting for Overhead Stream...")
        self.overhead_video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.overhead_video_label.setMinimumSize(320, 240) # Smaller minimum
        self.overhead_video_label.setStyleSheet("background-color: black; color: grey;")
        overhead_video_layout.addWidget(self.overhead_video_label)
        video_area_layout.addWidget(overhead_video_group, stretch=1)
        
        main_layout.addLayout(video_area_layout, stretch=2) # Video area takes more space
        
        # --- Control Panel Area --- (remains largely the same)
        control_panel = QWidget()
        control_layout = QHBoxLayout(control_panel)
        main_layout.addWidget(control_panel, stretch=1) # Control panel takes less space relatively
        
        # Speed control (far left)
        speed_layout = QVBoxLayout()
        self.speed_slider = QSlider(Qt.Orientation.Vertical)
        self.speed_slider.setRange(0, 100)
        self.speed_slider.setValue(50)
        self.speed_slider.valueChanged.connect(self.on_speed_change)
        speed_label = QLabel("Speed")
        speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        speed_layout.addWidget(speed_label)
        speed_layout.addWidget(self.speed_slider, alignment=Qt.AlignmentFlag.AlignCenter)
        control_layout.addLayout(speed_layout)
        
        # WASD Controls & Servo Arrows (middle-left)
        wasd_layout = QGridLayout()
        self.w_key = KeyDisplay("W")
        self.a_key = KeyDisplay("A")
        self.s_key = KeyDisplay("S")
        self.d_key = KeyDisplay("D")
        self.space_key = KeyDisplay("SPACE", width=120, height=35)
        self.up_arrow_key = KeyDisplay("↑")
        self.down_arrow_key = KeyDisplay("↓")
        wasd_layout.addWidget(self.w_key, 0, 1)
        wasd_layout.addWidget(self.a_key, 1, 0)
        wasd_layout.addWidget(self.s_key, 1, 1)
        wasd_layout.addWidget(self.d_key, 1, 2)
        wasd_layout.addWidget(self.space_key, 2, 0, 1, 3)
        wasd_layout.addWidget(self.up_arrow_key, 0, 3)
        wasd_layout.addWidget(self.down_arrow_key, 1, 3)
        key_grid_widget = QWidget()
        key_grid_widget.setLayout(wasd_layout)
        control_layout.addWidget(key_grid_widget) # Directly add, can adjust stretch later if needed

        # Control buttons (Connect, AutoNav, E-Stop) (middle-right)
        action_buttons_layout = QVBoxLayout()
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        action_buttons_layout.addWidget(self.connect_button)
        self.autonav_button = QPushButton("Start Auto-Nav")
        self.autonav_button.clicked.connect(self.toggle_autonav)
        self.autonav_button.setEnabled(False)
        action_buttons_layout.addWidget(self.autonav_button)
        self.emergency_button = QPushButton("Emergency Stop")
        self.emergency_button.setStyleSheet("background-color: red; color: white;")
        self.emergency_button.clicked.connect(self.emergency_stop)
        action_buttons_layout.addWidget(self.emergency_button)
        control_layout.addLayout(action_buttons_layout)
        
        # PID Tuning UI Section (far right)
        self.pid_tuning_group = QGroupBox("PID Tuning")
        self.pid_tuning_group.setMaximumWidth(250) # Set a maximum width for the PID tuning group
        pid_tuning_layout = QFormLayout(self.pid_tuning_group)
        pid_tuning_layout.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.ExpandingFieldsGrow)
        self.turning_pid_inputs = {}
        for param in ["Kp", "Ki", "Kd"]:
            self.turning_pid_inputs[param] = QLineEdit()
            self.turning_pid_inputs[param].setValidator(QDoubleValidator(0, 10000.0, 3))
            pid_tuning_layout.addRow(f"Turn {param}:", self.turning_pid_inputs[param])
        self.apply_turning_pid_button = QPushButton("Apply Turning PID")
        self.apply_turning_pid_button.clicked.connect(self.apply_turning_pid_settings)
        pid_tuning_layout.addRow(self.apply_turning_pid_button)
        self.distance_pid_inputs = {}
        for param in ["Kp", "Ki", "Kd"]:
            self.distance_pid_inputs[param] = QLineEdit()
            self.distance_pid_inputs[param].setValidator(QDoubleValidator(0, 10000.0, 3))
            pid_tuning_layout.addRow(f"Dist {param}:", self.distance_pid_inputs[param])
        self.apply_distance_pid_button = QPushButton("Apply Distance PID")
        self.apply_distance_pid_button.clicked.connect(self.apply_distance_pid_settings)
        pid_tuning_layout.addRow(self.apply_distance_pid_button)
        self.pid_tuning_group.setEnabled(False)
        control_layout.addWidget(self.pid_tuning_group)
        
        # Video update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_video_feeds) # Renamed method
        self.timer.start(33)  # ~30 FPS
        
        self.is_connected = False
        self.current_speed = 50
        self.is_autonav_active = False
        
    def keyPressEvent(self, event: QKeyEvent):
        if not self.is_connected: return
        key = event.key()
        if key == Qt.Key.Key_Space: self.space_key.set_active(True); self.emergency_stop(); return
        if self.is_autonav_active: return
        if key == Qt.Key.Key_W: self.w_key.set_active(True); self.robot_client.send_command("drive", {"direction": "FORWARD", "speed": self.current_speed})
        elif key == Qt.Key.Key_A: self.a_key.set_active(True); self.robot_client.send_command("drive", {"direction": "LEFT", "speed": self.current_speed})
        elif key == Qt.Key.Key_S: self.s_key.set_active(True); self.robot_client.send_command("drive", {"direction": "BACKWARD", "speed": self.current_speed})
        elif key == Qt.Key.Key_D: self.d_key.set_active(True); self.robot_client.send_command("drive", {"direction": "RIGHT", "speed": self.current_speed})
        elif key == Qt.Key.Key_Up: self.up_arrow_key.set_active(True); self.robot_client.send_command("servo", {"step_down": True})
        elif key == Qt.Key.Key_Down: self.down_arrow_key.set_active(True); self.robot_client.send_command("servo", {"step_up": True})
            
    def keyReleaseEvent(self, event: QKeyEvent):
        key = event.key()
        active_map = {Qt.Key.Key_W: self.w_key, Qt.Key.Key_A: self.a_key, Qt.Key.Key_S: self.s_key, Qt.Key.Key_D: self.d_key,
                        Qt.Key.Key_Space: self.space_key, Qt.Key.Key_Up: self.up_arrow_key, Qt.Key.Key_Down: self.down_arrow_key}
        if key in active_map: active_map[key].set_active(False)
            
    def toggle_connection(self):
        if not self.is_connected:
            logger.info("CLIENT: Attempting to connect...")
            self.robot_client.connect()
            # Note: Connection status (self.is_connected) should ideally be updated based on feedback
            # from the client thread or by checking client.is_connected after a short delay.
            # For simplicity, we'll assume it connects quickly here for UI purposes.
            self.connect_button.setText("Disconnect")
            self.autonav_button.setEnabled(True)
            if hasattr(self, 'pid_tuning_group'): self.pid_tuning_group.setEnabled(True)
            self.is_connected = True # Assume connection success for UI feedback
            logger.info("CLIENT: Connect button pressed. RobotClient.connect() called.")
        else:
            logger.info("CLIENT: Attempting to disconnect...")
            if self.is_autonav_active: self.toggle_autonav() # Stop autonav if active
            self.robot_client.disconnect()
            self.connect_button.setText("Connect")
            self.autonav_button.setEnabled(False)
            if hasattr(self, 'pid_tuning_group'): self.pid_tuning_group.setEnabled(False)
            self.is_connected = False
            # Clear video feeds on disconnect
            self.onboard_video_label.setText("Disconnected"); self.onboard_video_label.setStyleSheet("background-color: black; color: grey;")
            self.overhead_video_label.setText("Disconnected"); self.overhead_video_label.setStyleSheet("background-color: black; color: grey;")
            logger.info("CLIENT: Disconnect button pressed. RobotClient.disconnect() called.")

    def emergency_stop(self):
        logger.info("CLIENT: Emergency Stop triggered!")
        self.robot_client.send_command("stop")
        if self.is_autonav_active:
            self.is_autonav_active = False
            self.autonav_button.setText("Start Auto-Nav")
            self.autonav_button.setStyleSheet("")
            logger.info("CLIENT: Auto-Nav deactivated due to Emergency Stop.")
        
    def toggle_autonav(self):
        if not self.is_connected: return
        self.robot_client.send_command("TOGGLE_AUTONAV")
        self.is_autonav_active = not self.is_autonav_active
        if self.is_autonav_active: self.autonav_button.setText("Stop Auto-Nav"); self.autonav_button.setStyleSheet("background-color: #FFA500;")
        else: self.autonav_button.setText("Start Auto-Nav"); self.autonav_button.setStyleSheet("")
        logger.info(f"CLIENT: Auto-Nav toggled to: {self.is_autonav_active}")

    def on_speed_change(self, value):
        self.current_speed = value
        # No need to send if in autonav, server should ignore it anyway based on its logic
        if self.is_connected and not self.is_autonav_active:
            self.robot_client.send_command("drive", {"speed": self.current_speed, "action": "UPDATE_SPEED_ONLY"}) # Assuming server handles this
            logger.debug(f"CLIENT: Speed slider changed to {self.current_speed}")

    def _update_single_video_feed(self, frame: Optional[np.ndarray], label: QLabel, feed_name: str):
        if frame is not None:
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            q_image = QImage(frame.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            scaled_pixmap = pixmap.scaled(label.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            label.setPixmap(scaled_pixmap)
        # else: Do nothing, keep last frame or "Waiting..." text
            
    def update_video_feeds(self):
        if self.robot_client and self.robot_client.is_connected: # Check overall connection
            onboard_frame = self.robot_client.get_onboard_video_frame()
            self._update_single_video_feed(onboard_frame, self.onboard_video_label, "Onboard")
            
            overhead_frame = self.robot_client.get_overhead_video_frame()
            self._update_single_video_feed(overhead_frame, self.overhead_video_label, "Overhead")
        # If not connected, labels show "Disconnected" or "Waiting..."

    def apply_turning_pid_settings(self):
        if not self.is_connected: return
        try:
            payload = {p: float(self.turning_pid_inputs[p].text()) for p in ["Kp", "Ki", "Kd"]}
            self.robot_client.send_command("SET_NAV_TURNING_PID", payload)
            logger.info(f"CLIENT: Sent SET_NAV_TURNING_PID with {payload}")
        except ValueError: logger.error("Invalid input for turning PID.")

    def apply_distance_pid_settings(self):
        if not self.is_connected: return
        try:
            payload = {p: float(self.distance_pid_inputs[p].text()) for p in ["Kp", "Ki", "Kd"]}
            self.robot_client.send_command("SET_NAV_DISTANCE_PID", payload)
            logger.info(f"CLIENT: Sent SET_NAV_DISTANCE_PID with {payload}")
        except ValueError: logger.error("Invalid input for distance PID.")

    def closeEvent(self, event):
        logger.info("CLIENT: MainWindow closeEvent triggered. Disconnecting client.")
        self.robot_client.disconnect() # Ensure client disconnects when window is closed
        super().closeEvent(event) 