from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QSlider, QGridLayout, QSizePolicy,
                             QFormLayout, QGroupBox, QLineEdit)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap, QKeyEvent, QDoubleValidator
import cv2
import numpy as np
from src.network.client import RobotClient
import logging

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
        self.setWindowTitle("Forklift Control")
        self.setMinimumSize(950, 680)
        self.resize(950, 680)
        
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
        
        # Control panel - Revert to QHBoxLayout for the main control area
        control_panel = QWidget()
        control_layout = QHBoxLayout(control_panel) # This is the main horizontal layout for controls
        layout.addWidget(control_panel, stretch=1)
        
        # --- Item 1: Speed control (far left) ---
        speed_layout = QVBoxLayout()
        self.speed_slider = QSlider(Qt.Orientation.Vertical)
        self.speed_slider.setRange(0, 100)
        self.speed_slider.setValue(50)
        self.speed_slider.valueChanged.connect(self.on_speed_change)
        speed_layout.addWidget(QLabel("Speed"))
        speed_layout.addWidget(self.speed_slider)
        control_layout.addLayout(speed_layout)
        
        # --- Item 2: WASD Controls & Servo Arrows (middle-left) ---
        wasd_layout = QGridLayout()
        wasd_layout.setSpacing(10)
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
        key_grid_outer_layout = QHBoxLayout() # Centering layout for the key grid
        key_grid_outer_layout.addStretch(1)
        key_grid_outer_layout.addWidget(key_grid_widget)
        key_grid_outer_layout.addStretch(1)
        control_layout.addLayout(key_grid_outer_layout)

        # --- Item 3: Control buttons (Connect, AutoNav, E-Stop) (middle-right) ---
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
        
        # --- Item 4: PID Tuning UI Section (far right) ---
        self.pid_tuning_group = QGroupBox("PID Tuning")
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
        self.timer.timeout.connect(self.update_video)
        self.timer.start(33)  # ~30 FPS
        
        self.is_connected = False
        self.current_speed = 50
        self.is_autonav_active = False # Client-side state for auto-navigation
        
    def keyPressEvent(self, event: QKeyEvent):
        if not self.is_connected:
            return
            
        key = event.key()

        # Emergency stop via Spacebar always active
        if key == Qt.Key.Key_Space:
            self.space_key.set_active(True)
            self.emergency_stop() # Call the enhanced emergency_stop method
            return # Do not process other keys if space is pressed

        if self.is_autonav_active: # If auto-nav is active, ignore other movement/servo keys
            # We might want to allow some keys even in autonav, e.g., a different way to stop autonav
            # For now, all movement keys are suppressed.
            return
            
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
        elif key == Qt.Key.Key_Up:
            self.up_arrow_key.set_active(True)
            self.robot_client.send_command("servo", {"step_down": True})
        elif key == Qt.Key.Key_Down:
            self.down_arrow_key.set_active(True)
            self.robot_client.send_command("servo", {"step_up": True})
            
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
        elif key == Qt.Key.Key_Up:
            self.up_arrow_key.set_active(False)
        elif key == Qt.Key.Key_Down:
            self.down_arrow_key.set_active(False)
            
    def toggle_connection(self):
        if not self.is_connected:
            self.robot_client.connect()
            self.connect_button.setText("Disconnect")
            self.autonav_button.setEnabled(True)
            if hasattr(self, 'pid_tuning_group') and self.pid_tuning_group:
                 self.pid_tuning_group.setEnabled(True)
            self.is_connected = True
        else:
            if self.is_autonav_active:
                self.toggle_autonav()
            self.robot_client.disconnect()
            self.connect_button.setText("Connect")
            self.autonav_button.setEnabled(False)
            if hasattr(self, 'pid_tuning_group') and self.pid_tuning_group:
                 self.pid_tuning_group.setEnabled(False)
            self.is_connected = False
            
    def emergency_stop(self):
        logger.info("CLIENT: Emergency Stop triggered!") # Add client-side log
        self.robot_client.send_command("stop")
        if self.is_autonav_active:
            # We don't send another TOGGLE_AUTONAV here because the server-side e-stop
            # should already be deactivating auto-nav. We just update client state.
            self.is_autonav_active = False
            self.autonav_button.setText("Start Auto-Nav")
            self.autonav_button.setStyleSheet("") # Reset style
            logger.info("CLIENT: Auto-Nav deactivated due to Emergency Stop.")
            # Potentially re-enable manual controls if they were disabled by autonav state
            # This is implicitly handled as keyPressEvent will no longer be blocked by self.is_autonav_active
        # Potentially re-enable manual controls if they were disabled by autonav state
        # This is implicitly handled as keyPressEvent will no longer be blocked by self.is_autonav_active
        
    def toggle_autonav(self):
        if not self.is_connected:
            return

        self.robot_client.send_command("TOGGLE_AUTONAV")
        self.is_autonav_active = not self.is_autonav_active
        
        if self.is_autonav_active:
            self.autonav_button.setText("Stop Auto-Nav")
            self.autonav_button.setStyleSheet("background-color: #FFA500; color: white;") # Orange for active
            logger.info("CLIENT: Auto-Nav ACTIVATED.")
            # Manual controls (WASD, arrows, speed slider) will be suppressed by self.is_autonav_active flag
        else:
            self.autonav_button.setText("Start Auto-Nav")
            self.autonav_button.setStyleSheet("") # Reset style
            logger.info("CLIENT: Auto-Nav DEACTIVATED.")
            # Manual controls are now re-enabled implicitly

    def on_speed_change(self, value):
        self.current_speed = value
        if self.is_connected and not self.is_autonav_active: # Only send if connected and not in auto-nav
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

    def apply_turning_pid_settings(self):
        if not self.is_connected:
            logger.warning("Not connected. Cannot apply PID settings.")
            return
        try:
            kp = float(self.turning_pid_inputs["Kp"].text())
            ki = float(self.turning_pid_inputs["Ki"].text())
            kd = float(self.turning_pid_inputs["Kd"].text())
            payload = {"kp": kp, "ki": ki, "kd": kd}
            self.robot_client.send_command("SET_NAV_TURNING_PID", payload)
            logger.info(f"CLIENT: Sent SET_NAV_TURNING_PID with {payload}")
        except ValueError:
            logger.error("Invalid input for turning PID gains. Please enter numbers.")

    def apply_distance_pid_settings(self):
        if not self.is_connected:
            logger.warning("Not connected. Cannot apply PID settings.")
            return
        try:
            kp = float(self.distance_pid_inputs["Kp"].text())
            ki = float(self.distance_pid_inputs["Ki"].text())
            kd = float(self.distance_pid_inputs["Kd"].text())
            payload = {"kp": kp, "ki": ki, "kd": kd}
            self.robot_client.send_command("SET_NAV_DISTANCE_PID", payload)
            logger.info(f"CLIENT: Sent SET_NAV_DISTANCE_PID with {payload}")
        except ValueError:
            logger.error("Invalid input for distance PID gains. Please enter numbers.") 