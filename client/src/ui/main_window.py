from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QSlider, QGridLayout, QSizePolicy,
                             QFormLayout, QGroupBox, QLineEdit)
from PyQt6.QtCore import Qt, QTimer, QEvent
from PyQt6.QtGui import QImage, QPixmap, QKeyEvent, QDoubleValidator
import cv2
import numpy as np
from src.network.client import RobotClient
import logging
from typing import Optional, Dict

# Import client-side config for servo pins
from src.utils.config import (FORK_SERVO_A_PIN, FORK_SERVO_B_PIN, FORK_SERVO_C_PIN,
                              FORK_SERVO_D_PIN, FORK_SERVO_E_PIN, FORK_SERVO_F_PIN,
                              SERVO_KEY_TO_PIN_MAPPING, UI_SERVO_ORDER_MAPPING)

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
        self.setMinimumSize(1350, 700) # Adjusted for two feeds + controls + servo grid
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setFocus()
        
        self.robot_client = RobotClient()
        self.selected_servo_pin = FORK_SERVO_A_PIN # Default to Fork A (pin 13)
        self.servo_key_displays: Dict[int, KeyDisplay] = {} # To store pin -> KeyDisplay mapping for servo grid
        
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
        self.speed_slider.setMinimumHeight(200) # Increased minimum height for the slider
        self.speed_slider.valueChanged.connect(self.on_speed_change)
        speed_label = QLabel("Speed (75-100)")
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
        control_layout.addWidget(key_grid_widget)

        # Servo Selection Grid (middle-center)
        servo_selection_group = QGroupBox("Selected Servo (1-6)")
        servo_grid_layout = QGridLayout(servo_selection_group)
        
        # Create KeyDisplay widgets for servos 1-6 and map them
        # Grid:  5 6
        #        3 4
        #        1 2
        # Pins: E F
        #       C D
        #       A B
        key_to_servo_display_text = {
            FORK_SERVO_E_PIN: "5", FORK_SERVO_F_PIN: "6",
            FORK_SERVO_C_PIN: "3", FORK_SERVO_D_PIN: "4",
            FORK_SERVO_A_PIN: "1", FORK_SERVO_B_PIN: "2",
        }

        positions = {
            FORK_SERVO_E_PIN: (0, 0), FORK_SERVO_F_PIN: (0, 1), # Top row (5, 6)
            FORK_SERVO_C_PIN: (1, 0), FORK_SERVO_D_PIN: (1, 1), # Middle row (3, 4)
            FORK_SERVO_A_PIN: (2, 0), FORK_SERVO_B_PIN: (2, 1), # Bottom row (1, 2)
        }

        for pin, text_label in key_to_servo_display_text.items():
            display = KeyDisplay(text_label)
            self.servo_key_displays[pin] = display
            row, col = positions[pin]
            servo_grid_layout.addWidget(display, row, col)

        control_layout.addWidget(servo_selection_group)
        self._update_servo_selection_display() # Initial highlight

        # Control buttons (Connect, AutoNav, E-Stop) (middle-right)
        action_buttons_layout = QVBoxLayout()
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        action_buttons_layout.addWidget(self.connect_button)
        
        self.autonav_button = QPushButton("Start Auto-Nav")
        self.autonav_button.clicked.connect(self.toggle_autonav)
        self.autonav_button.setEnabled(False)
        action_buttons_layout.addWidget(self.autonav_button)
        
        self.set_pickup_button = QPushButton("Set Pickup Pose")
        self.set_pickup_button.clicked.connect(lambda: self.set_overhead_target_pose("pickup"))
        self.set_pickup_button.setEnabled(False) # Enable when connected
        action_buttons_layout.addWidget(self.set_pickup_button)
        
        self.set_dropoff_button = QPushButton("Set Dropoff Pose")
        self.set_dropoff_button.clicked.connect(lambda: self.set_overhead_target_pose("dropoff"))
        self.set_dropoff_button.setEnabled(False) # Enable when connected
        action_buttons_layout.addWidget(self.set_dropoff_button)
        
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
            self.turning_pid_inputs[param].editingFinished.connect(self.setFocus) # Re-focus main window
            pid_tuning_layout.addRow(f"Turn {param}:", self.turning_pid_inputs[param])
        self.apply_turning_pid_button = QPushButton("Apply Turning PID")
        self.apply_turning_pid_button.clicked.connect(self.apply_turning_pid_settings)
        pid_tuning_layout.addRow(self.apply_turning_pid_button)
        self.distance_pid_inputs = {}
        for param in ["Kp", "Ki", "Kd"]:
            self.distance_pid_inputs[param] = QLineEdit()
            self.distance_pid_inputs[param].setValidator(QDoubleValidator(0, 10000.0, 3))
            self.distance_pid_inputs[param].editingFinished.connect(self.setFocus) # Re-focus main window
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
        self.on_speed_change(self.speed_slider.value()) # Initialize current_speed correctly
        self.is_autonav_active = False
        self._update_servo_selection_display() # Initial update
        
    def _update_servo_selection_display(self):
        """Highlights the currently selected servo in the UI grid."""
        for pin, display in self.servo_key_displays.items():
            display.set_active(pin == self.selected_servo_pin)

    def keyPressEvent(self, event: QKeyEvent):
        if not self.is_connected: return
        key = event.key()
        key_char = event.text() # Get character for number keys

        # Servo Selection Keys (1-6)
        if key_char in SERVO_KEY_TO_PIN_MAPPING:
            self.selected_servo_pin = SERVO_KEY_TO_PIN_MAPPING[key_char]
            self._update_servo_selection_display()
            logger.info(f"Selected servo pin: {self.selected_servo_pin} (Key: {key_char})")
            self.setFocus() # Return focus to main window to ensure other keys work
            return # Consume event

        if key == Qt.Key.Key_Space: self.space_key.set_active(True); self.emergency_stop(); return
        if self.is_autonav_active: return

        # Servo (Fork) Control with Up/Down Arrows for the SELECTED servo
        if key == Qt.Key.Key_Up:
            self.up_arrow_key.set_active(True)
            logger.info(f"Sending SERVO UP command for pin {self.selected_servo_pin}")
            self.robot_client.send_command('servo', {'pin': self.selected_servo_pin, 'step_up': True})
            return
        elif key == Qt.Key.Key_Down:
            self.down_arrow_key.set_active(True)
            logger.info(f"Sending SERVO DOWN command for pin {self.selected_servo_pin}")
            self.robot_client.send_command('servo', {'pin': self.selected_servo_pin, 'step_down': True})
            return

        # WASD Drive Controls
        if key == Qt.Key.Key_W: self.w_key.set_active(True); self.robot_client.send_command("drive", {"direction": "FORWARD", "speed": self.current_speed})
        elif key == Qt.Key.Key_A: self.a_key.set_active(True); self.robot_client.send_command("drive", {"direction": "LEFT", "speed": self.current_speed})
        elif key == Qt.Key.Key_S: self.s_key.set_active(True); self.robot_client.send_command("drive", {"direction": "BACKWARD", "speed": self.current_speed})
        elif key == Qt.Key.Key_D: self.d_key.set_active(True); self.robot_client.send_command("drive", {"direction": "RIGHT", "speed": self.current_speed})
            
    def keyReleaseEvent(self, event: QKeyEvent):
        if not self.is_connected or self.is_autonav_active: return
        key = event.key()

        # Servo (Fork) Control Release
        if key == Qt.Key.Key_Up:
            self.up_arrow_key.set_active(False)
            # Optionally send a "servo stop" or rely on server for step behavior
            return
        elif key == Qt.Key.Key_Down:
            self.down_arrow_key.set_active(False)
            # Optionally send a "servo stop" or rely on server for step behavior
            return

        # WASD Release
        if key == Qt.Key.Key_W: self.w_key.set_active(False)
        elif key == Qt.Key.Key_A: self.a_key.set_active(False)
        elif key == Qt.Key.Key_S: self.s_key.set_active(False)
        elif key == Qt.Key.Key_D: self.d_key.set_active(False)
        elif key == Qt.Key.Key_Space: self.space_key.set_active(False)
            
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
            self.set_pickup_button.setEnabled(True)   # Enable on connect
            self.set_dropoff_button.setEnabled(True) # Enable on connect
            self.is_connected = True # Assume connection success for UI feedback
            logger.info("CLIENT: Connect button pressed. RobotClient.connect() called.")
        else:
            logger.info("CLIENT: Attempting to disconnect...")
            if self.is_autonav_active: self.toggle_autonav() # Stop autonav if active
            self.robot_client.disconnect()
            self.connect_button.setText("Connect")
            self.autonav_button.setEnabled(False)
            if hasattr(self, 'pid_tuning_group'): self.pid_tuning_group.setEnabled(False)
            self.set_pickup_button.setEnabled(False)   # Disable on disconnect
            self.set_dropoff_button.setEnabled(False) # Disable on disconnect
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

    def on_speed_change(self, slider_value):
        # Map slider_value (0-100) to actual speed (75-100)
        min_actual_speed = 75
        max_actual_speed = 100
        actual_speed_range = max_actual_speed - min_actual_speed
        
        self.current_speed = int(min_actual_speed + (slider_value / 100.0) * actual_speed_range)
        
        # We will now rely on the keyPressEvent to send the updated speed
        # with the drive commands. This avoids sending SET_SPEED proactively,
        # which might be causing the robot to move unintentionally on the server.
        logger.debug(f"CLIENT: Slider value {slider_value} -> Mapped speed {self.current_speed}. Speed will be applied on next drive command.")
        self.setFocus() # Ensure main window regains focus after slider interaction

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
        self.setFocus() # Re-focus main window after applying

    def apply_distance_pid_settings(self):
        if not self.is_connected: return
        try:
            payload = {p: float(self.distance_pid_inputs[p].text()) for p in ["Kp", "Ki", "Kd"]}
            self.robot_client.send_command("SET_NAV_DISTANCE_PID", payload)
            logger.info(f"CLIENT: Sent SET_NAV_DISTANCE_PID with {payload}")
        except ValueError: logger.error("Invalid input for distance PID.")
        self.setFocus() # Re-focus main window after applying

    def set_overhead_target_pose(self, target_name: str):
        if not self.is_connected:
            logger.warning("Cannot set target pose: Not connected.")
            return
        self.robot_client.send_command("SET_OVERHEAD_TARGET", {"target_name": target_name})
        logger.info(f"CLIENT: Sent SET_OVERHEAD_TARGET for '{target_name}'")

    def closeEvent(self, event):
        logger.info("CLIENT: MainWindow closeEvent triggered. Disconnecting client.")
        self.robot_client.disconnect() # Ensure client disconnects when window is closed
        super().closeEvent(event) 