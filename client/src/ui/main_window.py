from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QSlider, QGridLayout, QSizePolicy,
                             QFormLayout, QGroupBox, QLineEdit, QMessageBox)
from PyQt6.QtCore import Qt, QTimer, QEvent
from PyQt6.QtGui import QImage, QPixmap, QKeyEvent, QDoubleValidator, QCloseEvent
import cv2
import numpy as np
import logging
from typing import Optional, Dict, Any
import os
import sys

# --- Python Path Modification & Config Import ---
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
if project_root not in sys.path:
    sys.path.append(project_root)

from client.src.network.client import RobotClient
from client.src.network.direct_warehouse_feed import DirectWarehouseFeedReceiver
from client.src.utils import config as client_config
from client.src import config as client_app_config

logger = logging.getLogger(__name__)

class KeyDisplay(QLabel):
    """
A custom QLabel widget to visually represent a keyboard key, changing appearance
    when active (pressed).
    """
    def __init__(self, key_text: str, width: int = 35, height: int = 35):
        """
        Initializes the KeyDisplay widget.

        Args:
            key_text: Text to display on the key (e.g., "W", "↑").
            width: Fixed width of the key display.
            height: Fixed height of the key display.
        """
        super().__init__(key_text)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFixedSize(width, height)
        self._active = False # Internal state for active
        self._update_style() # Set initial style
        
    def set_active(self, active: bool):
        """
        Sets the active state of the key display and updates its style accordingly.

        Args:
            active: True if the key is considered pressed, False otherwise.
        """
        if self._active != active:
            self._active = active
            self._update_style()
            
    def _update_style(self):
        """Applies styling based on the current active state."""
        if self._active:
            self.setStyleSheet("""
                QLabel {
                    background-color: #4CAF50; /* Green for active */
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
                    background-color: #2c2c2c; /* Dark grey for inactive */
                    color: white;
                    border: 2px solid #3c3c3c;
                    border-radius: 4px;
                    font-size: 14px;
                    font-weight: bold;
                }
            """)

class MainWindow(QMainWindow):
    """
    The main window for the Forklift Control Client application.

    This class sets up the entire UI, including video feeds, control elements
    (buttons, sliders, key displays), and PID tuning inputs. It handles user interactions
    (keyboard presses, button clicks), manages connections to the robot server
    (via RobotClient) and a direct warehouse camera feed (via DirectWarehouseFeedReceiver),
    and updates the UI based on received data and application state.
    """
    def __init__(self):
        """Initializes the MainWindow, sets up UI components, and connects signals/slots."""
        super().__init__()
        self.setWindowTitle(client_app_config.WINDOW_TITLE)
        self.setMinimumSize(1350, 720) # Adjusted for dual feeds, controls, servo grid
        self.setGeometry(100, 100, 1400, 750) # Default position and size

        # Enable strong focus to capture key events directly on the main window
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setFocus() # Ensure the main window has focus initially
        
        # Network clients for communication
        self.robot_client = RobotClient(
            host=client_app_config.SERVER_HOST,
            command_port=client_app_config.COMMAND_PORT,
            onboard_video_port=client_app_config.VIDEO_PORT,
            overhead_video_port=client_app_config.OVERHEAD_VIDEO_PORT
        )
        self.direct_feed_receiver = DirectWarehouseFeedReceiver(
            host=client_app_config.DIRECT_WAREHOUSE_VIDEO_HOST,
            port=client_app_config.DIRECT_WAREHOUSE_VIDEO_PORT
        )
        
        # State variables
        self.is_pi_connected = False # Tracks connection to the main RobotClient (Pi)
        self.is_autonav_active = False
        self.overhead_feed_source = "pi"  # "pi" (via RobotClient) or "direct" (via DirectWarehouseFeedReceiver)
        self.selected_servo_pin = client_config.FORK_SERVO_A_PIN # Default selected servo
        self.current_speed = 0 # Speed for manual drive, updated by slider

        self.servo_key_displays: Dict[int, KeyDisplay] = {} # Stores [pin_number -> KeyDisplay widget]
        
        self._setup_ui()
        self._connect_signals()
        
        # Timer for updating video feeds from network clients
        self.video_update_timer = QTimer(self)
        self.video_update_timer.timeout.connect(self.update_video_feeds_display)
        self.video_update_timer.start(int(1000 / client_app_config.VIDEO_FPS))  # Target ~30 FPS (e.g., 33ms interval)
        
        self.on_speed_change(self.speed_slider.value()) # Initialize current_speed based on slider default
        self._update_servo_selection_display() # Initial UI update for servo selection
        self._update_connection_status_ui() # Initial UI state for connection buttons/features
        logger.info("MainWindow initialized.")

    def _setup_ui(self):
        """Creates and arranges all UI widgets within the main window."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # --- Video Display Area (Two feeds side-by-side) ---
        video_area_layout = QHBoxLayout()
        self.onboard_video_label, onboard_video_group = self._create_video_feed_group("Onboard Camera (Pi)", "Waiting for Onboard Stream...")
        self.overhead_video_label, overhead_video_group = self._create_video_feed_group("Overhead Camera (Warehouse)", "Waiting for Overhead Stream...")
        video_area_layout.addWidget(onboard_video_group, stretch=1)
        video_area_layout.addWidget(overhead_video_group, stretch=1)
        main_layout.addLayout(video_area_layout, stretch=2) # Video area takes more vertical space
        
        # --- Control Panel Area (Horizontal layout for various control groups) ---
        control_panel_container = QWidget()
        control_panel_layout = QHBoxLayout(control_panel_container)
        main_layout.addWidget(control_panel_container, stretch=1) # Control panel takes less vertical space
        
        # Speed Control Slider
        speed_group = self._create_speed_control_group()
        control_panel_layout.addWidget(speed_group)
        
        # WASD & Servo Arrow Key Displays
        key_display_group = self._create_key_display_group()
        control_panel_layout.addWidget(key_display_group)

        # Servo Selection Grid
        servo_selection_group = self._create_servo_selection_group()
        control_panel_layout.addWidget(servo_selection_group)

        # Action Buttons (Connect, Autonav, E-Stop, Switch Feed)
        action_buttons_group = self._create_action_buttons_group()
        control_panel_layout.addWidget(action_buttons_group)
        
        # PID Tuning UI
        pid_tuning_group_widget = self._create_pid_tuning_group()
        control_panel_layout.addWidget(pid_tuning_group_widget)

    def _create_video_feed_group(self, title: str, initial_text: str) -> Tuple[QLabel, QGroupBox]:
        """Helper to create a QGroupBox containing a QLabel for a video feed."""
        group = QGroupBox(title)
        layout = QVBoxLayout(group)
        label = QLabel(initial_text)
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        label.setMinimumSize(int(client_app_config.VIDEO_WIDTH / 2.5), int(client_app_config.VIDEO_HEIGHT / 2.5)) # Scaled minimum size
        label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored) # Allow scaling
        label.setStyleSheet("background-color: black; color: grey; font-size: 16px;")
        layout.addWidget(label)
        return label, group

    def _create_speed_control_group(self) -> QGroupBox:
        """Creates the GroupBox for the speed slider."""
        group = QGroupBox("Speed")
        layout = QVBoxLayout(group)
        self.speed_slider = QSlider(Qt.Orientation.Vertical)
        self.speed_slider.setRange(0, 100) # Slider value 0-100
        self.speed_slider.setValue(50)     # Default to middle
        self.speed_slider.setMinimumHeight(180)
        self.speed_slider.setTickInterval(10)
        self.speed_slider.setTickPosition(QSlider.TickPosition.TicksRight)
        
        self.current_speed_label = QLabel(f"Set: {self.speed_slider.value()} (Actual: ...)") # Will be updated by on_speed_change
        self.current_speed_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        layout.addWidget(self.current_speed_label)
        layout.addWidget(self.speed_slider, alignment=Qt.AlignmentFlag.AlignCenter)
        layout.addStretch(1)
        return group

    def _create_key_display_group(self) -> QGroupBox:
        """Creates the GroupBox for WASD, Space, and Servo Arrow key displays."""
        group = QGroupBox("Manual Controls")
        layout = QGridLayout(group)
        self.w_key = KeyDisplay("W")
        self.a_key = KeyDisplay("A")
        self.s_key = KeyDisplay("S")
        self.d_key = KeyDisplay("D")
        self.space_key = KeyDisplay("SPACE (E-STOP)", width=150, height=35)
        self.up_arrow_key = KeyDisplay("↑ (Fork Up)")
        self.down_arrow_key = KeyDisplay("↓ (Fork Down)")
        
        layout.addWidget(self.w_key, 0, 1)          # Row 0, Col 1
        layout.addWidget(self.a_key, 1, 0)          # Row 1, Col 0
        layout.addWidget(self.s_key, 1, 1)          # Row 1, Col 1
        layout.addWidget(self.d_key, 1, 2)          # Row 1, Col 2
        layout.addWidget(self.space_key, 2, 0, 1, 3) # Row 2, span 3 columns
        layout.addWidget(QLabel("Selected Servo:"), 0, 3) # Label for servo arrows
        layout.addWidget(self.up_arrow_key, 0, 4)    # Row 0, Col 4
        layout.addWidget(self.down_arrow_key, 1, 4)  # Row 1, Col 4
        layout.setColumnStretch(0,1) # Add some spacing around keys
        layout.setColumnStretch(1,1)
        layout.setColumnStretch(2,1)
        layout.setColumnStretch(3,1)
        layout.setColumnStretch(4,1)
        return group

    def _create_servo_selection_group(self) -> QGroupBox:
        """Creates the GroupBox for the 3x2 servo selection grid using number keys."""
        group = QGroupBox("Select Servo (Keys 1-6)")
        grid_layout = QGridLayout(group)
        
        # Order of servos for UI display (top-to-bottom, left-to-right in pairs)
        # This should match the UI_SERVO_ORDER_MAPPING from config if a specific visual layout is desired.
        # Current mapping: 1=A, 2=B, 3=C, 4=D, 5=E, 6=F
        # Visual Grid:  E(5) F(6)
        #               C(3) D(4)
        #               A(1) B(2)
        display_positions = [
            (client_config.FORK_SERVO_E_PIN, "5", 0, 0), (client_config.FORK_SERVO_F_PIN, "6", 0, 1),
            (client_config.FORK_SERVO_C_PIN, "3", 1, 0), (client_config.FORK_SERVO_D_PIN, "4", 1, 1),
            (client_config.FORK_SERVO_A_PIN, "1", 2, 0), (client_config.FORK_SERVO_B_PIN, "2", 2, 1),
        ]

        for pin, text_label, row, col in display_positions:
            display_widget = KeyDisplay(text_label)
            self.servo_key_displays[pin] = display_widget
            grid_layout.addWidget(display_widget, row, col)
        return group

    def _create_action_buttons_group(self) -> QGroupBox:
        """Creates the GroupBox for action buttons like Connect, Autonav, E-Stop."""
        group = QGroupBox("Actions")
        layout = QVBoxLayout(group)
        
        self.connect_button = QPushButton("Connect to Pi")
        layout.addWidget(self.connect_button)
        
        self.autonav_button = QPushButton("Start Auto-Nav (Pickup)") # Default to pickup
        layout.addWidget(self.autonav_button)

        # Buttons for setting overhead target poses dynamically
        self.set_pickup_button = QPushButton("Capture Pickup Pose")
        layout.addWidget(self.set_pickup_button)
        self.set_dropoff_button = QPushButton("Capture Dropoff Pose")
        layout.addWidget(self.set_dropoff_button)
        
        self.emergency_button = QPushButton("Emergency Stop (Space)")
        self.emergency_button.setStyleSheet("background-color: #d32f2f; color: white; font-weight: bold;") # Red color
        layout.addWidget(self.emergency_button)

        self.switch_feed_button = QPushButton("Switch to Direct Overhead")
        layout.addWidget(self.switch_feed_button)
        layout.addStretch(1)
        return group

    def _create_pid_tuning_group(self) -> QGroupBox:
        """Creates the GroupBox for PID tuning inputs."""
        self.pid_tuning_group = QGroupBox("Navigation PID Tuning")
        self.pid_tuning_group.setMaximumWidth(300) # Constrain width
        pid_form_layout = QFormLayout(self.pid_tuning_group)
        pid_form_layout.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.ExpandingFieldsGrow)
        
        self.pid_inputs: Dict[str, Dict[str, QLineEdit]] = {
            "point_turning": {},
            "xy_distance": {},
            "final_orientation": {}
        }
        pid_display_names = {
            "point_turning": "Point Turn PID",
            "xy_distance": "XY Drive PID",
            "final_orientation": "Final Angle PID"
        }

        for pid_name_internal, display_name in pid_display_names.items():
            pid_label = QLabel(f"<b>{display_name}</b>")
            pid_form_layout.addRow(pid_label)
            for param in ["Kp", "Ki", "Kd"]:
                line_edit = QLineEdit()
                line_edit.setValidator(QDoubleValidator(0, 10000.0, 4)) # Allow up to 4 decimal places
                line_edit.setPlaceholderText("0.0")
                # Store the QLineEdit for later access
                self.pid_inputs[pid_name_internal][param] = line_edit
                pid_form_layout.addRow(f"  {param}:", line_edit)
            
            apply_button = QPushButton(f"Apply {display_name}")
            # Use a lambda to pass the internal PID name to the handler
            apply_button.clicked.connect(lambda checked=False, name=pid_name_internal: self.apply_pid_settings(name))
            pid_form_layout.addRow(apply_button)
            pid_form_layout.addRow(QLabel("")) # Spacer

        return self.pid_tuning_group

    def _connect_signals(self):
        """Connects widget signals to their respective handler slots."""
        self.speed_slider.valueChanged.connect(self.on_speed_change)
        self.connect_button.clicked.connect(self.toggle_pi_connection_state)
        self.autonav_button.clicked.connect(self.toggle_autonav_state)
        self.set_pickup_button.clicked.connect(lambda: self.handle_set_overhead_target_pose("pickup"))
        self.set_dropoff_button.clicked.connect(lambda: self.handle_set_overhead_target_pose("dropoff"))
        self.emergency_button.clicked.connect(self.trigger_emergency_stop)
        self.switch_feed_button.clicked.connect(self.toggle_overhead_feed_source)
        
        # Connect editingFinished for PID QLineEdits to regain focus on main window
        for pid_params in self.pid_inputs.values():
            for line_edit in pid_params.values():
                line_edit.editingFinished.connect(self.setFocus)
        logger.debug("MainWindow signals connected.")

    def _update_servo_selection_display(self):
        """Highlights the currently selected servo KeyDisplay widget in the UI."""
        for pin, display_widget in self.servo_key_displays.items():
            display_widget.set_active(pin == self.selected_servo_pin)
        logger.debug(f"Servo selection display updated. Selected: Pin {self.selected_servo_pin}")
        
    def keyPressEvent(self, event: QKeyEvent):
        """Handles key press events for robot control and UI interaction."""
        if event.isAutoRepeat(): # Ignore auto-repeated key presses for continuous actions
            return

        key = event.key()
        key_char = event.text() # For character keys like '1'-'6'

        # Servo Selection Keys (1-6) - always active if main window has focus
        if key_char in client_config.SERVO_KEY_TO_PIN_MAPPING:
            selected_pin = client_config.SERVO_KEY_TO_PIN_MAPPING[key_char]
            if self.selected_servo_pin != selected_pin:
                self.selected_servo_pin = selected_pin
                self._update_servo_selection_display()
                logger.info(f"Selected servo: Pin {self.selected_servo_pin} (via key '{key_char}')")
            self.setFocus() # Ensure main window retains focus after selection
            event.accept()
            return

        # E-Stop (Space bar) - always active if connected
        if key == Qt.Key.Key_Space:
            self.space_key.set_active(True)
            if self.is_pi_connected: # Only send E-Stop if connected to Pi
                self.trigger_emergency_stop()
            event.accept()
            return
        
        # The following controls require connection to the Pi and Autonav to be OFF
        if not self.is_pi_connected or self.is_autonav_active:
            super().keyPressEvent(event) # Pass to parent if not handling
            return

        # Servo (Fork) Control with Up/Down Arrows for the currently selected servo
        if key == Qt.Key.Key_Up:
            self.up_arrow_key.set_active(True)
            logger.info(f"Sending SERVO UP command for selected pin {self.selected_servo_pin}")
            self.robot_client.send_command('servo', {'pin': self.selected_servo_pin, 'action': 'UP'})
        elif key == Qt.Key.Key_Down:
            self.down_arrow_key.set_active(True)
            logger.info(f"Sending SERVO DOWN command for selected pin {self.selected_servo_pin}")
            self.robot_client.send_command('servo', {'pin': self.selected_servo_pin, 'action': 'DOWN'})
        # WASD Drive Controls (using direct move commands)
        elif key == Qt.Key.Key_W: 
            self.w_key.set_active(True); 
            self.robot_client.send_command("drive", {"direction": "MOVE", "forward_component": self.current_speed, "turn_component": 0})
        elif key == Qt.Key.Key_A: 
            self.a_key.set_active(True); 
            self.robot_client.send_command("drive", {"direction": "MOVE", "forward_component": 0, "turn_component": self.current_speed})
        elif key == Qt.Key.Key_S: 
            self.s_key.set_active(True); 
            self.robot_client.send_command("drive", {"direction": "MOVE", "forward_component": -self.current_speed, "turn_component": 0})
        elif key == Qt.Key.Key_D: 
            self.d_key.set_active(True); 
            self.robot_client.send_command("drive", {"direction": "MOVE", "forward_component": 0, "turn_component": -self.current_speed})
        else:
            super().keyPressEvent(event) # Important for other keys (e.g., tab, focus changes)
            return
        event.accept() # Mark event as handled
            
    def keyReleaseEvent(self, event: QKeyEvent):
        """Handles key release events to stop actions or update UI."""
        if event.isAutoRepeat():
            return

        key = event.key()
        
        # Deactivate visual feedback for Space key
        if key == Qt.Key.Key_Space:
            self.space_key.set_active(False)
            event.accept()
            return

        # The following controls require connection to the Pi and Autonav to be OFF
        if not self.is_pi_connected or self.is_autonav_active:
            super().keyReleaseEvent(event)
            return

        # Servo Control Release (Up/Down arrows)
        if key == Qt.Key.Key_Up:
            self.up_arrow_key.set_active(False)
            # Server-side servo commands are stepped; no explicit stop needed from client for current design.
        elif key == Qt.Key.Key_Down:
            self.down_arrow_key.set_active(False)
        # WASD Release - send a stop command for the specific action type (or general stop)
        elif key in [Qt.Key.Key_W, Qt.Key.Key_S, Qt.Key.Key_A, Qt.Key.Key_D]:
            if key == Qt.Key.Key_W: self.w_key.set_active(False)
            elif key == Qt.Key.Key_A: self.a_key.set_active(False)
            elif key == Qt.Key.Key_S: self.s_key.set_active(False)
            elif key == Qt.Key.Key_D: self.d_key.set_active(False)
            # Send a command to stop motors only if no other WASD key is still pressed.
            # This requires checking the state of other KeyDisplay widgets.
            if not (self.w_key._active or self.a_key._active or self.s_key._active or self.d_key._active):
                 self.robot_client.send_command("drive", {"direction": "MOVE", "forward_component": 0, "turn_component": 0})
                 logger.debug("All WASD keys released, sent MOVE stop command.")
        else:
            super().keyReleaseEvent(event)
            return
        event.accept()

    def _update_connection_status_ui(self):
        """Updates UI elements based on the Pi connection state."""
        self.connect_button.setText("Disconnect from Pi" if self.is_pi_connected else "Connect to Pi")
        self.autonav_button.setEnabled(self.is_pi_connected)
        if hasattr(self, 'pid_tuning_group'): # Check if PID group exists
             self.pid_tuning_group.setEnabled(self.is_pi_connected)
        self.set_pickup_button.setEnabled(self.is_pi_connected)
        self.set_dropoff_button.setEnabled(self.is_pi_connected)
        self.switch_feed_button.setEnabled(self.is_pi_connected)

        if not self.is_pi_connected:
            self.autonav_button.setText("Start Auto-Nav (Pickup)")
            self.autonav_button.setStyleSheet("") # Reset style
            self.is_autonav_active = False # Ensure autonav state is reset
            self.switch_feed_button.setText("Switch to Direct Overhead")
            self.overhead_feed_source = "pi" # Reset to default source
            # Clear video feed labels
            self.onboard_video_label.setText("Disconnected from Pi")
            self.onboard_video_label.setStyleSheet("background-color: black; color: grey; font-size: 16px;")
            self.overhead_video_label.setText("Disconnected from Pi / Direct Feed Inactive")
            self.overhead_video_label.setStyleSheet("background-color: black; color: grey; font-size: 16px;")
            # Deactivate all key displays
            for kd in [self.w_key, self.a_key, self.s_key, self.d_key, self.space_key, self.up_arrow_key, self.down_arrow_key]:
                kd.set_active(False)
        else:
             self.onboard_video_label.setText("Waiting for Onboard Stream...")
             self.overhead_video_label.setText("Waiting for Pi Overhead Stream...")

    def toggle_pi_connection_state(self):
        """Handles connect/disconnect requests for the main Pi RobotClient."""
        if not self.is_pi_connected:
            logger.info("MainWindow: Attempting to connect to Pi (RobotClient)...")
            self.robot_client.connect()
            self.is_pi_connected = True
            logger.info("MainWindow: RobotClient.connect() called.")
            # When connecting to Pi, ensure direct feed is off and UI reflects Pi as source
            if self.direct_feed_receiver.is_connected:
                self.direct_feed_receiver.disconnect()
            self.overhead_feed_source = "pi"
        else:
            logger.info("MainWindow: Attempting to disconnect from Pi (RobotClient)...")
            if self.is_autonav_active: # Stop autonav if active before disconnecting
                self.toggle_autonav_state() 
            self.robot_client.disconnect()
            self.is_pi_connected = False
            logger.info("MainWindow: RobotClient.disconnect() called.")
            # Also disconnect direct feed if it was active, as Pi connection is primary
            if self.direct_feed_receiver.is_connected:
                self.direct_feed_receiver.disconnect()
        
        self._update_connection_status_ui()
        self.setFocus() # Ensure main window retains focus

    def toggle_overhead_feed_source(self):
        """Switches the source of the overhead video feed between Pi and Direct TCP."""
        if not self.is_pi_connected: # Pi connection is required to enable this button
            logger.warning("MainWindow: Cannot switch overhead source, Pi is not connected.")
            QMessageBox.warning(self, "Connection Error", "Connect to the Pi server first before switching overhead feed sources.")
            return

        if self.overhead_feed_source == "pi":
            logger.info("MainWindow: Switching overhead feed source from Pi to DIRECT.")
            # Attempt to connect to the direct feed
            self.direct_feed_receiver.connect() 
            # Check if direct feed connection was successful (it runs in a thread)
            # For now, we optimistically switch and let the update_video_feeds handle display.
            self.overhead_feed_source = "direct"
            self.switch_feed_button.setText("Switch to Pi Overhead")
            self.overhead_video_label.setText("Attempting Direct Overhead Stream...")
        elif self.overhead_feed_source == "direct":
            logger.info("MainWindow: Switching overhead feed source from DIRECT to Pi.")
            self.direct_feed_receiver.disconnect() # Disconnect direct feed
            self.overhead_feed_source = "pi"
            self.switch_feed_button.setText("Switch to Direct Overhead")
            self.overhead_video_label.setText("Waiting for Pi Overhead Stream...")
        
        self._update_connection_status_ui() # Reflect changes in button states if any
        self.setFocus()

    def trigger_emergency_stop(self):
        """Sends an emergency stop command to the robot and updates UI."""
        logger.critical("MainWindow: EMERGENCY STOP triggered!")
        self.robot_client.send_command("stop_all", {}) # Send E-STOP command
        if self.is_autonav_active:
            self.is_autonav_active = False
            self.autonav_button.setText("Start Auto-Nav (Pickup)")
            self.autonav_button.setStyleSheet("") # Reset style
            logger.info("MainWindow: Auto-Nav was active, deactivated due to Emergency Stop.")
        QMessageBox.critical(self, "Emergency Stop", "Emergency Stop command sent to robot!")
        self.setFocus()
        
    def toggle_autonav_state(self):
        """Toggles autonomous navigation state and sends command to server."""
        if not self.is_pi_connected: 
            QMessageBox.warning(self, "Connection Error", "Not connected to the Pi server.")
            return
        
        # Determine which target to use if starting autonav (e.g., from a dropdown or cycle)
        # For now, let's assume it tries to start for "pickup" if not active, or stops if active.
        target_for_autonav = "pickup" # Could be made configurable

        self.robot_client.send_command("toggle_autonav", {"target": target_for_autonav})
        self.is_autonav_active = not self.is_autonav_active # Assume server toggles successfully
        
        if self.is_autonav_active:
            self.autonav_button.setText(f"Stop Auto-Nav ({target_for_autonav.capitalize()})")
            self.autonav_button.setStyleSheet("background-color: #FFA500; color: black;") # Orange for active
            logger.info(f"MainWindow: Auto-Nav toggled ON (target: {target_for_autonav}). Command sent.")
        else:
            self.autonav_button.setText("Start Auto-Nav (Pickup)")
            self.autonav_button.setStyleSheet("") # Reset style
            logger.info("MainWindow: Auto-Nav toggled OFF. Command sent.")
        self.setFocus()

    def on_speed_change(self, slider_value: int):
        """
        Called when the speed slider value changes.
        Maps the slider value (0-100) to an actual speed range (e.g., 75-100)
        and updates `self.current_speed`.
        """
        min_actual_speed = client_app_config.MIN_DRIVE_SPEED # Example: 75 from a hypothetical config
        max_actual_speed = client_app_config.MAX_DRIVE_SPEED # Example: 100 from a hypothetical config
        
        # Ensure min_actual_speed is less than max_actual_speed to avoid division by zero or negative range
        if min_actual_speed >= max_actual_speed:
            min_actual_speed = 75  # Fallback values
        max_actual_speed = 100
            logger.warning(f"MainWindow: Invalid speed range in config (min_drive_speed >= max_drive_speed). Using default {min_actual_speed}-{max_actual_speed}.")

        actual_speed_range = max_actual_speed - min_actual_speed
        self.current_speed = int(min_actual_speed + (slider_value / 100.0) * actual_speed_range)
        self.current_speed_label.setText(f"Set: {slider_value} (Actual: {self.current_speed})")
        
        logger.debug(f"MainWindow: Speed slider value {slider_value} -> Mapped speed {self.current_speed}.")
        self.setFocus() # Ensure main window regains focus

    def _display_frame_on_label(self, frame: Optional[np.ndarray], video_label_widget: QLabel):
        """Displays a single video frame (NumPy array) on a QLabel, scaling it."""
        if frame is not None and frame.size > 0: # Check if frame is not None and not empty
            try:
            height, width, channel = frame.shape
                bytes_per_line = channel * width # channel is usually 3 for RGB
                # Create QImage from the NumPy array (assuming RGB format from network clients)
            q_image = QImage(frame.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
                
                # Scale pixmap to fit the label while keeping aspect ratio
                scaled_pixmap = pixmap.scaled(video_label_widget.size(), 
                                              Qt.AspectRatioMode.KeepAspectRatio, 
                                              Qt.TransformationMode.SmoothTransformation)
                video_label_widget.setPixmap(scaled_pixmap)
            except Exception as e:
                logger.error(f"MainWindow: Error processing/displaying frame: {e}", exc_info=True)
                video_label_widget.setText("Error Displaying Frame")
        # else: If frame is None, the label keeps its current text (e.g., "Waiting...") or last valid frame.
            
    def update_video_feeds_display(self):
        """Periodically called by QTimer to update both video feed displays."""
        # Update Onboard Video Feed (from RobotClient)
        if self.robot_client and self.robot_client.is_connected: 
            onboard_frame = self.robot_client.get_onboard_video_frame()
            self._display_frame_on_label(onboard_frame, self.onboard_video_label)
        elif not self.is_pi_connected: # If Pi is explicitly disconnected by user
             self.onboard_video_label.setText("Disconnected from Pi")
             self.onboard_video_label.setStyleSheet("background-color: black; color: grey; font-size: 16px;")

        # Update Overhead Video Feed (source depends on self.overhead_feed_source)
        if self.overhead_feed_source == "pi":
            if self.robot_client and self.robot_client.is_connected:
                overhead_pi_frame = self.robot_client.get_overhead_video_frame()
                self._display_frame_on_label(overhead_pi_frame, self.overhead_video_label)
            elif not self.is_pi_connected:
                 self.overhead_video_label.setText("Disconnected from Pi")
                 self.overhead_video_label.setStyleSheet("background-color: black; color: grey; font-size: 16px;")
        elif self.overhead_feed_source == "direct":
            # If direct feed is selected, get frames from DirectWarehouseFeedReceiver
            if not self.direct_feed_receiver.is_connected and \
               self.is_pi_connected and not (self.direct_feed_receiver.thread and self.direct_feed_receiver.thread.is_alive()):
                # Auto-attempt to connect direct feed if Pi is connected, switch is active, but feed isn't running
                logger.info("MainWindow: Direct feed selected but not running. Auto-attempting connection.")
                self.direct_feed_receiver.connect()
                self.overhead_video_label.setText("Attempting Direct Overhead Stream...") # Initial text while connecting
            
            direct_frame = self.direct_feed_receiver.get_video_frame()
            if direct_frame is not None:
                self._display_frame_on_label(direct_frame, self.overhead_video_label)
            elif not self.direct_feed_receiver.is_connected:
                self.overhead_video_label.setText("Direct Overhead: Disconnected / Error")
                self.overhead_video_label.setStyleSheet("background-color: black; color: #FF6347; font-size: 16px;") # Tomato color for error
            # else: if direct_frame is None but connected, keep last frame or "Attempting..."

    def apply_pid_settings(self, pid_name_internal: str):
        """Applies PID settings from the UI to the server for the specified PID controller."""
        if not self.is_pi_connected:
            QMessageBox.warning(self, "Connection Error", "Not connected to the Pi server.")
            return
        
        if pid_name_internal not in self.pid_inputs:
            logger.error(f"MainWindow: apply_pid_settings called with unknown PID name '{pid_name_internal}'")
            return

        try:
            payload = {
                "pid_name": pid_name_internal,
                "kp": float(self.pid_inputs[pid_name_internal]["Kp"].text()),
                "ki": float(self.pid_inputs[pid_name_internal]["Ki"].text()),
                "kd": float(self.pid_inputs[pid_name_internal]["Kd"].text())
            }
            self.robot_client.send_command("set_nav_pid", payload)
            logger.info(f"MainWindow: Sent SET_NAV_PID command for '{pid_name_internal}' with payload: {payload}")
            QMessageBox.information(self, "PID Update", f"PID settings for {pid_name_internal.replace('_', ' ').title()} sent.")
        except ValueError:
            logger.error(f"MainWindow: Invalid input for {pid_name_internal} PID values.")
            QMessageBox.critical(self, "Input Error", f"Invalid numeric input for {pid_name_internal} PID parameters.")
        except Exception as e:
            logger.error(f"MainWindow: Error sending PID settings for {pid_name_internal}: {e}", exc_info=True)
            QMessageBox.critical(self, "Error", f"Failed to send PID settings: {e}")
        self.setFocus() # Re-focus main window

    def handle_set_overhead_target_pose(self, target_type: str):
        """
        Sends a command to the server to set an overhead target pose.
        The server is expected to use its current primary ArUco marker detection from
        the *onboard* camera (if available) or a predefined logic to define the target.
        This client-side action just triggers the server to capture/set that pose.

        Args:
            target_type: "pickup" or "dropoff", indicating which target to set.
        """
        if not self.is_pi_connected:
            QMessageBox.warning(self, "Connection Error", "Not connected to the Pi server.")
            return
        
        # Payload for the server to identify which target type to set/capture.
        # The server-side handler will determine the actual X, Y, Theta.
        payload = {"target_type": target_type}
        self.robot_client.send_command("set_overhead_target", payload)
        logger.info(f"MainWindow: Sent SET_OVERHEAD_TARGET command for target type: '{target_type}'. Server will capture pose.")
        QMessageBox.information(self, "Target Pose", f"Request to set '{target_type}' pose sent to server.")
        self.setFocus()

    def closeEvent(self, event: QCloseEvent):
        """Handles the main window close event to ensure graceful shutdown of network clients."""
        logger.info("MainWindow: closeEvent triggered. Initiating shutdown of network clients.")
        
        # Disconnect RobotClient (to Pi)
        if self.robot_client:
            logger.debug("MainWindow: Disconnecting RobotClient...")
            self.robot_client.disconnect() # This method waits for its thread
        
        # Disconnect DirectWarehouseFeedReceiver
        if self.direct_feed_receiver:
            logger.debug("MainWindow: Disconnecting DirectWarehouseFeedReceiver...")
            self.direct_feed_receiver.disconnect() # This method waits for its thread
            
        self.video_update_timer.stop() # Stop the UI update timer
        logger.info("MainWindow: All network clients disconnected and timer stopped. Accepting close event.")
        super().closeEvent(event) 