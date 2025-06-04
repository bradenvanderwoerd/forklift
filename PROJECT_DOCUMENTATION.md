# Forklift Project: Comprehensive Documentation

## 1. Project Overview

The Forklift project is a sophisticated remote-controlled and autonomous robot car system. The core of the robot is a Raspberry Pi acting as a server, managing hardware interactions and network communications. A client application, designed for macOS, provides a graphical user interface for operators to control the robot manually, view real-time video feeds, and initiate autonomous navigation tasks.

The system leverages a dual-camera setup: an onboard camera for a first-person perspective and an overhead warehouse camera for global localization. Navigation, particularly for autonomous tasks, relies on the detection of ArUco markers.

## 2. Key Features

*   **Distributed Architecture:**
    *   **Raspberry Pi Server:** Central hub for hardware control (motors, servos), video processing (onboard and overhead camera feeds), and WebSocket-based network communication.
    *   **macOS Client:** Provides a rich PyQt6-based GUI for manual robot operation, displaying multiple video streams, configuring autonomous tasks, and dynamically tuning PID parameters for navigation.

*   **Advanced Camera and Video System:**
    *   **Onboard Camera (PiCamera2):** Mounted on the forklift, offering a first-person operational view. The field of view is optimized using `picamera2`'s raw stream capabilities combined with `ScalerCrop` to utilize the full sensor area.
    *   **Overhead Warehouse Camera:** A fixed camera providing a top-down view of the robot's operational environment, crucial for ArUco marker-based localization and path planning.
    *   **Flexible Video Feed Delivery:** The client can receive the overhead camera feed either relayed through the Raspberry Pi (which may include processed information like marker overlays) or by connecting directly to the warehouse camera's IP source for potentially lower latency.

*   **Robotics and Control:**
    *   **Multi-Servo Control:** Supports up to six individually addressable servos, allowing for complex attachments and manipulations (e.g., multi-stage forklift mechanisms, grippers). Each servo has configurable parameters for initial angle, movement range (up/down positions), and step increments.
    *   **Differential Drive Motor Control:** Enables precise and agile movement, including forward/backward motion and in-place turning.
    *   **Emergency Stop:** A critical safety feature accessible via the client UI (and spacebar shortcut) to halt all robot motion immediately.

*   **Autonomous Navigation (ArUco Markers):**
    *   **Localization:** Utilizes the overhead camera to detect ArUco markers on the robot itself and potentially at fixed locations (targets) within the warehouse.
    *   **State-Machine Navigation:** Implements a navigation controller with a state machine (e.g., `ALIGNING_TO_POINT`, `MOVING_TO_POINT`, `FINAL_ADJUSTMENT`) to guide the robot autonomously to predefined target coordinates (e.g., pickup and dropoff locations).
    *   **PID Control:** Employs PID (Proportional-Integral-Derivative) controllers for accurate distance tracking, point turning, and final orientation adjustments during autonomous maneuvers.
    *   **Dynamic Target Setting:** The client can command the server to capture the robot's current pose (based on overhead localization) and set it as a "pickup" or "dropoff" target for future autonomous navigation.

*   **Network and Configuration:**
    *   **Real-time Communication:** Uses WebSockets for low-latency bidirectional communication between the client and server, facilitating responsive control and timely feedback.
    *   **Dynamic PID Tuning:** The client UI allows operators to adjust PID controller gains (Kp, Ki, Kd) for different navigation aspects (turning, driving, final orientation) in real-time and send these updates to the server.
    *   **Environment Configuration (`.env` file):** Centralizes critical configuration parameters (network ports, camera URLs, GPIO pin assignments, PID gains, etc.) in a `.env` file at the project root. This allows for easy adaptation to different hardware setups and network environments without code modification. A `.env.template` file guides users on required settings.

## 3. Project Structure

```
forklift/
├── .env.template         # Template for environment variables, guiding .env creation
├── client/                 # macOS client application code
│   ├── src/
│   │   ├── network/      # Client-side network (RobotClient, DirectWarehouseFeedReceiver)
│   │   ├── ui/           # PyQt6 user interface (MainWindow, custom widgets like KeyDisplay)
│   │   ├── utils/        # Client-specific utilities, configuration (client/src/config.py)
│   │   └── main.py       # Client application entry point
│   ├── requirements.txt  # Python dependencies for the client
│   └── README.md         # Client-specific information (if any - currently minimal)
│
├── server/                 # Raspberry Pi server application code
│   ├── src/
│   │   ├── controllers/    # Hardware abstraction (MotorController, ServoController, NavigationController)
│   │   ├── localization/   # ArUco marker detection and pose estimation (OverheadLocalizer)
│   │   ├── network/        # Server-side network (CommandServer, VideoStreamer, OverheadStreamer, WarehouseCameraClient)
│   │   ├── utils/          # Server-specific utilities, configuration (server/src/utils/config.py), helpers
│   │   └── main.py         # Server application entry point (ForkliftServer orchestration)
│   ├── requirements.txt  # Python dependencies for the server
│   └── README.md         # Server-specific setup, troubleshooting, and details
│
├── shared/                 # Placeholder for potential future code shared between client and server
│   └── ...
│
└── README.md               # Main project README (to be written by the user)
└── PROJECT_DOCUMENTATION.md # This file
```

## 4. Development and Operational Setup

### 4.1. Prerequisites

*   **General:**
    *   Python 3.8+
    *   Git for version control.
*   **For Server (Raspberry Pi):**
    *   Raspberry Pi (Model 3B+ or newer recommended, e.g., Pi 4).
    *   Raspberry Pi OS (Bullseye or later recommended).
    *   `picamera2` library and its underlying system dependencies (e.g., `libcamera-apps`).
    *   Python libraries: `RPi.GPIO`, `opencv-python`, `numpy`, `websockets`, `python-dotenv`, `Pillow`, `v4l2-python3`, `piexif`.
    *   Hardware:
        *   Onboard camera (e.g., Raspberry Pi Camera Module V2/IMX219).
        *   Separate IP camera or USB camera for overhead view (if using direct feed or local processing on Pi).
        *   Motor driver (e.g., L298N) compatible with connected DC motors.
        *   Servo motors (up to 6).
        *   Appropriate wiring and power supply for all components.
    *   Camera interface enabled in `raspi-config`.
    *   User added to the `gpio` group for GPIO access.
*   **For Client (macOS):**
    *   macOS.
    *   Python libraries: `PyQt6`, `opencv-python`, `numpy`, `websockets`, `python-dotenv`, `colorama`.

### 4.2. General Project Setup

1.  **Clone the Repository:**
    ```bash
    git clone <your_repository_url> forklift
    cd forklift
    ```

2.  **Environment Variables (`.env` file):**
    *   In the root directory of the `forklift` project, create a `.env` file.
    *   You can use `.env.template` as a starting point.
    *   This file is crucial for configuring network addresses, ports, GPIO pins (if overridden from defaults), PID gains, camera settings, and other sensitive or environment-specific parameters without hardcoding them.
    *   **Example essential `.env` content:**
        ```env
        # Server Network Settings (used by server/src/utils/config.py)
        SERVER_HOST=0.0.0.0                 # Listen on all interfaces on the Pi
        COMMAND_PORT=3456
        VIDEO_PORT=3457                     # For onboard camera stream from Pi
        OVERHEAD_VIDEO_PORT=3458            # For Pi-relayed overhead camera stream

        # Client Network Settings (used by client/src/config.py)
        # Ensure SERVER_HOST here matches the Pi's actual IP address on your network
        # SERVER_HOST=192.168.x.x             # Example: Client needs Pi's IP

        # Direct Warehouse Camera Feed (if used)
        DIRECT_WAREHOUSE_VIDEO_HOST=192.168.x.y # IP of the external warehouse camera
        DIRECT_WAREHOUSE_VIDEO_PORT=5001        # Port of the external warehouse camera

        # Motor Pins (BCM numbering, example for L298N)
        MOTOR_LEFT_EN=12
        MOTOR_LEFT_FWD=16
        MOTOR_LEFT_BWD=20
        MOTOR_RIGHT_EN=13
        MOTOR_RIGHT_FWD=19
        MOTOR_RIGHT_BWD=26

        # Servo Pins & Config (Example for one servo, add others as needed)
        SERVO_A_PIN=22
        SERVO_A_INITIAL=90
        SERVO_A_UP=135
        SERVO_A_DOWN=45

        # Other configurations like camera resolution, ArUco IDs, PID gains can also be here.
        # Refer to server/src/utils/config.py and client/src/config.py for all potential variables.
        ```

### 4.3. Server Setup (Raspberry Pi)

(Details primarily sourced from `server/README.md`)

1.  **Enable Camera Interface:**
    ```bash
    sudo raspi-config
    ```
    Navigate to `Interface Options` -> `Camera` and enable it. Reboot if prompted.

2.  **Install System Dependencies:**
    ```bash
    sudo apt update
    sudo apt install -y python3-picamera2 python3-rpi.gpio python3-prctl python3-libcamera python3-opencv
    # python3-opencv might be needed if not using pip version for some system-level integrations
    ```

3.  **Navigate to Server Directory:**
    ```bash
    cd server
    ```

4.  **Create and Activate Python Virtual Environment:**
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    ```

5.  **Install Python Dependencies:**
    ```bash
    pip install -r requirements.txt
    ```
    The `requirements.txt` should include: `RPi.GPIO`, `opencv-python`, `numpy`, `python-dotenv`, `websockets`, `Pillow`, `v4l2-python3`, `piexif`, and `picamera2` (if not fully covered by system install for venv).

6.  **Symbolic Links (If Necessary for Picamera2/RPi.GPIO in venv):**
    If `picamera2` or `RPi.GPIO` installed via `apt` are not accessible within the virtual environment, you might need to create symbolic links. *This step can sometimes be complex and version-dependent. Prefer pip installs if they work reliably.*
    Example for Python 3.9 (adjust version path as needed):
    ```bash
    # Ensure site-packages directory exists in your venv
    # mkdir -p venv/lib/python3.9/site-packages

    # Example links (verify paths on your system):
    # ln -s /usr/lib/python3/dist-packages/picamera2 venv/lib/python3.9/site-packages/
    # ln -s /usr/lib/python3/dist-packages/RPi venv/lib/python3.9/site-packages/
    # ln -s /usr/lib/python3/dist-packages/libcamera venv/lib/python3.9/site-packages/
    # ... and other related files like _prctl.cpython-*-linux-gnu.so
    ```
    **Note:** It's generally preferable to have `picamera2` and other key packages installed via `pip` directly into the virtual environment if compatible versions are available, to avoid symlinking complexities. The `server/requirements.txt` should aim for this.

7.  **Configure Hardware Connections:**
    *   Connect motors to the motor driver and the driver to the Raspberry Pi GPIO pins as defined in your `.env` file or `server/src/utils/config.py`.
    *   Connect servo motors to the specified GPIO pins.
    *   Connect the Raspberry Pi Camera Module.
    *   Ensure the overhead camera is operational and accessible on the network if used.

8.  **Verify GPIO Access:**
    Ensure your user is part of the `gpio` group:
    ```bash
    groups
    ```
    If not, add the user (e.g., `pi`):
    ```bash
    sudo usermod -a -G gpio $(whoami) # Or replace $(whoami) with your username
    ```
    Log out and log back in for the group change to take effect.

9.  **Run the Server:**
    From the `server` directory (with the virtual environment activated):
    ```bash
    python src/main.py
    ```
    Observe the logs for any errors during initialization or operation.

### 4.4. Client Setup (macOS)

1.  **Navigate to Client Directory:**
    From the project root:
    ```bash
    cd client
    ```

2.  **Create and Activate Python Virtual Environment:**
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    ```

3.  **Install Python Dependencies:**
    ```bash
    pip install -r requirements.txt
    ```
    The `client/requirements.txt` should include: `PyQt6`, `opencv-python`, `numpy`, `websockets`, `python-dotenv`, `colorama`.

4.  **Run the Client:**
    *   Ensure the `SERVER_HOST` variable in your root `.env` file correctly points to the IP address of your Raspberry Pi on the local network. The client application (`client/src/config.py`) reads this.
    *   From the `client` directory (with virtual environment activated):
    ```bash
    python src/main.py
    ```

## 5. Server-Side Troubleshooting (from `server/README.md`)

### 5.1. Camera Issues

1.  **Verify Camera is Enabled and Detected:**
    ```bash
    vcgencmd get_camera
    ```
    Expected output: `supported=1 detected=1`. If `detected=0`, check the physical connection of the Pi camera. If `supported=0` or `supported=1 detected=0`, ensure the camera interface is enabled in `raspi-config`.

2.  **Test Camera with `libcamera-hello`:**
    ```bash
    libcamera-hello -t 5000 # Displays a 5-second preview
    ```
    If this fails, there's a fundamental issue with the camera setup or `libcamera` stack.

3.  **Module Not Found Issues in Virtual Environment (for `picamera2`, `RPi.GPIO` etc.):**
    *   Confirm system packages are installed:
        ```bash
        dpkg -l | grep python3-picamera2
        dpkg -l | grep python3-libcamera
        # ... and others mentioned in server/README.md
        ```
    *   If relying on symlinks, ensure they are correct and point to valid installations. Recreate them if broken:
        ```bash
        # Example for picamera2, adjust paths for your Python version in venv
        # ln -sf /usr/lib/python3/dist-packages/picamera2 venv/lib/python3.9/site-packages/
        ```
    *   Verify Python packages within the venv:
        ```bash
        pip list | grep -E "opencv-python|numpy|RPi.GPIO|picamera2"
        ```
        If `picamera2` or `RPi.GPIO` are missing from `pip list` and system symlinks are problematic, try installing them directly via `pip` within the activated virtual environment (this is often the cleaner approach).

### 5.2. GPIO Issues

1.  **Verify User is in `gpio` Group:**
    ```bash
    groups
    ```
    If 'gpio' is not listed, add the current user:
    ```bash
    sudo usermod -a -G gpio $USER
    ```
    Then, **log out and log back in** for the change to take effect. Without this, scripts may fail to access GPIO pins.

## 6. Development Workflow

1.  **Iterative Development:**
    *   Make changes to client (`client/src`) and/or server (`server/src`) code in separate feature branches.
    *   Test client UI and basic functionality. For some server logic (non-hardware dependent), you might adapt parts to run on macOS for quicker iteration.
    *   Commit and push changes frequently.
2.  **Hardware Testing (Server on Pi):**
    *   Pull the latest changes to your Raspberry Pi.
    *   Run the server on the Pi and test with actual hardware.
    *   Focus on `controllers`, `localization`, and hardware interactions. Use extensive logging.
3.  **Integration Testing (Client & Server):**
    *   Run the server on the Pi and the client on macOS.
    *   Thoroughly test all functionalities: manual driving, servo controls, video feed switching, autonomous navigation initiation, target setting, PID tuning updates, emergency stop.
    *   Monitor network traffic and logs on both client and server.
4.  **Code Refinement:** Review code for clarity, efficiency, and robustness. Add/update comments and docstrings.
5.  **Merge & Repeat:** Once a feature is stable and reviewed, merge into the `main` branch.

## 7. Contributing

1.  Ensure your local `main` branch is up-to-date with the remote repository.
2.  Create a descriptive feature branch from `main` (e.g., `feature/enhance-aruco-tracking`).
3.  Make changes, adhering to project coding style (if established).
4.  Commit changes with clear, conventional commit messages (e.g., `feat: Add secondary servo control logic`, `fix: Resolve video feed lag under high load`).
5.  Push your feature branch to the remote repository.
6.  Open a Pull Request (PR) against the `main` branch. Provide a detailed description of changes, rationale, and testing performed.

## 8. License

This project is typically licensed under the MIT License. Check for a `LICENSE` file in the project root for specifics. 