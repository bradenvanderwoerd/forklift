# ArUco Marker Navigation Plan for Forklift Robot

This document outlines the plan for implementing autonomous navigation and task execution using ArUco markers.

## Goal
The robot must detect ArUco markers of a certain type (on a "box"), pick up the box using the forklift servo, navigate to another specific marker (a "shipping container"), and drop off the box.

## Phases

### Phase 1: ArUco Detection & Basic Setup

**Objective:** Enable the server to detect ArUco markers in the video stream and provide visual feedback.

1.  **Dependencies (Server):**
    *   Add `opencv-contrib-python` to `server/requirements.txt`.
    *   Install the updated requirements in the server's virtual environment.
2.  **Camera Calibration (Prerequisite):**
    *   **CRITICAL:** Obtain camera intrinsic matrix and distortion coefficients for the Raspberry Pi camera being used. Accurate pose estimation is impossible without this.
    *   Store these calibration values (e.g., in `server/src/config.py` or a separate calibration file).
3.  **Basic Detection Logic (Server - `video_stream.py`):**
    *   Import `cv2.aruco`.
    *   Define a target ArUco dictionary (e.g., `cv2.aruco.DICT_4X4_50`).
    *   Create ArUco detector parameters.
    *   In the video processing loop:
        *   Convert the captured frame to grayscale.
        *   Call `cv2.aruco.detectMarkers()` to find marker corners and IDs.
4.  **Visual Feedback (Server - `video_stream.py`):**
    *   If markers are detected, use `cv2.aruco.drawDetectedMarkers()` to draw outlines and IDs onto the *color* video frame before it's encoded and sent to the client.

---

### Phase 2: Pose Estimation & Marker Identification

**Objective:** Determine the 3D position and orientation of detected markers and identify specific target markers.

1.  **Pose Estimation (Server - `video_stream.py` or dedicated module):**
    *   Define the physical size of the ArUco markers being used (in meters). Store in config.
    *   Use `cv2.aruco.estimatePoseSingleMarkers()` with detected corners, marker size, camera matrix, and distortion coefficients. This yields rotation vectors (`rvecs`) and translation vectors (`tvecs`).
    *   `(Optional)` Use `cv2.drawFrameAxes()` (OpenCV >= 4.7) or `cv2.aruco.drawAxis()` (older OpenCV) to draw the marker's coordinate system axes onto the video frame for visualization.
2.  **Marker Identification (Server - `config.py` & logic):**
    *   Define specific ArUco IDs for "box" markers and the "shipping container" marker.
    *   Add logic to check detected marker IDs against these target IDs.

---

### Phase 3: Navigation & Control Logic

**Objective:** Implement closed-loop control to navigate the robot towards a target marker.

1.  **Navigation Controller (Server - new controller class):**
    *   Create `NavigationController` (e.g., `server/src/controllers/navigation.py`).
    *   Input: Target marker's pose (`tvec`, `rvec`).
    *   Output: Motor commands (delegated to `MotorController`).
    *   Implement PID control loops:
        *   **Turning PID:** Corrects heading based on the marker's X-coordinate in `tvec`. Target: X ≈ 0. Output controls differential speed (e.g., `motor_controller.turn_left/right` or direct wheel speeds).
        *   **Distance PID:** Controls forward/backward movement based on the marker's Z-coordinate (distance) in `tvec`. Target: A predefined approach distance (e.g., 10cm). Output controls overall speed (`motor_controller.drive_forward/backward`).
2.  **Servo Positions (Server - `config.py` & `ServoController`):**
    *   Define `FORK_DOWN_POSITION` and `FORK_UP_POSITION` angles in `config.py`.
    *   Add methods to `ServoController` like `go_to_down_position()` and `go_to_up_position()`.

---

### Phase 4: State Machine & Communication

**Objective:** Orchestrate the entire pickup-and-delivery task and communicate progress to the client.

1.  **State Machine (Server - `main.py` / `ForkliftServer`):**
    *   Define states: `IDLE`, `SEEKING_BOX`, `APPROACHING_BOX`, `LIFTING`, `SEEKING_CONTAINER`, `APPROACHING_CONTAINER`, `DROPPING`, `DONE`, `ERROR`.
    *   Implement state transition logic based on marker detection, navigation controller status (e.g., target reached), and servo actions.
    *   Manage the overall task flow.
2.  **Communication Protocol (Shared, Client, Server):**
    *   **Commands (Client -> Server):**
        *   Add `START_DELIVERY` command type in `shared/protocol/commands.py`.
        *   Add button in client UI (`main_window.py`) to send this command.
        *   Update `CommandServer` (`tcp_server.py`) and `ForkliftServer` (`main.py`) to handle this command (e.g., initiate the state machine).
    *   **Status/Feedback (Server -> Client):**
        *   Define `STATUS_UPDATE` message type (e.g., in a new `shared/protocol/feedback.py`). Include fields like `state`, `message`, `target_marker_id`.
        *   Modify `CommandServer` to allow sending messages back to the client(s).
        *   Have the state machine in `ForkliftServer` send status updates via the `CommandServer`.
        *   Modify `RobotClient` (`client.py`) to receive messages from the command websocket.
        *   Add a status display area in the client UI (`main_window.py`) to show feedback from the server.

--- 