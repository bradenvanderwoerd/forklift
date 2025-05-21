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

### Phase 5: Overhead Camera Navigation (Pixel Space Localization)

**Core Idea:** Use a fixed, uncalibrated (in terms of world units like meters) overhead camera to determine the robot's (X, Y) pixel coordinates and orientation (Theta) within the camera's image. Navigation will be performed in this "image-space" or a scaled version of it. Target and obstacle locations (bins, truck, pole) will also be defined in this image-space.

**Sub-Phase 5.1: Overhead Robot Tracking (Pixel Space)**

1.  **Hardware Setup:**
    *   Mount a fixed overhead camera viewing the entire arena.
    *   Mount an ArUco marker flat on top of the forklift, clearly visible to the overhead camera.
    *   Place an ArUco marker on the central pole.
    *   (Recommended) Place ArUco markers on the bins and truck destination area, visible to the overhead camera.

2.  **Overhead Camera Video Stream:**
    *   Adapt or replicate the existing "arena server architecture" (from `teleoperation` branch or similar) to stream the overhead camera's video feed to the `ForkliftServer` (running on the Raspberry Pi), likely via TCP.

3.  **Robot Pose Estimation (within `ForkliftServer`):**
    *   **Input:** The `ForkliftServer` receives the video stream from the overhead camera.
    *   **Processing (New module/class, e.g., `OverheadLocalizer` within `ForkliftServer` or as a separate utility):**
        *   For each frame from the overhead camera:
            *   Detect the ArUco marker on top of the robot using its specific ID.
            *   Determine the robot's (X_pixel, Y_pixel) position using the center of the detected marker in the image.
            *   Determine the robot's orientation (Theta_pixel_frame) from the orientation of the marker's corners within the image (angle relative to image axes).
        *   Store this estimated (X_pixel, Y_pixel, Theta_pixel_frame) as `self.robot_overhead_pose`.

**Sub-Phase 5.2: Defining Targets and Obstacles (Pixel Space)**

1.  **Mapping Static Elements (Pole, Bins, Truck Destination):**
    *   **Method (One-time setup or dynamic if markers are used):**
        *   Create a utility script that displays the overhead camera feed.
        *   Option A (Manual): Manually click on or identify the pixel coordinates of key static locations (center of pole, its radius; centers of bin areas; center of truck destination).
        *   Option B (Automated - Preferred for static ArUco markers): If ArUco markers are placed on these static elements and visible to the overhead camera, the script automatically detects them and records their (X_pixel, Y_pixel) centers and orientations.
    *   Store these image-space coordinates for static elements in `server/src/utils/config.py`.

2.  **Dynamic Box Mapping (Using Onboard Pi Camera and Overhead Localization):**
    *   **Objective:** To find the (X_pixel, Y_pixel) coordinates of movable boxes (which don't have overhead markers) within the overhead camera's image space.
    *   **Process (Likely managed by `ForkliftServer`'s main logic or a dedicated mapping state):**
        a.  **(Optional) Robot Maneuvers:** Robot navigates (using `self.robot_overhead_pose`) to predefined viewing locations where its onboard Pi camera can see into the box bin areas.
        b.  **Onboard Camera Detection:** The `VideoStreamer` (managing the Pi camera) detects ArUco markers on the boxes (e.g., White Box ID 5, Black Box ID 6). For each detected box, it calculates its pose (`tvec_box_onboard`, `rvec_box_onboard`) *relative to the Pi camera*.
        c.  **Obtain Current Robot Pose:** At the moment of onboard detection, retrieve the robot's current global pose from the overhead system: `(X_robot_pixel, Y_robot_pixel, Theta_robot_pixel) = self.robot_overhead_pose`.
        d.  **Transformation to Overhead Pixel Space:**
            *   **Required Data:**
                *   Box pose relative to Pi camera (`tvec_box_onboard`, `rvec_box_onboard`).
                *   Fixed transform from the robot's overhead marker/center to the Pi camera's optical center (dX_rc, dY_rc, dZ_rc, dRoll_rc, dPitch_rc, dYaw_rc - measured once).
                *   Robot's current pose in overhead pixel space (`X_robot_pixel`, `Y_robot_pixel`, `Theta_robot_pixel`).
                *   Pi camera's intrinsic parameters (from `config.py`).
            *   **Transformation Steps (Simplified Conceptual Outline):**
                1.  Convert `tvec_box_onboard` (box relative to Pi cam) to a real-world offset (e.g., meters_x, meters_y from Pi cam).
                2.  Transform this offset into the robot's base frame using the known Pi camera to robot center transform. This gives the box's (meters_x, meters_y) relative to the robot's center.
                3.  Using `(X_robot_pixel, Y_robot_pixel, Theta_robot_pixel)` and an approximate pixel-per-meter scaling factor (derived from observing the overhead view or a simple calibration), project the box's real-world offset from the robot onto the overhead image to get an estimated `(X_box_pixel, Y_box_pixel)`.
                *   *Note: A more accurate projection would involve using the overhead camera's (even if roughly estimated) intrinsics and extrinsics, or a homography if the plane is consistent. The simplified approach assumes the robot and boxes are roughly on the same plane visible to the overhead camera.*
        e.  **Store Mapped Box Locations:** The `ForkliftServer` stores these dynamically determined `(X_box_pixel, Y_box_pixel)` coordinates for each box ID (e.g., in `self.mapped_box_pixel_locations = {BOX_ID_WHITE: (x,y), BOX_ID_BLACK: (x,y)}`). This map is updated whenever a new scan/mapping is performed.

3.  **Dynamic Target Confirmation (Forklift's Onboard Camera - As Before):**
    *   The forklift's front-facing camera remains crucial for:
        *   Confirming the identity of a box (e.g., White Box ID 5 vs. Black Box ID 6) when at a bin location.
        *   Fine-grained alignment for pickup (using its `tvec`/`rvec`).

**Sub-Phase 5.3: Navigation and Control Logic (Pixel Space)**

1.  **Navigation Controller (`NavigationController` Adaptation):**
    *   **Target Setting:** The `set_target` method will now accept target coordinates in *pixel space* (e.g., (X_target_pixel, Y_target_pixel, Theta_target_pixel)).
    *   **Path Planning (Iterative Development):**
        *   **Initial:** Direct line-of-sight navigation in pixel space towards the target (X_pixel, Y_pixel).
        *   **Obstacle Avoidance (Pole):** Implement logic to check if the direct path to the target intersects the known pixel-space region of the pole. If so, generate intermediate waypoints to navigate around it.
        *   **Advanced (Future):** Consider more sophisticated path planning algorithms if needed (e.g., A* on a grid representation of the pixel space).
    *   **Motion Control:**
        *   PID controllers will be adapted to minimize errors between the robot's current `robot_overhead_pose` and the target pixel-space pose.
        *   This will likely involve separate PIDs for:
            *   Error in X_pixel.
            *   Error in Y_pixel.
            *   Error in Theta_pixel_frame (orientation).
        *   Alternatively, a "go-to-goal" PID for (X,Y) and a separate "orient" PID for Theta.
        *   PID gains (`Kp`, `Ki`, `Kd`) and thresholds in `config.py` will need to be re-tuned for pixel-space errors and desired movement characteristics.

**Considerations for Pixel-Space Navigation:**

*   **Consistency:** Performance relies on the overhead camera remaining perfectly fixed. Any shift will invalidate mapped pixel coordinates.
*   **Distortion:** Lens distortion in the overhead camera can cause non-linear relationships between pixel movement and real-world movement, potentially affecting PID tuning across the arena. If problematic, a 2D homography transformation could be applied to the overhead image to create a "flattened" top-down view before pixel coordinates are extracted.
*   **Coordinate Systems:** Clearly manage transformations if any are needed (e.g., between robot's local frame from front camera and the overhead pixel-space frame). 