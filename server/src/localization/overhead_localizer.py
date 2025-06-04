import cv2
import numpy as np
import math
import logging
from typing import Optional, Tuple, NamedTuple

# Assuming config is in server.src.utils.config
# This relative import might need adjustment depending on how this module is imported/used
from ..utils.config import (
    ARUCO_DICTIONARY, ROBOT_OVERHEAD_ARUCO_ID,
    MARKER_TO_ROTATION_CENTER_X_OFFSET_PIXELS as X_OFFSET,
    MARKER_TO_ROTATION_CENTER_Y_OFFSET_PIXELS as Y_OFFSET
)

logger = logging.getLogger(__name__)

class RobotPoseEstimate(NamedTuple):
    """Represents the estimated 2D pose of the robot in pixel coordinates.
    Includes both the robot's rotation center and the detected ArUco marker's center.
    """
    r_x_px: float  # X-coordinate of the robot's rotation center (pixels).
    r_y_px: float  # Y-coordinate of the robot's rotation center (pixels).
    theta_rad: float # Orientation of the robot (radians), 0 is along image +X, positive CCW.
    marker_x_px: float # X-coordinate of the ArUco marker's center (pixels).
    marker_y_px: float # Y-coordinate of the ArUco marker's center (pixels).

class OverheadLocalizer:
    """Detects an ArUco marker from an overhead camera view and calculates the robot's
    2D pose (position and orientation) in the image's pixel coordinate system.

    It identifies a specific ArUco marker ID defined as the robot's primary marker.
    From this marker, it calculates:
    1.  The marker's center point (m_x, m_y).
    2.  The marker's orientation (and by extension, the robot's initial orientation).
    3.  The robot's actual rotation center (r_x, r_y) by applying pre-configured
        offsets from the marker's center. These offsets are defined relative to the
        marker's own coordinate system (forward and left).
    """
    def __init__(self):
        """Initializes the OverheadLocalizer.
        Loads ArUco dictionary, parameters, and robot-specific marker ID and offsets from config.
        """
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARY)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.robot_marker_id = ROBOT_OVERHEAD_ARUCO_ID
        
        # Offsets from the ArUco marker's center to the robot's actual rotation center.
        # These are defined in the marker/robot's local coordinate system:
        # - x_offset: Distance along the robot's forward direction (derived from marker orientation).
        # - y_offset: Distance to the robot's left (perpendicular to forward direction).
        self.x_offset_marker_frame = X_OFFSET
        self.y_offset_marker_frame = Y_OFFSET
        
        logger.info(f"OverheadLocalizer initialized. Target ArUco ID: {self.robot_marker_id}, Dictionary: {ARUCO_DICTIONARY}.")
        logger.info(f"Robot center offset from marker (marker's frame): X (fwd)={self.x_offset_marker_frame}px, Y (left)={self.y_offset_marker_frame}px.")

    def _normalize_angle_rad(self, angle_rad: float) -> float:
        """Normalizes an angle in radians to the range [-pi, pi]."""
        while angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        while angle_rad < -math.pi:
            angle_rad += 2 * math.pi
        return angle_rad

    def get_robot_pose_from_frame(self, frame: np.ndarray) -> Optional[RobotPoseEstimate]:
        """Detects the robot's ArUco marker in the frame and returns its 2D pose.

        Args:
            frame: The overhead camera image (NumPy array, expected in BGR format).

        Returns:
            A `RobotPoseEstimate` NamedTuple (r_x_px, r_y_px, theta_rad, marker_x_px, marker_y_px)
            if the robot marker is found, otherwise None.
            - r_x_px, r_y_px: Coordinates of the robot's rotation center (pixels).
            - theta_rad: Orientation of the robot (radians). 0 is along image +X axis,
                         positive angles are counter-clockwise (CCW).
            - marker_x_px, marker_y_px: Coordinates of the ArUco marker's geometric center (pixels).
        """
        if frame is None:
            logger.warning("OverheadLocalizer.get_robot_pose_from_frame received a None frame.")
            return None

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids_list, _ = cv2.aruco.detectMarkers(gray_frame, self.aruco_dict, parameters=self.aruco_params)

        if ids_list is not None:
            for i, marker_id_array in enumerate(ids_list):
                if marker_id_array[0] == self.robot_marker_id:
                    marker_corners = corners_list[i][0] # Get the four corners (type: np.ndarray of shape (4,2))

                    # 1. Calculate the geometric center of the marker (m_x, m_y)
                    marker_x_px = np.mean(marker_corners[:, 0])
                    marker_y_px = np.mean(marker_corners[:, 1])

                    # 2. Calculate marker orientation (theta_rad)
                    #    Assumes standard ArUco corner ordering: 0:top-left, 1:top-right, 2:bottom-right, 3:bottom-left.
                    #    The vector from corner 0 to corner 1 defines the marker's "top" edge.
                    if len(marker_corners) < 2:
                        logger.warning(f"Marker ID {self.robot_marker_id} detected but with < 2 corners. Cannot calculate orientation.")
                        return None
                        
                    p0 = marker_corners[0] # Top-left corner
                    p1 = marker_corners[1] # Top-right corner
                    
                    # Angle of the vector from p0 to p1 (marker's top edge relative to image X-axis).
                    marker_top_edge_angle_rad = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
                    
                    # The robot's "front" is defined as being 90 degrees clockwise from this marker_top_edge_angle_rad.
                    # (i.e., if marker top edge points right (0 rad), robot front points down ( -pi/2 rad) initially).
                    # To get the robot's forward direction (theta_rad for the robot): 
                    # angle of marker top edge + (-pi/2) for CW rotation.
                    robot_theta_rad = marker_top_edge_angle_rad - (math.pi / 2.0)
                    robot_theta_rad = self._normalize_angle_rad(robot_theta_rad) # Normalize to [-pi, pi]

                    # 3. Calculate true robot rotation center (r_x_px, r_y_px)
                    #    Apply offsets from marker center, rotated by the robot's orientation.
                    #    self.x_offset_marker_frame is along robot's forward (defined by robot_theta_rad).
                    #    self.y_offset_marker_frame is to the robot's left.
                    # Standard 2D rotation: x' = x*cos(theta) - y*sin(theta), y' = x*sin(theta) + y*cos(theta)
                    r_x_px = marker_x_px + (
                        self.x_offset_marker_frame * math.cos(robot_theta_rad) - 
                        self.y_offset_marker_frame * math.sin(robot_theta_rad)
                    )
                    r_y_px = marker_y_px + (
                        self.x_offset_marker_frame * math.sin(robot_theta_rad) + 
                        self.y_offset_marker_frame * math.cos(robot_theta_rad)
                    )
                    
                    # logger.debug(f"Robot ID {self.robot_marker_id} found: MarkerCenter=({marker_x_px:.0f}, {marker_y_px:.0f}), RobotCenter=({r_x_px:.0f}, {r_y_px:.0f}), RobotTheta={math.degrees(robot_theta_rad):.1f}°")
                    return RobotPoseEstimate(
                        r_x_px=float(r_x_px),
                        r_y_px=float(r_y_px),
                        theta_rad=float(robot_theta_rad),
                        marker_x_px=float(marker_x_px),
                        marker_y_px=float(marker_y_px)
                    )
            
            return None # Robot marker ID not found among detected markers.
        else:
            return None # No markers detected at all.

    def draw_robot_pose_on_frame(self, frame: np.ndarray, pose_estimate: Optional[RobotPoseEstimate]) -> np.ndarray:
        """Draws the detected robot pose (rotation center, orientation line, and marker center) on the frame.

        Args:
            frame: The image frame (NumPy BGR array) to draw on.
            pose_estimate: A `RobotPoseEstimate` object containing the pose data, or None.

        Returns:
            The frame with the pose visualization drawn on it.
        """
        if pose_estimate is None:
            return frame

        # Draw the original ArUco marker center (small blue dot)
        cv2.circle(frame, (int(pose_estimate.marker_x_px), int(pose_estimate.marker_y_px)), 
                   radius=3, color=(255, 100, 100), thickness=-1) # Light Blue dot for marker center

        # Draw the calculated robot rotation center (larger magenta circle)
        cv2.circle(frame, (int(pose_estimate.r_x_px), int(pose_estimate.r_y_px)), 
                   radius=5, color=(255, 0, 255), thickness=-1) # Magenta circle for rotation center

        # Draw orientation line from the robot rotation center, indicating robot's forward direction
        line_length = 50 # pixels
        orient_end_x = int(pose_estimate.r_x_px + line_length * math.cos(pose_estimate.theta_rad))
        orient_end_y = int(pose_estimate.r_y_px + line_length * math.sin(pose_estimate.theta_rad))
        cv2.line(frame, (int(pose_estimate.r_x_px), int(pose_estimate.r_y_px)), 
                 (orient_end_x, orient_end_y), color=(0, 255, 0), thickness=2) # Green line for orientation

        return frame

    def draw_target_on_frame(self, frame: np.ndarray, target_pose_pixels: Optional[Tuple[float,float,float]], 
                             label: str, color: Tuple[int,int,int] = (0,255,255)) -> np.ndarray:
        """Draws a target waypoint (position and orientation) on the frame.

        Args:
            frame: The image frame (NumPy BGR array) to draw on.
            target_pose_pixels: A tuple (x_px, y_px, theta_rad) for the target, or None.
            label: Text label for the target (e.g., "PICKUP", "DROPOFF").
            color: BGR tuple for the drawing color.

        Returns:
            The frame with the target visualization drawn on it.
        """
        if target_pose_pixels is None:
            return frame
        
        x_px, y_px, theta_rad = target_pose_pixels

        # Draw target position (e.g., a crosshair or circle)
        cv2.drawMarker(frame, (int(x_px), int(y_px)), color, markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        # cv2.circle(frame, (int(x_px), int(y_px)), radius=10, color=color, thickness=2)

        # Draw orientation line from the target position
        line_length = 40 # pixels
        orient_end_x = int(x_px + line_length * math.cos(theta_rad))
        orient_end_y = int(y_px + line_length * math.sin(theta_rad))
        cv2.line(frame, (int(x_px), int(y_px)), (orient_end_x, orient_end_y), color, 2)

        # Put label
        cv2.putText(frame, label, (int(x_px) + 15, int(y_px) - 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return frame

if __name__ == '__main__':
    # This is a simple test block that you can use if you have an image file
    # and want to test the OverheadLocalizer directly.
    # You'll need to adjust file paths and potentially the robot_marker_id.
    
    # Configure logging for standalone testing
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    logger.info("Running OverheadLocalizer standalone test...")
    
    # Create a dummy config if it's not found (e.g. when running standalone)
    try:
        from ..utils.config import ARUCO_DICTIONARY, ROBOT_OVERHEAD_ARUCO_ID
    except ImportError:
        logger.warning("Could not import from ..utils.config. Using dummy values for standalone test.")
        ARUCO_DICTIONARY = cv2.aruco.DICT_6X6_50 # Ensure this matches your target dict
        ROBOT_OVERHEAD_ARUCO_ID = 50 # Ensure this matches your target marker ID

    localizer = OverheadLocalizer()

    # --- Option 1: Test with a static image file ---
    # test_image_path = 'path_to_your_test_image_with_aruco_marker.jpg'
    # try:
    #     test_frame = cv2.imread(test_image_path)
    #     if test_frame is None:
    #         logger.error(f"Failed to load test image from: {test_image_path}")
    #     else:
    #         logger.info(f"Successfully loaded test image: {test_image_path}")
    #         pose = localizer.detect_robot_pose(test_frame)
    #         if pose:
    #             logger.info(f"Detected pose: x={pose[0]:.2f}, y={pose[1]:.2f}, theta={math.degrees(pose[2]):.2f} deg")
    #             frame_with_pose = localizer.draw_pose_on_frame(test_frame.copy(), pose)
    #             cv2.imshow("Overhead Pose Test", frame_with_pose)
    #             logger.info("Displaying image with pose. Press any key to close.")
    #             cv2.waitKey(0)
    #             cv2.destroyAllWindows()
    #         else:
    #             logger.info("Robot marker not found in the test image.")
    #             cv2.imshow("Overhead Pose Test - Not Found", test_frame)
    #             cv2.waitKey(0)
    #             cv2.destroyAllWindows()
    # except Exception as e:
    #     logger.error(f"Error during static image test: {e}", exc_info=True)

    # --- Option 2: Test with a blank image and a manually drawn marker (for very basic check) ---
    # This won't be a real ArUco marker but can test drawing logic.
    # blank_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    # test_pose = (320, 240, math.radians(45)) # Center, 45 degrees
    # frame_with_drawn_pose = localizer.draw_pose_on_frame(blank_frame, test_pose)
    # cv2.imshow("Drawn Pose Test", frame_with_drawn_pose)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    logger.info("OverheadLocalizer standalone test finished. Uncomment image loading section to test with an image.") 