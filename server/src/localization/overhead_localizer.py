import cv2
import numpy as np
import math
import logging
from typing import Optional, Tuple

# Assuming config is in server.src.utils.config
# This relative import might need adjustment depending on how this module is imported/used
from ..utils.config import (
    ARUCO_DICTIONARY, ROBOT_OVERHEAD_ARUCO_ID,
    MARKER_TO_ROTATION_CENTER_X_OFFSET_PIXELS as X_OFFSET,
    MARKER_TO_ROTATION_CENTER_Y_OFFSET_PIXELS as Y_OFFSET
)

logger = logging.getLogger(__name__)

class OverheadLocalizer:
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARY)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.robot_marker_id = ROBOT_OVERHEAD_ARUCO_ID
        self.x_offset = X_OFFSET
        self.y_offset = Y_OFFSET
        logger.info(f"OverheadLocalizer initialized to track ArUco ID: {self.robot_marker_id} using dictionary: {ARUCO_DICTIONARY}")
        logger.info(f"Robot center offset from marker: X={self.x_offset}px, Y={self.y_offset}px (marker perspective)")

    def _normalize_angle(self, angle_rad: float) -> float:
        """Normalize an angle to the range [-pi, pi]."""
        while angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        while angle_rad < -math.pi:
            angle_rad += 2 * math.pi
        return angle_rad

    def detect_robot_pose(self, frame: np.ndarray) -> Optional[Tuple[float, float, float, float, float]]:
        """
        Detects the robot's ArUco marker in the frame and returns its 2D pose (r_x, r_y, m_theta_rad, m_x, m_y).

        Args:
            frame: The overhead camera image (NumPy array).

        Returns:
            A tuple (r_x, r_y, m_theta_rad, m_x, m_y) if the robot marker is found, otherwise None.
            r_x: The x-coordinate of the robot's rotation center in pixels.
            r_y: The y-coordinate of the robot's rotation center in pixels.
            m_theta_rad: The orientation of the robot in radians, relative to the image's positive x-axis.
                         0 radians points to the right. Positive angles are counter-clockwise.
            m_x: The x-coordinate of the marker's center in pixels.
            m_y: The y-coordinate of the marker's center in pixels.
        """
        if frame is None:
            logger.warning("OverheadLocalizer.detect_robot_pose received a None frame.")
            return None

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray_frame, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for i, marker_id in enumerate(ids):
                if marker_id[0] == self.robot_marker_id:
                    marker_corners = corners[i][0] # Get the four corners of the detected marker

                    # Calculate the center of the marker (m_x, m_y)
                    m_x = np.mean(marker_corners[:, 0])
                    m_y = np.mean(marker_corners[:, 1])

                    # Calculate orientation (m_theta_rad) based on marker corners
                    if len(marker_corners) < 2:
                        logger.warning(f"Marker ID {self.robot_marker_id} detected but with less than 2 corners. Cannot calculate orientation.")
                        return None
                        
                    p1 = marker_corners[0] # Top-left
                    p2 = marker_corners[1] # Top-right
                    
                    m_theta_rad = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
                    m_theta_rad -= math.pi / 2 # Existing 90-degree clockwise adjustment for marker vs robot front
                    m_theta_rad = self._normalize_angle(m_theta_rad) # Normalize the adjusted angle

                    # Calculate true robot rotation center (r_x, r_y) based on marker center, orientation, and offsets
                    # Rotate the offset vector (self.x_offset, self.y_offset) by m_theta_rad
                    # and add it to the marker center (m_x, m_y).
                    # Note: self.x_offset is along the robot's perceived forward, self.y_offset is to its left.
                    r_x = m_x + (self.x_offset * math.cos(m_theta_rad) - self.y_offset * math.sin(m_theta_rad))
                    r_y = m_y + (self.x_offset * math.sin(m_theta_rad) + self.y_offset * math.cos(m_theta_rad))
                    
                    # The robot's final reported pose is (r_x, r_y, m_theta_rad)
                    # We also pass the original marker center for drawing purposes.
                    # logger.debug(f"Robot marker ID {self.robot_marker_id} found at M=({m_x:.0f}, {m_y:.0f}), R=({r_x:.0f}, {r_y:.0f}), Theta={math.degrees(m_theta_rad):.1f}")
                    return (float(r_x), float(r_y), float(m_theta_rad), float(m_x), float(m_y)) # Return r_x, r_y, theta, m_x, m_y
            
            return None # Robot marker ID not found
        else:
            return None # No markers detected

    def draw_pose_on_frame(self, frame: np.ndarray, pose_data: Tuple[float, float, float, float, float]) -> np.ndarray:
        """
        Draws the detected pose (center and orientation line) on the frame.
        Also draws the original marker center.

        Args:
            frame: The image frame to draw on.
            pose_data: Tuple containing (r_x, r_y, m_theta_rad, m_x, m_y).

        Returns:
            The frame with the pose drawn on it.
        """
        if pose_data is None:
            return frame

        r_x, r_y, m_theta_rad, m_x, m_y = pose_data
        
        # Draw the original marker center (small blue dot)
        cv2.circle(frame, (int(m_x), int(m_y)), 3, (255, 0, 0), -1) # Blue dot for marker center

        # Draw the calculated robot rotation center (small magenta circle)
        cv2.circle(frame, (int(r_x), int(r_y)), 5, (255, 0, 255), -1) # Magenta circle for rotation center

        # Draw orientation line from the robot rotation center (r_x, r_y)
        line_length = 50 # pixels
        end_x = int(r_x + line_length * math.cos(m_theta_rad))
        end_y = int(r_y + line_length * math.sin(m_theta_rad))
        cv2.line(frame, (int(r_x), int(r_y)), (end_x, end_y), (0, 255, 0), 2) # Green line for orientation from rotation center

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