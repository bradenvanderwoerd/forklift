import cv2
import numpy as np
import math
import logging
from typing import Optional, Tuple

# Assuming config is in server.src.utils.config
# This relative import might need adjustment depending on how this module is imported/used
from ..utils.config import ARUCO_DICTIONARY, ROBOT_OVERHEAD_ARUCO_ID

logger = logging.getLogger(__name__)

class OverheadLocalizer:
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARY)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.robot_marker_id = ROBOT_OVERHEAD_ARUCO_ID
        logger.info(f"OverheadLocalizer initialized to track ArUco ID: {self.robot_marker_id} using dictionary: {ARUCO_DICTIONARY}")

    def detect_robot_pose(self, frame: np.ndarray) -> Optional[Tuple[float, float, float]]:
        """
        Detects the robot's ArUco marker in the frame and returns its 2D pose (x_pixel, y_pixel, theta_pixel).

        Args:
            frame: The overhead camera image (NumPy array).

        Returns:
            A tuple (x_pixel, y_pixel, theta_pixel) if the robot marker is found, otherwise None.
            x_pixel: The x-coordinate of the marker's center in pixels.
            y_pixel: The y-coordinate of the marker's center in pixels.
            theta_pixel: The orientation of the marker in radians, relative to the image's positive x-axis.
                         0 radians points to the right. Positive angles are counter-clockwise.
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

                    # Calculate the center of the marker (x_pixel, y_pixel)
                    center_x = np.mean(marker_corners[:, 0])
                    center_y = np.mean(marker_corners[:, 1])

                    # Calculate orientation (theta_pixel)
                    # We'll use the vector from the top-left corner (0) to the top-right corner (1)
                    # to determine the orientation.
                    # Corners are usually ordered: 0: top-left, 1: top-right, 2: bottom-right, 3: bottom-left
                    
                    # Check if we have enough corners (should be 4)
                    if len(marker_corners) < 2:
                        logger.warning(f"Marker ID {self.robot_marker_id} detected but with less than 2 corners. Cannot calculate orientation.")
                        return None
                        
                    p1 = marker_corners[0] # Top-left
                    p2 = marker_corners[1] # Top-right
                    
                    # Calculate the angle of the vector p1->p2 relative to the positive x-axis
                    # The y-axis in image coordinates is typically inverted (positive downwards)
                    # So, dy = p2[1] - p1[1] means:
                    # - if p2 is lower than p1, dy is positive.
                    # - if p2 is higher than p1, dy is negative.
                    # math.atan2(dy, dx)
                    # dx = p2[0] - p1[0]
                    # dy = p2[1] - p1[1]
                    # Angle from positive x-axis, counter-clockwise.
                    # A marker edge from (0,0) to (10,0) -> angle 0
                    # A marker edge from (0,0) to (0,10) -> angle pi/2 (if y points down)
                    # atan2 handles quadrants correctly.
                    angle_rad = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
                    
                    # logger.debug(f"Robot marker ID {self.robot_marker_id} found at ({center_x:.2f}, {center_y:.2f}) pixels, angle: {math.degrees(angle_rad):.2f} degrees.")
                    return (float(center_x), float(center_y), float(angle_rad))
            
            # Robot marker ID not found among detected markers
            # logger.debug(f"Detected markers {ids.flatten()} but not the robot ID {self.robot_marker_id}")
            return None
        else:
            # No markers detected at all
            # logger.debug("No ArUco markers detected in the overhead frame.")
            return None

    def draw_pose_on_frame(self, frame: np.ndarray, pose: Tuple[float, float, float]) -> np.ndarray:
        """
        Draws the detected pose (center and orientation line) on the frame.

        Args:
            frame: The image frame to draw on.
            pose: The (x_pixel, y_pixel, theta_pixel) pose.

        Returns:
            The frame with the pose drawn on it.
        """
        if pose is None:
            return frame

        x_center, y_center, angle_rad = pose
        
        # Draw the center
        cv2.circle(frame, (int(x_center), int(y_center)), 5, (0, 0, 255), -1) # Red circle for center

        # Draw orientation line
        line_length = 50 # pixels
        end_x = int(x_center + line_length * math.cos(angle_rad))
        end_y = int(y_center + line_length * math.sin(angle_rad))
        cv2.line(frame, (int(x_center), int(y_center)), (end_x, end_y), (0, 255, 0), 2) # Green line for orientation

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