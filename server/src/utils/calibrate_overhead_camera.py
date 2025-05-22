import cv2
import cv2.aruco as aruco
import numpy as np
import time
import os
import logging

# --- Custom Project Imports ---
# Assuming this script is in server/src/utils, and WarehouseCameraClient is in server/src/network
# Adjust the import path if your project structure is different.
try:
    from ..network.overhead_camera_client import WarehouseCameraClient
    from .config import OVERHEAD_CAMERA_HOST, OVERHEAD_CAMERA_PORT
except ImportError:
    print("Error: Could not import WarehouseCameraClient or config.")
    print("Make sure this script is run in an environment where server.src modules are accessible.")
    print("You might need to run it as a module from the project root, e.g., python -m server.src.utils.calibrate_overhead_camera")
    exit()

# --- Logging Setup ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# --- ChArUco Board Parameters (from existing calibrate_camera.py) ---
CHARUCO_SQUARES_X = 7        # Number of squares in X direction
CHARUCO_SQUARES_Y = 9        # Number of squares in Y direction
CHARUCO_SQUARE_LENGTH = 0.0204 # Square size in meters
CHARUCO_MARKER_LENGTH = 0.015  # ArUco marker size in meters
CHARUCO_ARUCO_DICT_NAME = aruco.DICT_7X7_1000 # ArUco dictionary

# --- Calibration Parameters ---
OUTPUT_CALIBRATION_FILE = "../overhead_camera_calibration.npz" # Relative to this script (i.e., server/src/overhead_camera_calibration.npz)
MIN_CAPTURES_FOR_CALIBRATION = 10 # Minimum number of good captures required

def create_charuco_board():
    """Creates the ChArUco board object based on the defined parameters."""
    aruco_dict = aruco.getPredefinedDictionary(CHARUCO_ARUCO_DICT_NAME)
    board = aruco.CharucoBoard(
        (CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y),
        CHARUCO_SQUARE_LENGTH,
        CHARUCO_MARKER_LENGTH,
        aruco_dict
    )
    logger.info(f"ChArUco board created: Squares=({CHARUCO_SQUARES_X},{CHARUCO_SQUARES_Y}), "
                f"Square={CHARUCO_SQUARE_LENGTH}m, Marker={CHARUCO_MARKER_LENGTH}m, Dict={CHARUCO_ARUCO_DICT_NAME}")
    return board

def main_calibration_loop():
    logger.info("Starting overhead camera calibration process...")
    logger.info(f"Connecting to overhead camera at {OVERHEAD_CAMERA_HOST}:{OVERHEAD_CAMERA_PORT}")

    camera_client = WarehouseCameraClient(host=OVERHEAD_CAMERA_HOST, port=OVERHEAD_CAMERA_PORT)
    if not camera_client.connect(): # connect() should ideally return a status or raise exception
        logger.error("Failed to connect to the overhead camera. Exiting.")
        return

    board = create_charuco_board()
    aruco_detector_params = aruco.DetectorParameters()
    aruco_detector = aruco.ArucoDetector(board.getDictionary(), aruco_detector_params)

    all_charuco_corners = []
    all_charuco_ids = []
    image_size = None # Will be determined from the first frame

    logger.info("\n--- Instructions ---")
    logger.info("1. Position the ChArUco board in the camera's view.")
    logger.info("2. Press SPACEBAR to capture a frame and detect the board.")
    logger.info(f"3. Collect at least {MIN_CAPTURES_FOR_CALIBRATION} good views of the board from different angles and positions.")
    logger.info("4. Press 'c' to perform calibration once enough views are collected.")
    logger.info("5. Press 'q' to quit without calibrating.")
    logger.info("--------------------\n")

    window_name = "Overhead Camera Calibration - Live View"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    while True:
        frame = camera_client.get_video_frame() # Assumes this method exists and returns a BGR numpy array
        
        if frame is None:
            logger.warning("No frame received from camera client. Retrying...")
            time.sleep(0.1)
            continue

        if image_size is None:
            image_size = (frame.shape[1], frame.shape[0]) # (width, height)
            logger.info(f"Determined image size: {image_size}")

        display_frame = frame.copy()
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers first
        marker_corners, marker_ids, _ = aruco_detector.detectMarkers(gray_frame)

        if marker_ids is not None and len(marker_ids) > 0:
            aruco.drawDetectedMarkers(display_frame, marker_corners, marker_ids)
            
            # Interpolate ChArUco corners
            response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray_frame, board
            )

            if response > 0: # Some ChArUco corners were found
                aruco.drawDetectedCornersCharuco(display_frame, charuco_corners, charuco_ids, (0, 255, 0))
                
                # Display text about current capture
                cv2.putText(display_frame, f"Press SPACE to capture. Found {response} ChArUco corners.",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display_frame, "No ArUco markers detected. Position the board.",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display_frame, f"Captures: {len(all_charuco_corners)}/{MIN_CAPTURES_FOR_CALIBRATION}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 0), 2)
        cv2.putText(display_frame, "Press 'c' to calibrate, 'q' to quit.",
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 0), 2)
        
        cv2.imshow(window_name, display_frame)
        key = cv2.waitKey(30) & 0xFF

        if key == ord('q'):
            logger.info("Quitting calibration process.")
            break
        elif key == ord(' '): # Spacebar to capture
            if marker_ids is not None and len(marker_ids) > 0 and response > 4: # Require at least a few corners
                logger.info(f"Captured frame with {response} ChArUco corners.")
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
            else:
                logger.warning("Capture failed: Not enough ChArUco corners detected in the current view.")
        elif key == ord('c'):
            if len(all_charuco_corners) >= MIN_CAPTURES_FOR_CALIBRATION:
                logger.info(f"Proceeding to calibration with {len(all_charuco_corners)} views.")
                perform_calibration(board, all_charuco_corners, all_charuco_ids, image_size)
                break # Exit after attempting calibration
            else:
                logger.warning(f"Cannot calibrate yet. Need at least {MIN_CAPTURES_FOR_CALIBRATION} captures, "
                               f"but only have {len(all_charuco_corners)}.")

    camera_client.disconnect()
    cv2.destroyAllWindows()
    logger.info("Calibration loop finished.")

def perform_calibration(board, all_charuco_corners, all_charuco_ids, image_size):
    """Performs camera calibration using the collected ChArUco corners and IDs."""
    logger.info("Starting calibration calculation...")
    try:
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            all_charuco_corners,
            all_charuco_ids,
            board,
            image_size,
            None, # cameraMatrix initially
            None  # distCoeffs initially
        )

        if not ret:
            logger.error("Calibration failed! calibrateCameraCharuco returned False.")
            return

        logger.info("\n--- Calibration Results ---")
        logger.info(f"Overall Reprojection Error (ret value): {ret:.4f} pixels (lower is better)")
        logger.info("Camera Matrix (mtx):")
        logger.info(str(camera_matrix))
        logger.info("\nDistortion Coefficients (dist):")
        logger.info(str(dist_coeffs))

        # Detailed Reprojection Error (more common way to express it)
        total_error = 0
        for i in range(len(all_charuco_ids)):
            # Project 3D points to image plane
            img_points_reprojected, _ = cv2.projectPoints(
                board.getChessboardCorners()[all_charuco_ids[i].flatten()], # Get 3D obj points for these IDs
                rvecs[i],
                tvecs[i],
                camera_matrix,
                dist_coeffs
            )
            error = cv2.norm(all_charuco_corners[i], img_points_reprojected, cv2.NORM_L2) / len(img_points_reprojected)
            total_error += error
        
        mean_reprojection_error = total_error / len(all_charuco_ids)
        logger.info(f"\nMean Reprojection Error (calculated): {mean_reprojection_error:.4f} pixels")

        if mean_reprojection_error > 1.0: # Arbitrary threshold, can be adjusted
            logger.warning("Mean reprojection error is high (> 1.0 pixels). "
                           "Calibration might be suboptimal. Consider more/better views or check board planarity.")

        # Save calibration results
        script_dir = os.path.dirname(__file__)
        output_file_path = os.path.abspath(os.path.join(script_dir, OUTPUT_CALIBRATION_FILE))
        
        np.savez(output_file_path, mtx=camera_matrix, dist=dist_coeffs, reproj_error=mean_reprojection_error)
        logger.info(f"Calibration results saved to: {output_file_path}")
        logger.info("You can now use these values to undistort images from the overhead camera.")

    except Exception as e:
        logger.error(f"An error occurred during calibration calculation: {e}", exc_info=True)

if __name__ == "__main__":
    # Ensure the WarehouseCameraClient is working standalone first for image fetching
    # This script is intended to be run from the project root or an environment where
    # server.src modules are in PYTHONPATH.
    # Example: python -m server.src.utils.calibrate_overhead_camera
    main_calibration_loop() 