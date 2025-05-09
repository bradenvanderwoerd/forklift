import cv2
import cv2.aruco as aruco
import numpy as np
import glob
import os

# --- Configuration ---
IMAGES_DIR = "../calibration_images/" # Relative to this script
IMAGE_FORMAT = "*.jpg" # Image file format
CALIBRATION_FILE = "../camera_calibration.npz" # Output file, relative path

# ChArUco board parameters (UPDATE THESE TO MATCH YOUR BOARD!)
PATTERN_SIZE = (6, 8) # Internal corners: (cols-1, rows-1)
SQUARE_LENGTH = 0.0204 # Square size in meters
MARKER_LENGTH = 0.015 # ArUco marker size in meters
ARUCO_DICT_NAME = aruco.DICT_7X7_50 # The dictionary used for markers ON the board
# -------------------

def calibrate():
    print("Starting ChArUco calibration...")
    # Define ArUco dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT_NAME)
    params = aruco.DetectorParameters()
    aruco_detector = aruco.ArucoDetector(aruco_dict, params)

    # Create ChArUco board object
    board = aruco.CharucoBoard(PATTERN_SIZE, SQUARE_LENGTH, MARKER_LENGTH, aruco_dict)
    print(f"ChArUco board created: Pattern={PATTERN_SIZE}, Square={SQUARE_LENGTH}m, Marker={MARKER_LENGTH}m, Dict={ARUCO_DICT_NAME}")

    # Prepare lists to store object points and image points from all images
    all_charuco_corners = [] # Pixel coordinates of detected charuco corners
    all_charuco_ids = [] # IDs of detected charuco corners

    # Find image files
    script_dir = os.path.dirname(__file__)
    images_path_pattern = os.path.join(script_dir, IMAGES_DIR, IMAGE_FORMAT)
    images = sorted(glob.glob(images_path_pattern))

    if not images:
        print(f"Error: No images found at '{images_path_pattern}'. Did you run capture script?")
        return

    print(f"Found {len(images)} images for calibration.")

    # Analyze each image
    img_size = None
    for fname in images:
        print(f"Processing {os.path.basename(fname)}...")
        img = cv2.imread(fname)
        if img is None:
            print(f"  Warning: Could not read image {fname}")
            continue
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = gray.shape[::-1] # (width, height)
            print(f"  Image size: {img_size}")
        elif gray.shape[::-1] != img_size:
             print(f"  Warning: Image size mismatch ({gray.shape[::-1]}) compared to first image ({img_size}). Skipping.")
             continue

        # Detect ArUco markers
        corners, ids, rejected = aruco_detector.detectMarkers(gray)

        # If markers are found, interpolate ChArUco corners
        if ids is not None and len(ids) > 3: # Need enough markers for interpolation
            response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                corners, ids, gray, board
            )
            # Log the number of corners found for every attempt
            print(f"  InterpolateCornersCharuco response (corners found): {response}")
            
            # If enough corners were found, store the results
            if response > 10: # Require a decent number of corners found
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
                # print(f"  Detected {response} ChArUco corners.") # Redundant now
                # Optional: Draw and display corners found - UNCOMMENTED FOR DEBUGGING
                img_drawn = aruco.drawDetectedCornersCharuco(img.copy(), charuco_corners, charuco_ids)
                cv2.imshow('Charuco Detection', cv2.resize(img_drawn, (img_size[0]//2, img_size[1]//2)))
                cv2.waitKey(0) # Wait indefinitely until a key is pressed
            else:
                # print(f"  Warning: Not enough ChArUco corners found ({response}).") # Redundant now
                # Also show images where detection failed or was low
                if ids is not None:
                    img_drawn = aruco.drawDetectedMarkers(img.copy(), corners, ids)
                    cv2.imshow('Charuco Detection', cv2.resize(img_drawn, (img_size[0]//2, img_size[1]//2)))
                    cv2.waitKey(0)
        else:
            print("  Warning: Not enough ArUco markers detected.")
            # Show the image even if no markers found
            cv2.imshow('Charuco Detection', cv2.resize(img.copy(), (img_size[0]//2, img_size[1]//2)))
            cv2.waitKey(0)

    cv2.destroyAllWindows()

    if len(all_charuco_corners) < 5:
        print("Error: Not enough valid images with detected ChArUco corners for calibration.")
        print(f"Need at least 5, found {len(all_charuco_corners)}.")
        return

    print(f"\nStarting calibration process with {len(all_charuco_corners)} image views...")

    # Calibrate camera
    try:
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            all_charuco_corners, all_charuco_ids, board, img_size, None, None
        )
        
        # Calculate reprojection error (optional but recommended)
        total_error = 0
        for i in range(len(all_charuco_corners)):
            img_points2, _ = cv2.projectPoints(board.getChessboardCorners()[all_charuco_ids[i][:,0]], 
                                               rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
            error = cv2.norm(all_charuco_corners[i], img_points2, cv2.NORM_L2) / len(img_points2)
            total_error += error
        mean_error = total_error / len(all_charuco_corners)

        if ret:
            print("\nCalibration successful!")
            print("Camera Matrix (mtx):")
            print(camera_matrix)
            print("\nDistortion Coefficients (dist):")
            print(dist_coeffs)
            print(f"\nMean Reprojection Error: {mean_error:.4f} pixels")
            if mean_error > 1.0:
                 print("  Warning: Mean reprojection error is high (> 1.0 pixels). Calibration might be inaccurate.")
                 print("  Consider retaking images with more variety or checking board planarity.")

            # Save calibration results
            output_file = os.path.abspath(os.path.join(script_dir, CALIBRATION_FILE))
            np.savez(output_file, mtx=camera_matrix, dist=dist_coeffs)
            print(f"\nCalibration results saved to: {output_file}")
        else:
            print("\nCalibration failed.")

    except Exception as e:
        print(f"\nAn error occurred during calibration: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    calibrate() 