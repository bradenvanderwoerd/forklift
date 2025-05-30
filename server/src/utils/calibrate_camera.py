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
SQUARES_X = 7 # Number of squares in X direction
SQUARES_Y = 9 # Number of squares in Y direction
# PATTERN_SIZE = (6, 8) # Internal corners: (cols-1, rows-1) - This is for calibrateCameraCharuco, not board creation
SQUARE_LENGTH = 0.0204 # Square size in meters
MARKER_LENGTH = 0.015 # ArUco marker size in meters
ARUCO_DICT_NAME = aruco.DICT_7X7_1000 # Trying 1000 unique markers
DEBUG_OUTPUT_DIR = "debug_detections" # Directory for debug images
# -------------------

def calibrate():
    print("Starting ChArUco calibration...")
    # Define ArUco dictionary and parameters
    aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT_NAME)
    
    # Simpler DetectorParameters for testing
    params = aruco.DetectorParameters() 
    # params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX # Reverted for now
    
    aruco_detector = aruco.ArucoDetector(aruco_dict, params)

    # Create ChArUco board object
    # Corrected: CharucoBoard takes the number of squares (SQUARES_X, SQUARES_Y)
    # PATTERN_SIZE (cols-1, rows-1) is used later for calibrateCameraCharuco via board.getChessboardCorners()
    board = aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_LENGTH, MARKER_LENGTH, aruco_dict)
    print(f"ChArUco board created: Squares=({SQUARES_X},{SQUARES_Y}), Square={SQUARE_LENGTH}m, Marker={MARKER_LENGTH}m, Dict={ARUCO_DICT_NAME}")

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

    # Create debug output directory if it doesn't exist
    debug_dir_path = os.path.join(script_dir, DEBUG_OUTPUT_DIR)
    if not os.path.exists(debug_dir_path):
        os.makedirs(debug_dir_path)
        print(f"Created debug directory: {debug_dir_path}")

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

        # --- Start Enhanced Debugging ---
        if ids is not None:
            print(f"  Detected ArUco IDs: {ids.flatten().tolist()}") # Print detected IDs
            img_for_debug_drawing = img.copy()
            aruco.drawDetectedMarkers(img_for_debug_drawing, corners, ids)
            debug_image_filename = f"debug_markers_on_{os.path.basename(fname)}"
            debug_image_path = os.path.join(debug_dir_path, debug_image_filename)
            try:
                cv2.imwrite(debug_image_path, img_for_debug_drawing)
                print(f"  Saved debug image with detected markers to: {debug_image_path}")
            except Exception as e:
                print(f"  Error saving debug image {debug_image_path}: {e}")
        else:
            print("  No ArUco markers detected in this image by detectMarkers.")
        # --- End Enhanced Debugging ---

        # If markers are found, interpolate ChArUco corners
        if ids is not None and len(ids) > 3: # Need enough markers for interpolation
            # Try with explicit None for cameraMatrix and distCoeffs
            response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                corners, ids, gray, board, cameraMatrix=None, distCoeffs=None
            )
            # Log the number of corners found for every attempt
            print(f"  InterpolateCornersCharuco response (corners found): {response}")
            
            # If enough corners were found, store the results
            if response > 10: # Require a decent number of corners found
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
                # print(f"  Detected {response} ChArUco corners.") # Redundant now
                # Optional: Draw and display corners found
                img_drawn = aruco.drawDetectedCornersCharuco(img.copy(), charuco_corners, charuco_ids)
                cv2.imshow('Charuco Detection Result', cv2.resize(img_drawn, (img_size[0]//2, img_size[1]//2)))
                cv2.waitKey(0) 
            else:
                # print(f"  Warning: Not enough ChArUco corners found ({response}).") # Redundant now
                # Also show images where detection failed or was low, now with detected ArUco markers
                # This part is covered by the debug image saving, but imshow can be useful for quick checks
                if ids is not None: # Ensure ids exist before drawing
                    img_drawn_markers_only = aruco.drawDetectedMarkers(img.copy(), corners, ids)
                    cv2.imshow('Charuco Detection Result (Markers Only)', cv2.resize(img_drawn_markers_only, (img_size[0]//2, img_size[1]//2)))
                    cv2.waitKey(0)
        else:
            # This print was "Warning: Not enough ArUco markers detected."
            # The enhanced debugging section above already prints if no markers are detected or prints their IDs.
            # So, if we reach here, it means ids were None or len(ids) <= 3.
            # The specific condition is already covered by the print statements in the "Enhanced Debugging" block.
            print("  Skipping ChArUco interpolation: Not enough ArUco markers detected initially (need > 3).")
            # Show the image even if no markers found or too few for ChArUco.
            # The debug image saving will capture this if any markers were found at all.
            # If no markers were found, just show the plain image.
            if ids is None:
                cv2.imshow('Charuco Detection Result (Original Image)', cv2.resize(img.copy(), (img_size[0]//2, img_size[1]//2)))
                cv2.waitKey(0)

    cv2.destroyAllWindows()

    if len(all_charuco_corners) < 5:
        print("Error: Not enough valid images with detected ChArUco corners for calibration.")
        print(f"Need at least 5, found {len(all_charuco_corners)}.")
        return

    print(f"\nStarting calibration process with {len(all_charuco_corners)} image views...")

    # Calibrate camera
    try:
        # The board object (which defines the ChArUco pattern including square/marker sizes and ID layout)
        # and img_size are crucial.
        # all_charuco_corners and all_charuco_ids are the detected points from the images.
        
        # The actual pattern size for calibration (number of internal corners) is derived 
        # implicitly from the board object's dimensions (SQUARES_X-1, SQUARES_Y-1).
        # We don't need to pass PATTERN_SIZE directly to calibrateCameraCharuco if using the board object.
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            all_charuco_corners, all_charuco_ids, board, img_size, None, None
        )
        
        # Calculate reprojection error (optional but recommended)
        total_error = 0
        for i in range(len(all_charuco_corners)):
            # Get the 3D object points for the detected charuco corners for this specific view
            obj_points_for_view = np.zeros((len(all_charuco_ids[i]), 3), dtype=np.float32)
            for k, corner_id in enumerate(all_charuco_ids[i].flatten()):
                # board.getChessboardCorners() gives all corners. Charuco IDs are indices into this.
                # However, the charuco board object has a specific attribute 'chessboardCorners'
                # which contains the 3D coordinates of the ChArUco corners.
                # Each charuco_id in all_charuco_ids[i] is an index into board.chessboardCorners
                if corner_id < len(board.getChessboardCorners()): # Check if ID is valid
                     obj_points_for_view[k] = board.getChessboardCorners()[corner_id]
                else:
                    # This case should ideally not happen if interpolateCornersCharuco is working correctly
                    # and charuco_ids are valid indices for the board.
                    print(f"Warning: charuco_id {corner_id} out of bounds for board.chessboardCorners")
                    # Fallback or skip this point for error calculation if necessary
                    # For simplicity, let's assume valid IDs from interpolateCornersCharuco
                    # If this warning appears, it indicates a deeper issue.

            img_points2, _ = cv2.projectPoints(obj_points_for_view, 
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