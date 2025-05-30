import cv2
import time
import os
from picamera2 import Picamera2

# --- Configuration ---
SAVE_DIR = "../calibration_images"  # Relative to this script's location
IMAGE_PREFIX = "calib_img_"
RESOLUTION = (640, 480) # Changed to 640x480
FRAME_RATE = 30
WAIT_TIME_MS = 30 # Delay between frames in preview loop (milliseconds)
# -------------------

def main():
    # Ensure save directory exists
    script_dir = os.path.dirname(__file__)
    save_path = os.path.abspath(os.path.join(script_dir, SAVE_DIR))
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print(f"Created directory: {save_path}")
    else:
        print(f"Saving images to: {save_path}")

    # Initialize Camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": RESOLUTION})
    picam2.configure(config)
    picam2.start()
    time.sleep(1) # Allow camera to settle

    print("Camera initialized.")
    print(f"Preview resolution: {RESOLUTION}")
    print("Press SPACE BAR to capture an image.")
    print("Press 'q' to quit.")

    img_counter = 0
    window_name = "Calibration Image Capture - Press SPACE to save, Q to quit"
    cv2.namedWindow(window_name)

    try:
        while True:
            # Capture frame for display
            frame = picam2.capture_array()
            # Picamera2 captures in RGB, OpenCV uses BGR by default
            display_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Show frame
            cv2.imshow(window_name, display_frame)
            key = cv2.waitKey(WAIT_TIME_MS) & 0xFF

            if key == ord(' '): # Space bar pressed
                img_name = os.path.join(save_path, f"{IMAGE_PREFIX}{img_counter:02d}.jpg")
                
                # Rotate the frame 180 degrees to make it right-side up before saving
                rotated_frame = cv2.rotate(frame, cv2.ROTATE_180)
                
                # Convert the rotated RGB frame to BGR for OpenCV imwrite
                bgr_frame_to_save = cv2.cvtColor(rotated_frame, cv2.COLOR_RGB2BGR)
                cv2.imwrite(img_name, bgr_frame_to_save)
                print(f"Image captured: {img_name} (rotated 180)")
                img_counter += 1
            elif key == ord('q'): # 'q' pressed
                print("Quitting capture.")
                break
    finally:
        # Cleanup
        print("Stopping camera and closing windows.")
        picam2.stop()
        cv2.destroyAllWindows()
        print("Done.")

if __name__ == "__main__":
    main() 