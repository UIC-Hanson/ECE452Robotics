import time
import cv2
import numpy as np
import yaml
from picarx import Picarx
import utils
import math
import cv2.aruco as aruco

# Constants
INIT_ANGLE = 0
NEXT_ANGLE = 20
MARKER_LENGTH = 0.1
CALIB_DATA_FILE = 'calib_data.yaml'

# Initialize Picarx
px = Picarx()

def load_calibration_data(file_path):
    """Load camera calibration data from YAML file."""
    with open(file_path) as file:
        calib_data = yaml.load(file, Loader=yaml.FullLoader)
    return calib_data

def setup_aruco_detector():
    """Set up the ArUco marker detector."""
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
    aruco_params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, aruco_params)
    return detector

def move_camera_to_angle(angle):
    """Move the camera to the specified angle."""
    try:
        px.set_cam_pan_angle(angle)
        time.sleep(0.01)
    except Exception as e:
        print(f"Error setting camera angle to {angle}: {e}")

def main():
    # Load calibration data
    calib_data = load_calibration_data(CALIB_DATA_FILE)
    mtx = np.asarray(calib_data["camera_matrix"])
    dist = np.asarray(calib_data["distortion_coefficients"])

    # Set up ArUco detector
    detector = setup_aruco_detector()

    # Initialize camera
    cap = cv2.VideoCapture(cv2.CAP_V4L)

    # Move camera to initial angle
    move_camera_to_angle(INIT_ANGLE)
    time.sleep(3)

    # Start detecting the ArUco marker
    print("Press s to save the initial data or press q to quit...")
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            continue
        # Processing frame for ArUco detection
        # [Your existing ArUco detection and processing code goes here]

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    # [Rest of your code for calculations and resetting the robot]

if __name__ == "__main__":
    main()

