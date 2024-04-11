import time
import cv2
import numpy as np
import yaml
import utils
import math
from picarx import Picarx
from time import sleep

# Constants
INIT_ANGLE = -60
DESIRED_THETA = 10
MARKER_LENGTH = 0.1
CALIB_DATA_FILE = 'calib_data.yaml'

# Initialize Picarx
px = Picarx()

def initialize_robot():
    # Initialize settings
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)
    sleep(.5)

def load_calibration_data(file_path):
    """Load camera calibration data from YAML file."""
    with open(file_path) as file:
        return yaml.load(file, Loader=yaml.FullLoader)

def setup_aruco_detector():
    """Setup the ArUco marker detector."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    aruco_params = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

def move_camera_to_angle(angle):
    """Move the camera to the specified angle."""
    try:
        px.set_cam_pan_angle(angle)
        time.sleep(0.01)
    except Exception as e:
        print(f"Error setting camera angle to {angle}: {e}")

def detect_and_draw_markers(frame, detector, mtx, dist, markerCorners3D):
    """Detect ArUco markers and return their rotation and translation vectors."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    rvecs, tvecs = [], []

    if len(corners) > 0:
        for corner in corners:
            markerCorners2D = np.array(corner).reshape(-1, 2)
            success, rvec, tvec = cv2.solvePnP(markerCorners3D, markerCorners2D, mtx, dist)
            rvecs.append(rvec)
            tvecs.append(tvec)
            
        cv2.aruco.drawDetectedMarkers(frame, corners, ids, (0, 255, 0))

        # Draw frame axes for each detected marker
        cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, MARKER_LENGTH * 1.5, 2)
    return rvecs, tvecs

def calculate_transformation_matrix(markerCorners3D, markerCorners2D, mtx, dist):
    """Calculate the transformation matrix."""
    success, rvec, tvec = cv2.solvePnP(markerCorners3D, markerCorners2D, mtx, dist)
    return utils.cvdata2transmtx(rvec, tvec)[0]

def main():
    detector = setup_aruco_detector()
    calib_data = load_calibration_data(CALIB_DATA_FILE)
    mtx = np.asarray(calib_data["camera_matrix"])
    dist = np.asarray(calib_data["distortion_coefficients"])
    cap = cv2.VideoCapture(cv2.CAP_V4L)

    # Define 3D coordinates for ArUco marker corners
    markerCorners3D = np.array([
        [-MARKER_LENGTH / 2, MARKER_LENGTH / 2, 0],
        [MARKER_LENGTH / 2, MARKER_LENGTH / 2, 0],
        [MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0],
        [-MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0]])

    runs = 5  # Number of runs for the experiment
    errors = []  # List to record errors for each run
    
    for run in range(runs):
        print(f"Run {run + 1}/{runs}")
        initialize_robot()  # Initialize the robot for each run
        
        # Reset or reinitialize variables as needed
        g0 = None
        actual_rot_angle = 0
        estimated_rot_angle = 0  # Variable to hold the estimated rotation angle based on ArUco detection

        for current_angle in range(INIT_ANGLE, 181, 10):
            move_camera_to_angle(current_angle)
            print(f"Current camera angle: {current_angle}")  # Print current angle for verification
            
            ret, frame = cap.read()
            if ret:
                rvecs, tvecs = detect_and_draw_markers(frame, detector, mtx, dist, markerCorners3D)
                if len(rvecs) > 0 and len(tvecs) > 0:  # Checks if at least one marker is detected
                    # You should use corners here instead. 
                    # Assuming corners are being returned by detect_and_draw_markers, but it needs to be adjusted to do so.
                    corners = corners[0]  # Use the first detected marker's corners
                    if g0 is None:
                        g0 = calculate_transformation_matrix(markerCorners3D, corners.reshape(-1, 2), mtx, dist)
                        print("Initial data saved...")
                    else:
                        gth = calculate_transformation_matrix(markerCorners3D, corners.reshape(-1, 2), mtx, dist)
                        exp_mtx = gth @ np.linalg.inv(g0)
                        _, _, theta = utils.transmtx2twist(exp_mtx)
                        estimated_rot_angle = math.degrees(theta)
                        error = abs(DESIRED_THETA - estimated_rot_angle)
                        print(f"Estimated rotation angle: {estimated_rot_angle} degrees, error: {error}")
                        if error <= 10:
                            actual_rot_angle = current_angle - INIT_ANGLE
                            print(f"Actual rotation angle: {actual_rot_angle} degrees")
                            break
                
                cv2.imshow('aruco', frame)
                
                key = cv2.waitKey(2) & 0xFF
                if key == ord('q'):
                    break
                cv2.waitKey(300)  # Reduced delay for more responsive feedback
        
        error = abs(actual_rot_angle - estimated_rot_angle)
        errors.append(error)
        
        print(f"Run {run + 1} error: {error} degrees")
        time.sleep(2)  # Short pause between runs, adjust as necessary

    average_error = np.mean(errors)
    std_dev_error = np.std(errors)
    
    print(f"Average Error: {average_error} degrees")
    print(f"Standard Deviation of Error: {std_dev_error} degrees")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    initialize_robot()
    main()

