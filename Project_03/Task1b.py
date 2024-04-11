import time
import cv2
import numpy as np
import yaml
import utils
import math
from picarx import Picarx

# Constants
INIT_ANGLE = -10
DESIRED_THETA = 10
MARKER_LENGTH = 0.1
CALIB_DATA_FILE = 'calib_data.yaml'

# Initialize Picarx
px = Picarx()

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

def calculate_transformation_matrix(markerCorners3D, markerCorners2D, mtx, dist):
    """Calculate the transformation matrix."""
    success, rvec, tvec = cv2.solvePnP(markerCorners3D, markerCorners2D, mtx, dist)
    return utils.cvdata2transmtx(rvec, tvec)[0]

def main():
    calib_data = load_calibration_data(CALIB_DATA_FILE)
    mtx = np.asarray(calib_data["camera_matrix"])
    dist = np.asarray(calib_data["distortion_coefficients"])
    detector = setup_aruco_detector()
    cap = cv2.VideoCapture(cv2.CAP_V4L)
    markerCorners3D = np.array([[-MARKER_LENGTH / 2, MARKER_LENGTH / 2, 0], [MARKER_LENGTH / 2, MARKER_LENGTH / 2, 0],
                                [MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0], [-MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0]])

    move_camera_to_angle(INIT_ANGLE)
    time.sleep(3)
    g0 = None
    actual_rot_angle = 0
    print("Start scanning the marker, you may quit the program by pressing q ...")

    for current_angle in range(INIT_ANGLE, 181, 2):
        move_camera_to_angle(current_angle)
        ret, frame = cap.read()
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
            if len(corners) != 0:
                markerCorners2D = np.array(corners[0]).reshape(-1, 2)
                if g0 is None:
                    g0 = calculate_transformation_matrix(markerCorners3D, markerCorners2D, mtx, dist)
                    print("Initial data saved...")
                else:
                    gth = calculate_transformation_matrix(markerCorners3D, markerCorners2D, mtx, dist)
                    exp_mtx = gth * np.linalg.inv(g0)
                    _, _, theta = utils.transmtx2twist(exp_mtx)
                    error = np.square(DESIRED_THETA - math.degrees(theta))
                    print(f"error: {error}")
                    if error <= 10:
                        actual_rot_angle = current_angle - INIT_ANGLE
                        break
        cv2.imshow('aruco', frame)
        cv2.waitKey(3000)

    print("Finished rotation...")
    print(f"Estimated rotation angle: {math.degrees(theta)} degrees")
    print(f"Actual rotation angle: {actual_rot_angle} degrees")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

