import time
import cv2
import numpy as np
import yaml
from picarx import Picarx
import utils
import math

# Constants
INIT_ANGLE = 0
NEXT_ANGLE = 25
MARKER_LENGTH = 0.1
CALIB_DATA_FILE = 'calib_data.yaml'

# Initialize Picarx
px = Picarx()

def load_calibration_data(file_path):
    with open(file_path) as file:
        calib_data = yaml.load(file, Loader=yaml.FullLoader)
    return calib_data

def setup_aruco_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    aruco_params = cv2.aruco.DetectorParameters_create()
    return aruco_dict, aruco_params

def move_camera_to_angle(angle):
    try:
        px.set_cam_pan_angle(angle)
        time.sleep(1)  # Increased sleep for stability
    except Exception as e:
        print(f"Error setting camera angle to {angle}: {e}")

def detect_and_draw_markers(frame, aruco_dict, aruco_params, mtx, dist, marker_length):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    rvecs, tvecs = [], []
    if corners:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for rvec, tvec in zip(rvecs, tvecs):
            cv2.aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.05)
    return rvecs, tvecs

def main():
    aruco_dict, aruco_params = setup_aruco_detector()
    calib_data = load_calibration_data(CALIB_DATA_FILE)
    mtx = np.asarray(calib_data["camera_matrix"])
    dist = np.asarray(calib_data["distortion_coefficients"])
    cap = cv2.VideoCapture(cv2.CAP_V4L)
    if not cap.isOpened():
        print("Camera could not be opened.")
        return

    move_camera_to_angle(INIT_ANGLE)
    time.sleep(3)

    init_rvec, init_tvec, next_rvec, next_tvec = None, None, None, None
    print("Press 's' to save the initial data or 'q' to quit...")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Failed to read from camera.")
            continue

        rvecs, tvecs = detect_and_draw_markers(frame, aruco_dict, aruco_params, mtx, dist, MARKER_LENGTH)
        cv2.imshow("aruco", frame)

        key = cv2.waitKey(2) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s') and rvecs:
            init_rvec, init_tvec = rvecs[-1], tvecs[-1]
            print
