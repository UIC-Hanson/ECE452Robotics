from picarx import Picarx
import cv2
import numpy as np
import yaml
import utils
import time

def initialize_robot():
    # Initialize Picarx robot
    px = Picarx()
    return px

def load_calibration_data(filepath):
    # Load camera calibration data from a YAML file
    with open(filepath) as file:
        calib_data = yaml.load(file, Loader=yaml.FullLoader)
    mtx = np.asarray(calib_data["camera_matrix"])
    dist = np.asarray(calib_data["distortion_coefficients"])
    return mtx, dist

def initialize_camera():
    # Initialize the camera
    cap = cv2.VideoCapture(cv2.CAP_V4L)
    if not cap.isOpened():
        raise IOError("Cannot open video camera")
    return cap

def main():
    try:
        px = initialize_robot()
        mtx, dist = load_calibration_data('calib_data.yaml')
        cap = initialize_camera()
        
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        aruco_params = cv2.aruco.DetectorParameters_create()
        
        print("Start running task 2...")
        
        marker_length = 0.1  # Side length of the ArUco marker in meters
        state_flag = 0  # Manage navigation states
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            
            if len(corners) > 0:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
                g, _, p = utils.cvdata2transmtx(rvec, tvec)
                _, _, th = utils.transmtx2twist(g)

                # Placeholder for state handling logic
                print(f"Detected marker with ID: {ids[0][0]}")
                # Additional navigation logic based on state_flag
                
            time.sleep(1)

    except Exception as e:
        print(f"An error occurred: {e}")
    
    finally:
        px.forward(0)  # Ensure the robot stops moving
        cap.release()
        cv2.destroyAllWindows()