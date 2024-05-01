import cv2
import numpy as np
import yaml
import utils
import math
import time

class point:
    def __init__(self):
        self.x = None
        self.z = None

def initialize_aruco():
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
    aruco_params = cv2.aruco.DetectorParameters_create()
    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
    return aruco_dict, aruco_params

def load_calibration(filename):
    with open(filename) as file:
        calib_data = yaml.load(file, Loader=yaml.FullLoader)
    mtx = np.asarray(calib_data["camera_matrix"])
    dist = np.asarray(calib_data["distortion_coefficients"])
    return mtx, dist

def save_calibration(filename, calib_data):
    with open(filename, 'w') as file:
        yaml.dump(calib_data, file)

def detect_markers(frame, aruco_dict, aruco_params, marker_length, mtx, dist):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if len(corners) != 0:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
        cv2.aruco.drawDetectedMarkers(frame, corners)
        return corners, ids, rvecs, tvecs
    return None, None, None, None

def process_video(cap, aruco_dict, aruco_params, mtx, dist, goal_id, max_helpers):
    marker_length = 0.05
    goal = point()
    helpers = {id_: point() for id_ in range(1, max_helpers + 1)}  # Initialize points for each helper
    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            corners, ids, rvecs, tvecs = detect_markers(frame, aruco_dict, aruco_params, marker_length, mtx, dist)
            if ids is not None:
                for i, id_ in enumerate(ids):
                    if id_ == goal_id:
                        cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                        g_gc = utils.cvdata2transmtx(rvecs[i], tvecs[i])[0]
                        p_gc = g_gc[:, 3]
                        goal.x = p_gc[0]
                        goal.z = p_gc[2]
                        print(f"Goal point x:{goal.x}, z:{goal.z}")
                    elif id_ in helpers:
                        cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                        g_hc = utils.cvdata2transmtx(rvecs[i], tvecs[i])[0]
                        p_hc = g_hc[:, 3]
                        helpers[id_].x = p_hc[0]
                        helpers[id_].z = p_hc[2]
                        print(f"Helper {id_} point x:{helpers[id_].x}, z:{helpers[id_].z}")
            cv2.imshow('aruco', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    cap.release()
    cv2.destroyAllWindows()
    return goal, helpers

def output():
    aruco_dict, aruco_params = initialize_aruco()
    mtx, dist = load_calibration('calib_data.yaml')
    cap = cv2.VideoCapture(cv2.CAP_V4L)
    goal, helpers = process_video(cap, aruco_dict, aruco_params, mtx, dist, 0, [1, 2])
    return goal, helpers

def find_trans_main():
    aruco_dict, aruco_params = initialize_aruco()
    mtx, dist = load_calibration('calib_data.yaml')
    cap = cv2.VideoCapture(cv2.CAP_V4L)
    # Assuming helper IDs range from 1 to 5, pass the maximum helper ID
    goal, helpers = process_video(cap, aruco_dict, aruco_params, mtx, dist, 0, 5)
    print(f"Detected goal point at x: {goal.x}, z: {goal.z}")
    for id_, helper in helpers.items():
        print(f"Detected helper {id_} point at x: {helper.x}, z: {helper.z}")