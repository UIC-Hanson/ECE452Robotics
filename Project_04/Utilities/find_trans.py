import cv2
import numpy as np
import yaml
import utils
import math

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
        transformations = []
        for i, _ in enumerate(rvecs):
            transformation = (ids[i], utils.cvdata2transmtx(rvecs[i], tvecs[i])[0])
            cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
            transformations.append(transformation)
        return transformations
    return []

def process_video(camera_id, marker_length, goal_id, helper_id, mtx, dist, aruco_dict, aruco_params):
    cap = cv2.VideoCapture(camera_id)
    print(f"Press q to save the transformation between Goal and Helper {helper_id}")
    while cap.isOpened():
        ret, frame = cap.read()
        if ret:
            transformations = detect_markers(frame, aruco_dict, aruco_params, marker_length, mtx, dist)
            for id, trans in transformations:
                if id == goal_id:
                    g_goal = trans
                elif id == helper_id:
                    g_helper = trans
            cv2.imshow('aruco', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
    cap.release()
    cv2.destroyAllWindows()
    return g_goal.dot(g_helper)

def find_trans_main():
    aruco_dict, aruco_params = initialize_aruco()
    mtx, dist = load_calibration('calib_data.yaml')
    g_gh1 = process_video(cv2.CAP_V4L, 0.05, 2, 1, mtx, dist, aruco_dict, aruco_params)
    g_gh2 = process_video(cv2.CAP_V4L, 0.05, 2, 0, mtx, dist, aruco_dict, aruco_params)
    print(f"Helper1 location x:{g_gh1[0,3]}, z:{g_gh1[2,3]}")
    print(f"Helper2 location x:{g_gh2[0,3]}, z:{g_gh2[2,3]}")
    calib_data = {
        'camera_matrix': np.asarray(mtx).tolist(),
        'distortion_coefficients': np.asarray(dist).tolist(),
        'g_gh1': np.asarray(g_gh1).tolist(),
        'g_gh2': np.asarray(g_gh2).tolist()
    }
    save_calibration('calib_data.yaml', calib_data)

if __name__ == '__main__':
    find_trans_main()
