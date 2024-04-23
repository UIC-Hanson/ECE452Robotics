#Project 4 Calibrate

import cv2
import glob
import numpy as np
import yaml
import utils  # Assuming utils module has necessary functions

# Constants
MARKER_LENGTH = 0.05  # Side length of the ArUco marker in meters
GOAL_ID = 2
HELPER1_ID = 1
HELPER2_ID = 0

# Load and save functions for YAML
def load_calibration_data(filepath):
    try:
        with open(filepath, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading calibration data: {e}")
        return None

def save_calibration_data(filepath, data):
    try:
        with open(filepath, 'w') as file:
            yaml.dump(data, file)
    except Exception as e:
        print(f"Error saving calibration data: {e}")

# Image processing functions
def load_images(image_path):
    return glob.glob(image_path)

def find_corners(images, board_size, square_length, criteria, max_images=20):
    objp = np.zeros((board_size[1] * board_size[0], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2) * square_length
    objpoints, imgpoints, image_size = [], [], None
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = gray.shape[::-1]
        ret, corners = cv2.findChessboardCorners(gray, board_size, None)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
    return objpoints, imgpoints, image_size

def calibrate_camera(objpoints, imgpoints, image_size):
    return cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)

def detect_and_draw_markers(frame, aruco_dict, aruco_params, mtx, dist):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if corners:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, mtx, dist)
        cv2.aruco.drawDetectedMarkers(frame, corners)
        return corners, ids, rvecs, tvecs
    return None, None, None, None

# Main function
def main():
    # Calibration part
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    square_length = 0.02  # Chessboard square size
    board_size = (7, 6)
    images_path = './calib_images/*.jpg'
    images = load_images(images_path)
    objpoints, imgpoints, image_size = find_corners(images, board_size, square_length, criteria)
    if objpoints and imgpoints and image_size:
        ret, mtx, dist, rvecs, tvecs = calibrate_camera(objpoints, imgpoints, image_size)
        print("Calibration successful.")
        save_calibration_data('calib_data.yaml', {'camera_matrix': mtx.tolist(), 'distortion_coefficients': dist.tolist()})

    # ArUco detection part
    calib_data = load_calibration_data('calib_data_Bug1_New.yaml')
    if not calib_data:
        return
    mtx = np.asarray(calib_data["camera_matrix"])
    dist = np.asarray(calib_data["distortion_coefficients"])
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
    aruco_params = cv2.aruco.DetectorParameters_create()
    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
    cap = cv2.VideoCapture(cv2.CAP_V4L)
    transformations = {}

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                corners, ids, rvecs, tvecs = detect_and_draw_markers(frame, aruco_dict, aruco_params, mtx, dist)
                if ids is not None:
                    for i, marker_id in enumerate(ids.flatten()):
                        if marker_id in [GOAL_ID, HELPER1_ID, HELPER2_ID]:
                            transformations[marker_id] = utils.cvdata2transmtx(rvecs[i], tvecs[i])[0]
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
