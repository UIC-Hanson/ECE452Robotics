import cv2
import glob
import numpy as np
import yaml
import utils

# Constants
MARKER_LENGTH = 0.05  # Side length of the ArUco marker in meters
GOAL_ID = 2
HELPER1_ID = 1
HELPER2_ID = 0

def load_images(image_path):
    """Load images from a specified directory."""
    return glob.glob(image_path)

def find_corners(images, board_size, square_length, criteria, max_images=20):
    """Find and visualize chessboard corners in images."""
    objp = np.zeros((board_size[1] * board_size[0], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2) * square_length

    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane
    image_size = None

    count = 0
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = gray.shape[::-1]

        ret, corners = cv2.findChessboardCorners(gray, board_size, None)
        if ret:
            if count < max_images:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
                cv2.drawChessboardCorners(img, board_size, corners2, ret)
                cv2.imshow('img', img)
                cv2.waitKey(500)
                count += 1
            else:
                break

    cv2.destroyAllWindows()
    return objpoints, imgpoints, image_size

def calibrate_camera(objpoints, imgpoints, image_size):
    """Calibrate the camera given object points and image points."""
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)
    return ret, mtx, dist, rvecs, tvecs

def save_calibration_data(mtx, dist, filename='calib_data_new.yaml'):
    """Save the calibration data to a file."""
    calib_data = {'camera_matrix': np.asarray(mtx).tolist(),
                  'distortion_coefficients': np.asarray(dist).tolist()}
    with open(filename, 'w') as file:
        yaml.dump(calib_data, file)

def detect_and_draw_markers(frame, aruco_dict, aruco_params, mtx, dist):
    """ Detect markers and draw them on the frame. """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if corners:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, mtx, dist)
        cv2.aruco.drawDetectedMarkers(frame, corners)
        return corners, ids, rvecs, tvecs
    return None, None, None, None

def process_and_save_transformations():
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    square_length = 0.02  # Square size of the chessboard
    board_size = (7, 6)  # Board size
    images_path = './calib_images/*.jpg'
    
    images = load_images(images_path)
    objpoints, imgpoints, image_size = find_corners(images, board_size, square_length, criteria)
    
    if objpoints and imgpoints and image_size:
        ret, mtx, dist, rvecs, tvecs = calibrate_camera(objpoints, imgpoints, image_size)
        print("Calibration successful.")
        save_calibration_data(mtx, dist)
        return mtx, dist
    else:
        print("Calibration was not successful. Make sure there are images and detected points.")

def save_calibration_data(filepath, data):
    """ Save calibration data to a YAML file. """
    try:
        with open(filepath, 'w') as file:
            yaml.dump(data, file)
    except Exception as e:
        print(f"Error saving calibration data: {e}")

def main():
    mtx, dist = process_and_save_transformations()
    
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
    aruco_params = cv2.aruco.DetectorParameters_create()
    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

    cap = cv2.VideoCapture(cv2.CAP_V4L)

    transformations = {}

    try:
        print("Press 'q' to exit and save transformations.")
        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                corners, ids, rvecs, tvecs = detect_and_draw_markers(frame, aruco_dict, aruco_params, mtx, dist)
                if ids is not None:
                    for i, marker_id in enumerate(ids):
                        if marker_id in [GOAL_ID, HELPER1_ID, HELPER2_ID]:
                            cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], MARKER_LENGTH)
                            transformations[marker_id] = utils.cvdata2transmtx(rvecs[i], tvecs[i])[0]
                cv2.imshow('aruco', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()

    # Example usage of transformations
    if GOAL_ID in transformations and HELPER1_ID in transformations:
        g_gh1 = transformations[GOAL_ID].dot(transformations[HELPER1_ID])
        print(f"Helper1 location x:{g_gh1[0, 3]}, z:{g_gh1[2, 3]}")
        calib_data['g_gh1'] = np.asarray(g_gh1).tolist()

    if GOAL_ID in transformations and HELPER2_ID in transformations:
        g_gh2 = transformations[GOAL_ID].dot(transformations[HELPER2_ID])
        print(f"Helper2 location x:{g_gh2[0, 3]}, z:{g_gh2[2, 3]}")
        calib_data['g_gh2'] = np.asarray(g_gh2).tolist()

    save_calibration_data('calib_data.yaml', calib_data)


if __name__ == '__main__':
    main()
