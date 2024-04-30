import cv2
import numpy as np
import yaml
import utils

# Constants
MARKER_LENGTH = 0.05  # Side length of the ArUco marker in meters
GOAL_ID = 2
HELPER1_ID = 1
HELPER2_ID = 0

def load_calibration_data(filepath):
    """ Load camera calibration data from a YAML file. """
    try:
        with open(filepath, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading calibration data: {e}")
        return None

def save_calibration_data(filepath, data):
    """ Save calibration data to a YAML file. """
    try:
        with open(filepath, 'w') as file:
            yaml.dump(data, file)
    except Exception as e:
        print(f"Error saving calibration data: {e}")

def detect_and_draw_markers(frame, aruco_dict, aruco_params, mtx, dist):
    """ Detect markers and draw them on the frame. """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if corners:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, mtx, dist)
        cv2.aruco.drawDetectedMarkers(frame, corners)
        return corners, ids, rvecs, tvecs
    return None, None, None, None

def main():
    calib_data = load_calibration_data('calib_data_bug1')
    if calib_data is None:
        return

    mtx = np.asarray(calib_data["camera_matrix"])
    dist = np.asarray(calib_data["distortion_coefficients"])

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
