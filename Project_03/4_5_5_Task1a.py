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
    """Load camera calibration data from YAML file."""
    with open(file_path) as file:
        calib_data = yaml.load(file, Loader=yaml.FullLoader)
    return calib_data

def setup_aruco_detector():
    """Set up the ArUco marker detector."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    aruco_params = cv2.aruco.DetectorParameters_create()
    return aruco_dict, aruco_params

def move_camera_to_angle(angle):
    """Move the camera to the specified angle."""
    try:
        px.set_cam_pan_angle(angle)
        time.sleep(1)  # Increased sleep for stability
    except Exception as e:
        print(f"Error setting camera angle to {angle}: {e}")

def detect_and_draw_markers(frame, aruco_dict, aruco_params, mtx, dist, marker_length):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if len(corners)!=0: # if aruco marker detected
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.05)
    return rvec, tvec

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
    
    print("Press 's' to save the initial data or 'q' to quit...")
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Failed to read from camera.")
            continue

        rvec, tvec = detect_and_draw_markers(frame, aruco_dict, aruco_params, mtx, dist, MARKER_LENGTH)
        cv2.imshow("aruco", frame)

        key = cv2.waitKey(2) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            init_rvec, init_tvec = rvec, tvec
            print("Initial data saved, press 'm' to move the camera or 'q' to quit...")
        elif key == ord('m'):
            move_camera_to_angle(NEXT_ANGLE)
            print("Camera position changed, press 'r' to save current data or 'q' to quit...")
        elif key == ord('r'):
            next_rvec, next_tvec = rvec[-1], tvec[-1]
            print("Current data saved, press 'q' to quit and start the calculation...")

    # Turn off the camera
    cap.release()
    cv2.destroyAllWindows()

    if init_rvec is not None and next_rvec is not None:
        # g(0)
        g0 = utils.cvdata2transmtx(init_rvec, init_tvec)[0]
        # g(th)
        gth = utils.cvdata2transmtx(next_rvec, next_tvec)[0]
        # Calculate exp^(hat(xi)*th) using g(0) and g(th)
        exp_mtx = gth @ np.linalg.inv(g0)
        # The twist coordinate and screw motion of the servo
        v, w, th = utils.transmtx2twist(exp_mtx)
        q,h,u,M = utils.twist2screw(v,w,th)
        print("Estimated rotation angle: {} degrees".format(math.degrees(th)))
        print("Twist Coordinates:\n {}".format(np.vstack((v,w))*th))
        print("Screw motion:\n q:{},\n h:{},\n u:{},\n M:{}".format(q,h,u,M))
        # Reset servo and camera angles, stop motors
        px.set_cam_tilt_angle(0)
        px.set_cam_pan_angle(0)  
        px.set_dir_servo_angle(0)  
        px.stop()
        print("Robot stopped.")
if __name__ == "__main__":
    main()
