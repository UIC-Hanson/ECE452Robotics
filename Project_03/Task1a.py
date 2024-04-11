import time
import cv2
import numpy as np
import yaml
from picarx import Picarx
import utils
import math
import cv2.aruco as aruco

# Constants
INIT_ANGLE = -90
NEXT_ANGLE = 0
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
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    return detector

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
        [-MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0]
    ])
    
    move_camera_to_angle(INIT_ANGLE)
    time.sleep(3)

    init_rvec, init_tvec = None, None
    print("Press 's' to save the initial data or 'q' to quit...")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Failed to read from camera.")
            continue

        rvecs, tvecs = detect_and_draw_markers(frame, detector, mtx, dist, markerCorners3D)

        cv2.imshow("aruco", frame)
        
        # Key event handling
        key = cv2.waitKey(2) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s') and rvecs:
            init_rvec, init_tvec = rvecs[-1], tvecs[-1]
            print("Initial data saved, press 'm' to move the camera or 'q' to quit...")
        elif key == ord('m'):
            move_camera_to_angle(NEXT_ANGLE)
            print("Camera position changed, press 'r' to save current data or 'q' to quit...")
        elif key == ord('r') and rvecs:
            next_rvec, next_tvec = rvecs[-1], tvecs[-1]
            print("Current data saved, press 'q' to quit and start the calculation...")

    cap.release()
    cv2.destroyAllWindows()
    
    if init_rvec.all() and next_rvec.all():
        # g(0)
        g0 = utils.cvdata2transmtx(init_rvec,init_tvec)[0]
        # g(th)
        gth = utils.cvdata2transmtx(next_rvec,next_tvec)[0] 
        # ======== TO DO ========
        #find exp^(hat(xi)*th) using g(0) and g(th)
        exp_mtx = gth * np.linalg.inv(g0)
        #================================================
        # The twist coordinate and screw motion of the servo
        v,w,th = utils.transmtx2twist(exp_mtx)
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

