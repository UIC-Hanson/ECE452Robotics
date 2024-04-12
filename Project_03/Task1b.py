from picarx import Picarx
import time
import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
import utils
import math

px = Picarx()

def load_calibration_data(calibration_file):
    with open(calibration_file) as file:
        calib_data = yaml.load(file, Loader=yaml.FullLoader)
    mtx = np.asarray(calib_data["camera_matrix"])
    dist = np.asarray(calib_data["distortion_coefficients"])
    return mtx, dist

def initialize_camera():
    return cv2.VideoCapture(cv2.CAP_V4L)

def set_camera_angle(angle):
    px.set_cam_pan_angle(angle)
    time.sleep(.3)  # Wait for the servo to move and stabilize

def detect_aruco_markers(frame, aruco_dict, aruco_params, mtx, dist):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

def estimate_pose(corners, marker_length, mtx, dist):
    return cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)

def get_initial_transformation(rvec, tvec):
    return utils.cvdata2transmtx(rvec, tvec)[0]

def calculate_rotation_matrix(g0, gth):
    return np.dot(np.linalg.inv(g0), gth)

def get_rotation_angle(exp_mtx):
    _, _, th = utils.transmtx2twist(exp_mtx)
    return th

def main():
    # Load resources
    mtx, dist = load_calibration_data('calib_data.yaml')
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    aruco_params = aruco.DetectorParameters_create()
    cap = initialize_camera()
    
    # Define initial conditions
    init_angle = -20
    desired_th_deg = 20
    g0 = None
    first_detected_angle = None
    last_detected_angle = None
    theta = None # Estimated rotation angle in radians

    # Start camera control
    set_camera_angle(init_angle)
    print("Start scanning the marker, you may quit the program by pressing 'q'...")

    for current_angle in range(init_angle, desired_th_deg, 1):
        set_camera_angle(current_angle)
        ret, frame = cap.read()
        if not ret:
            break
        
        corners, ids, _ = detect_aruco_markers(frame, aruco_dict, aruco_params, mtx, dist)
        
        if corners:
            rvec, tvec, _ = estimate_pose(corners, 0.1, mtx, dist)
            cv2.aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.05)
            
            if g0 is None:
                g0 = get_initial_transformation(rvec, tvec)
                first_detected_angle = current_angle
                print("First detection at angle:", first_detected_angle)
            
            last_detected_angle = current_angle
            gth = get_initial_transformation(rvec, tvec)
            exp_mtx = calculate_rotation_matrix(g0, gth)
            th = get_rotation_angle(exp_mtx)
            estimated_th_deg = math.degrees(th)
            print(f"Desired angle reached: {estimated_th_deg} degrees")
            
            if abs(desired_th_deg - estimated_th_deg) <= 5:
                theta = th
                print(f"Desired angle reached: {estimated_th_deg} degrees")
                break
        
        cv2.imshow('Frame', frame)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    # Calculate midpoint and return to it
    if first_detected_angle is not None and last_detected_angle is not None:
        midpoint_angle = (first_detected_angle + last_detected_angle) // 2
        set_camera_angle(midpoint_angle)
        print(f"Returned to midpoint angle: {midpoint_angle} degrees")
    else:
        print("Marker was not consistently detected; cannot return to midpoint position.")

    # Final output
    print("Finished rotation...")
    if theta is not None:
        print(f"Estimated rotation angle: {math.degrees(theta)} degrees")
    else:
        print("Estimated rotation angle not determined.")

    actual_rot_angle = (last_detected_angle - first_detected_angle) if last_detected_angle and first_detected_angle else None
    if actual_rot_angle is not None:
        print(f"Actual rotation angle: {actual_rot_angle} degrees")
    else:
        print("Actual rotation angle not determined.")

if __name__ == "__main__":
    main()