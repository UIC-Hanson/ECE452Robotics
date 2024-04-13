from picarx import Picarx
import cv2
import numpy as np
import yaml
import utils
import math
import time

def rotate_left(car, angle, speed=10, turnTime=1):
    """Rotate the robot left by the specified angle."""
    car.set_dir_servo_angle(-angle)  # Assuming negative angle for left turn
    car.forward(speed)
    time.sleep(turnTime)
    car.set_dir_servo_angle(0)
    car.forward(0)

def rotate_right(car, angle, speed=10, turnTime=1):
    """Rotate the robot right by the specified angle."""
    car.set_dir_servo_angle(angle)  # Assuming positive angle for right turn
    car.forward(speed)
    time.sleep(turnTime)
    car.set_dir_servo_angle(0)
    car.forward(0)

def move_forward(car, distance, speed=10):
    """Move the robot forward for a specific distance."""
    duration = distance / speed
    car.forward(speed)
    time.sleep(duration)
    car.forward(0)

def move_backward(car, distance, speed=10):
    """Move the robot backward for a specific distance."""
    duration = distance / speed
    car.backward(speed)
    time.sleep(duration)
    car.forward(0)

def load_calibration_data(file_path):
    """Load camera calibration data from a YAML file."""
    with open(file_path) as file:
        return yaml.load(file, Loader=yaml.FullLoader)

def setup_video_capture():
    """Set up video capture for the robot."""
    return cv2.VideoCapture(cv2.CAP_V4L)

def setup_aruco_detector():
    """Set up the ArUco marker detector."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    aruco_params = cv2.aruco.DetectorParameters_create()
    return aruco_dict, aruco_params

def main():
    px = Picarx()
    calib_data = load_calibration_data('calib_data.yaml')
    mtx = np.asarray(calib_data["camera_matrix"])
    dist = np.asarray(calib_data["distortion_coefficients"])
    cap = setup_video_capture()
    aruco_dict, aruco_params = setup_aruco_detector()

    state_flag = 0
    count = 0
    goal_x, goal_z = 0, 0
    mindist = 0.1  # Minimum distance to goal
    speed = 10  # Speed setting for the robot

    print("Start running task 2...")

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
                cv2.aruco.drawDetectedMarkers(frame, corners, ids, (0, 255, 0))
                if len(corners) != 0:  # If ArUco marker detected
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
                    g, _, p = utils.cvdata2transmtx(rvec, tvec)
                    _, _, th = utils.transmtx2twist(g)
                    cv2.imshow("aruco", frame)
                    cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, 0.05)

                    if state_flag == 0:
                        goal_x = p[0]
                        goal_z = p[2]
                        state_flag = 1
                        rot_flag = 0
                        print("Goal point: x:{} z:{}".format(goal_x, goal_z))

                    elif state_flag == 1:
                        xdiff = p[0] - goal_x
                        zdiff = p[2] - goal_z
                        cur_dist = utils.distance(xdiff, zdiff)
                        if cur_dist > mindist:
                            move_forward(px, cur_dist, speed)
                        else:
                            rotate_left(px, 90, speed)
                            state_flag = 0
                            count += 1
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    finally:
        px.forward(0)  # Ensure the robot stops
        cap.release()
        cv2.destroyAllWindows()
        print("Navigation task completed.")

if __name__ == '__main__':
    main()

