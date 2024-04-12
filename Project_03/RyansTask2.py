import cv2
import numpy as np
import yaml
import time

import utils

class RobotController:
    def __init__(self):
        self.px = Picarx()  # Assuming Picarx is properly defined and imported elsewhere
        self.state_flag = 0
        self.rot_flag = 0
        self.count = 0
        self.goal_x = 0
        self.goal_z = 0

    def move_forward(self, duration, speed=30):
        self.px.forward(speed)
        time.sleep(duration)
        self.px.forward(0)

    def rotate_to_angle(self, start_angle, end_angle, step=1):
        for angle in range(start_angle, end_angle + step, step):
            self.px.set_dir_servo_angle(angle)
            time.sleep(0.01)

    def stop(self):
        self.px.forward(0)

    def detect_aruco_marker(self, frame):
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        aruco_params = cv2.aruco.DetectorParameters_create()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if len(corners) > 0:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.mtx, self.dist)
            g, _, p = utils.cvdata2transmtx(rvec, tvec)
            return corners, ids, rvec, tvec, p
        return None, None, None, None, None

    def load_calibration(self, file_path='calib_data.yaml'):
        with open(file_path) as file:
            calib_data = yaml.load(file, Loader=yaml.FullLoader)
        self.mtx = np.asarray(calib_data["camera_matrix"])
        self.dist = np.asarray(calib_data["distortion_coefficients"])

    def run(self):
        self.load_calibration()
        cap = cv2.VideoCapture(cv2.CAP_V4L)

        try:
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    continue

                corners, ids, rvec, tvec, p = self.detect_aruco_marker(frame)
                if corners is not None:
                    if self.state_flag == 0:
                        self.goal_x, self.goal_z = p[0], p[2]
                        self.move_forward(1)
                        self.state_flag = 1
                        self.rot_flag = 0
                        print(f"Goal point: x:{self.goal_x} z:{self.goal_z}")
                    elif self.state_flag == 1:
                        xdiff = p[0] - self.goal_x
                        zdiff = p[2] - self.goal_z
                        cur_dist = utils.distance(xdiff, zdiff)
                        if cur_dist > 0.05:  # Arbitrary threshold for distance
                            self.rotate_to_angle(-90, 0)
                            self.move_forward(0.5)
                        elif cur_dist < -0.05:
                            self.rotate_to_angle(90, 0)
                            self.move_forward(0.5)
                        else:
                            self.move_forward(1)
                else:
                    self.stop()
        finally:
            cap.release()
            cv2.destroyAllWindows()

# Example usage:
# robot = RobotController()
# robot.run()
