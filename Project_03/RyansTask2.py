
import cv2
import numpy as np
import yaml
import time
from picarx import Picarx  # This should be available in the PiCar-X package

# Utils module containing necessary transformations and distance calculations
import utils

class RobotController:
    def __init__(self):
        self.px = Picarx()
        self.marker_counter = 0  # To keep track of the markers detected

    def move_forward(self, duration, speed=30):
        self.px.forward(speed)
        time.sleep(duration)
        self.px.forward(0)

    def rotate_90_degrees(self):
        # Rotate 90 degrees; this is just an example and will likely need calibration
        self.px.set_dir_servo_angle(90)
        time.sleep(1)  # Assuming it takes 1 second to rotate; this will need to be tested and calibrated
        self.px.set_dir_servo_angle(0)  # Reset the steering angle

    def detect_aruco_marker(self, frame):
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        aruco_params = cv2.aruco.DetectorParameters_create()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if len(corners) > 0:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.mtx, self.dist)
            g, _, p = utils.cvdata2transmtx(rvec, tvec)
            return p
        return None

    def load_calibration(self, file_path='calib_data.yaml'):
        with open(file_path) as file:
            calib_data = yaml.load(file, Loader=yaml.FullLoader)
        self.mtx = np.asarray(calib_data["camera_matrix"])
        self.dist = np.asarray(calib_data["distortion_coefficients"])

    def run(self):
        self.load_calibration()
        cap = cv2.VideoCapture(cv2.CAP_V4L)
        goal_reached = False

        try:
            while cap.isOpened() and self.marker_counter < 4:
                ret, frame = cap.read()
                if not ret:
                    continue

                p = self.detect_aruco_marker(frame)
                if p is not None and not goal_reached:
                    # Assuming goal position is the position of the marker detected
                    self.move_forward(1)  # Move towards the marker
                    goal_reached = True
                elif goal_reached:
                    self.rotate_90_degrees()
                    self.marker_counter += 1
                    goal_reached = False
        finally:
            cap.release()
            cv2.destroyAllWindows()

# Example usage:
# robot = RobotController()
# robot.run()
