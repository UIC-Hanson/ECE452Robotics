import time
import cv2
import numpy as np
import yaml
import utils
import math
from picarx import Picarx
from time import sleep

# Constants
INIT_ANGLE = -10
DESIRED_THETA = 10
MARKER_LENGTH = 0.1
CALIB_DATA_FILE = 'calib_data.yaml'

# Initialize Picarx
px = Picarx()

class Robot:
    def __init__(self):
        self.px = Picarx()
        self.initialize()

    def initialize(self):
        self.px.set_dir_servo_angle(0)
        self.px.set_cam_pan_angle(0)
        self.px.set_cam_tilt_angle(0)
        sleep(0.5)

    def move_camera_to_angle(self, angle):
        try:
            self.px.set_cam_pan_angle(angle)
            time.sleep(0.01)
        except Exception as e:
            print(f"Error setting camera angle to {angle}: {e}")

class CalibrationDataManager:
    def __init__(self, file_path):
        self.calib_data = self.load_calibration_data(file_path)

    def load_calibration_data(self, file_path):
        with open(file_path) as file:
            return yaml.load(file, Loader=yaml.FullLoader)

    @property
    def camera_matrix(self):
        return np.asarray(self.calib_data["camera_matrix"])

    @property
    def distortion_coefficients(self):
        return np.asarray(self.calib_data["distortion_coefficients"])

class ArUcoDetector:
    def __init__(self, camera_matrix, distortion_coefficients):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.camera_matrix = camera_matrix
        self.distortion_coefficients = distortion_coefficients

    def detect_markers(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

    def draw_markers(self, frame, corners, ids):
        cv2.aruco.drawDetectedMarkers(frame, corners, ids, (0, 255, 0))
    
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

    def calculate_transformation_matrix(self, corners, markerCorners3D):
        # Assuming corners is a list of corner points and markerCorners3D are the 3D points
        # You need to ensure corners are reshaped if necessary, similar to your existing code
        markerCorners2D = np.array(corners).reshape(-1, 2)
        success, rvec, tvec = cv2.solvePnP(markerCorners3D, markerCorners2D, self.camera_matrix, self.distortion_coefficients)
        # Assuming utils.cvdata2transmtx exists and converts rvec, tvec to a transformation matrix
        return utils.cvdata2transmtx(rvec, tvec)[0]

class MarkerAlignmentApp:
    def __init__(self):
        self.robot = Robot()
        calib_manager = CalibrationDataManager('calib_data.yaml')
        self.detector = ArUcoDetector(calib_manager.camera_matrix, calib_manager.distortion_coefficients)
        self.cap = cv2.VideoCapture(cv2.CAP_V4L)
        if not self.cap.isOpened():
            raise IOError("Cannot open video capture")
        self.marker_length = MARKER_LENGTH
        self.init_angle = INIT_ANGLE
        self.desired_theta = DESIRED_THETA

    def run(self):
        self.robot.move_camera_to_angle(self.init_angle)
        time.sleep(1)
        g0 = None
        actual_rot_angle = 0
        print("Start scanning the marker, you may quit the program by pressing q ...")

        markerCorners3D = np.array([
            [-self.marker_length / 2, self.marker_length / 2, 0],
            [self.marker_length / 2, self.marker_length / 2, 0],
            [self.marker_length / 2, -self.marker_length / 2, 0],
            [-self.marker_length / 2, -self.marker_length / 2, 0]])

        for current_angle in range(self.init_angle, 181, 2):
            self.robot.move_camera_to_angle(current_angle)
            ret, frame = self.cap.read()
            if ret:
                corners, ids, _ = self.detector.detect_markers(frame)
                if len(corners) > 0:
                    gth = self.detector.calculate_transformation_matrix(corners[0], markerCorners3D)
                    if g0 is None:
                        g0 = gth
                        print("Initial data saved...")
                    else:
                        exp_mtx = gth @ np.linalg.inv(g0)
                # Display and quit mechanism
                cv2.imshow('aruco', frame)
                if cv2.waitKey(2) & 0xFF == ord('q'):
                    break

        print("Finished rotation...")

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


        self.cap.release()
        cv2.destroyAllWindows()        

if __name__ == "__main__":
    app = MarkerAlignmentApp()
    app.run()