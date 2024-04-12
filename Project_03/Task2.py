from picarx import Picarx
import time
import cv2
import numpy as np
import yaml
import utils
import math
import time




#======== TO DO ========
# define a list of functions that allows the robot to 
#turn 90 degree
#going forward/backward (or you can use the functions implemented in the examples)

px = Picarx()

#=======================



# The different ArUco dictionaries built into the OpenCV library. 
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters_create()

# Side length of the ArUco marker in meters 
marker_length = 0.05
 
# Calibration parameters yaml file
with open(r'calib_data.yaml') as file:
    calib_data = yaml.load(file, Loader=yaml.FullLoader)

mtx = np.asarray(calib_data["camera_matrix"])
dist = np.asarray(calib_data["distortion_coefficients"])

cap = cv2.VideoCapture(cv2.CAP_V4L)

print("Start running task 2...")

# The width and height of the rectangle track
width = 0.8
height = 0.3

state_flag = 0 # flag to change between the two states
rot_flag = 0 # flag to check if the robot has rotated 90 degrees recently
count = 0 # flag to check whether it is operating with the current id of the ArUco marker

lw_flag = 0 # Indicates if the robot should move along the width or length of
            # the rectangle. Default (0) is width

goal_z = 0 # We assume that the origin is the initial position of the robot
goal_x = 0

while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if len(corners)!=0: # if aruco marker detected
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            g,_,p = utils.cvdata2transmtx(rvec,tvec)
            _,_,th = utils.transmtx2twist(g)
            if state_flag == 0: # Determine goal
                if lw_flag == 0:
                    goal_z += width
                else:
                    goal_z += height
                print("Goal point: x:{} z:{}".format(goal_x,goal_z))
                state_flag = 1
            elif state_flag == 1: # Move towards goal
                xdiff = p[0]-goal_x
                zdiff = p[2]-goal_z
                cur_dist = utils.distance(xdiff,zdiff)
                if cur_dist > 0.1:
                    px.forwards(10)
                else:
                    state_flag = 2
            elif state_flag == 2: # Rotate
                if abs(th) < 0.5 : # ignore this for now bc idk why this is here
                    px.set_dir_servo_angle(-35)
                else:
                    px.set_dir_servo_angle(0)
                    state_flag = 0
            else: # This should never happen
                state_flag = 0
        time.sleep(1)
#         cv2.imshow('aruco',frame)
#         key = cv2.waitKey(1500) & 0xFF
#         if key == ord('q'):
#             break
            
# Turn off the camera

cap.release()
cv2.destroyAllWindows()
