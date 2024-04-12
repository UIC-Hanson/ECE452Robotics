import pigpio
import time
import cv2
import numpy as np
import yaml
import utils
import math

#======== TO DO ========
# define a list of functions that allows the robot to 
#turn 90 degree
#going forward/backward (or you can use the functions implemented in the examples)
x = Picarx()

# Move forward for 1 seconds
px.forward(1)
time.sleep(1)

# Rotate from 0 to 90 degrees
for angle in range(0, 91):
    px.set_dir_servo_angle(angle)
    time.sleep(0.01)

# Rotate from 90 to -90 degrees
for angle in range(90, -91, -1):
    px.set_dir_servo_angle(angle)
    time.sleep(0.01)

# Rotate from -90 to 0 degrees
for angle in range(-90, 1):
    px.set_dir_servo_angle(angle)
    time.sleep(0.01)

# Stop moving
px.forward(0)
time.sleep(1)
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

while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if len(corners)!=0: # if aruco marker detected
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            g,_,p = utils.cvdata2transmtx(rvec,tvec)
            _,_,th = utils.transmtx2twist(g)
            # cv2.aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.05)
            if state_flag == 0 and count == ids:
                #======== TO DO ========
                #State 0. How should you set the goal points based on the moment when the robot 
                #detects a marker?
                px.forward(30)
                time.sleep(1)
                
                state_flag = 1
                rot_flag = 0
                print("Goal point: x:{} z:{}".format(goal_x,goal_z))
                #=======================
            elif state_flag == 1 and count == ids:
                xdiff = p[0]-goal_x
                zdiff = p[2]-goal_z
                cur_dist = utils.distance(xdiff,zdiff)
                #======== TO DO ========

# State 1. How should the robot move based on the information above?
if cur_dist > 0:
    px.set_dir_servo_angle(-90)
    time.sleep(1)
    px.forward(1)
    time.sleep(1)
    state_flag = 0
    rot_flag = 1
    count += 1
elif cur_dist < 0:
    px.set_dir_servo_angle(90)
    time.sleep(1)
    px.forward(1)
    time.sleep(0.5)
else:
    px.forward(1)
    time.sleep(1)
    
# =======================
else:
    px.forward(0)
    time.sleep(1)
    
    # ======== TO DO ========
    # How should the robot move when it misses the marker?
    if rot_flag == 1:
        left(Ab) # or it could be right, depends on how you set turn90()
    # =======================
    
time.sleep(1)
# cv2.imshow('aruco',frame)
# key = cv2.waitKey(1500) & 0xFF
# if key == ord('q'):
#     break
            
# Turn off the camera

cap.release()
cv2.destroyAllWindows()
