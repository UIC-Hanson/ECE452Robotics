from picarx import Picarx
import time
import cv2
import numpy as np
import yaml
import utils
import math

#========TO DO: DECLARE THE SERVO ========
px = Picarx()

#========TO DO========
# define the initial and the next angle
# amount of rotation (theta) = next angle - initial angle
init_angle = 0
next_angle = 20
#================================================


# define the initial and the desired angle of rotation
# amount of rotation (theta) = goal angle - initial angle
init_angle = 55
desired_th = 25

# The different ArUco dictionaries built into the OpenCV library. 
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters()

# Side length of the ArUco marker in meters 
marker_length = 0.1
 
# Calibration parameters yaml file
with open(r'calib_data.yaml') as file:
    calib_data = yaml.load(file, Loader=yaml.FullLoader)

mtx = np.asarray(calib_data["camera_matrix"])
dist = np.asarray(calib_data["distortion_coefficients"])

cap = cv2.VideoCapture(cv2.CAP_V4L)

#======== TO DO ========
# move the camera to the initial angle  
try:
    px.set_cam_pan_angle(init_angle)
    time.sleep(0.01)
except Exception as e:
    print("Error setting initial servo angle:", e)
 
time.sleep(3)
#================================================

init_rvec = None
init_tvec = None
g0 = None

print("Start scanning the marker, you may quit the program by pressing q ...")

for current_angle in range(init_angle,181,2):
    #======== TO DO ========
    # move the camera to current_angle
    try:
        px.set_cam_pan_angle(next_angle)
        time.sleep(0.01)
    except Exception as e:
        print("Error setting next servo angle:", e)
    
    #================================================
    
    
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if len(corners)!=0: # if aruco marker detected
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            cv2.aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.05)
            if g0 is None:
                init_rvec = rvec
                init_tvec = tvec
                g0 = utils.cvdata2transmtx(init_rvec,init_tvec)[0] # g(0)
                print("Initial data saved...")
            else:
                gth = utils.cvdata2transmtx(rvec,tvec)[0] # g(th)
                
                
                #======== TO DO ========
                #find exp^(hat(xi)*th) using g(0) and g(th)
                exp_mtx = gth * np.linalg.inv(g0)
                #================================================
                
                
                v,w,th = utils.transmtx2twist(exp_mtx)
                
                # if the estimated rotation angle is closed to the desired rotation angle
                # the program will stop and find the error
                if np.square(desired_th-math.degrees(th)) <= 10:
                    actual_rot_angle = current_angle-init_angle
                    break
    cv2.imshow('aruco',frame)
    cv2.waitKey(3000) # program halts for 3 seconds
                
print("Finished rotation...")
print("Estimated rotation angle: {} degrees".format(math.degrees(th)))
print("Actual rotation angle: {} degrees".format(actual_rot_angle))

# Turn off the camera
cap.release()
cv2.destroyAllWindows()
