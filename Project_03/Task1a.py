from picarx import Picarx
import time
import cv2
import numpy as np
import yaml
import utils
import math



#========TO DO: DECLARE THE SERVO ========
px = Picarx()


#==============================================

#========TO DO========
# define the initial and the next angle
# amount of rotation (theta) = next angle - initial angle
init_angle = 20
next_angle = -20
#================================================


# The different ArUco dictionaries built into the OpenCV library. 
# Updated ArUco initialization
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

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

#start detecting the aruco marker
print("Press s to save the initial data or press q to quit...")
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Updated marker detection using ArucoDetector
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

        if len(corners) != 0:  # if aruco marker detected
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            cv2.aruco.drawDetectedMarkers(frame, corners, ids, (0,255,0))
            cv2.aruco.drawAxis(frame, mtx, dist, rvec, tvec, 0.05)
        cv2.imshow("aruco", frame)
        key = cv2.waitKey(2) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            init_rvec = rvec
            init_tvec = tvec
            print("Initial data saved, press m to move the camera or q to quit...")
        elif key == ord('m'):
            
            #======== TO DO ========
            #Change the servo angle (you can use the value next_angle)
            try:
                px.set_cam_pan_angle(next_angle)
                time.sleep(0.01)
            except Exception as e:
                print("Error setting initial servo angle:", e)
            
            #================================================
            time.sleep(3)
            print("Camera position changed, press r to save current data or q to quit...")
        elif key == ord('r'):
            next_rvec = rvec
            next_tvec = tvec
            print("Current data saved, press q to quit and start the calculation...")


# Turn off the camera
cap.release()
cv2.destroyAllWindows()

if init_rvec.all() and next_rvec.all():
    # g(0)
    g0 = utils.cvdata2transmtx(init_rvec,init_tvec)[0]
    # g(th)
    gth = utils.cvdata2transmtx(next_rvec,next_tvec)[0] 
    # ======== TO DO ========
    #find exp^(hat(xi)*th) using g(0) and g(th)
    #exp_mtx = 
    #================================================
    # The twist coordinate and screw motion of the servo
    v,w,th = utils.transmtx2twist(exp_mtx)
    q,h,u,M = utils.twist2screw(v,w,th)
    print("Estimated rotation angle: {} degrees".format(math.degrees(th)))
    print("Twist Coordinates:\n {}".format(np.vstack((v,w))*th))
    print("Screw motion:\n q:{},\n h:{},\n u:{},\n M:{}".format(q,h,u,M))
