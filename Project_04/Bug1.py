# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 20:19:34 2023

@author: Leonardo
"""

import RPi.GPIO as GPIO
import cv2
import numpy as np
import yaml
import utils
import math
import time
import pickle
#from TRSensor import TRSensor
from picarx import Picarx

px = Picarx()

#CHANGE IT FOR AN OPTIMAL RESULT
px.set_grayscale_reference([0,0,0]) 

class point(object):
    def __init__(self):
        point.x = None
        point.z = None

def stop(px):
    px.set_motor_speed(1,0)
    px.set_motor_speed(2,0)

#======= TO DO =======: define 5 methods as for the precedent project
# turn, move left, move right, move forward, move backward

def turn_left(px):
    px.set_dir_servo_angle(-90)
    px.forward(10)
    time.sleep(0.1)
    px.set_dir_servo_angle(0)
    px.forward(0)

def turn_right(px):
    px.set_dir_servo_angle(90)
    px.forward(10)
    time.sleep(0.1)
    px.set_dir_servo_angle(0)
    px.forward(0)

def move_forward(px):
    px.set_dir_servo_angle(0)
    px.forward(10);
    time.sleep(0.1);
    px.forward(0);

def move_backward(px):
    px.set_dir_servo_angle(0)
    px.backward(10);
    time.sleep(0.1);
    px.backward(0);


'''
Returns an estimated position of the robot with respect to a line
The estimate is made using a weighted average of the sensor indices
tiplied by 1000, so that a return value of 0 indicates that
 line is directly below sensor 0, a return value of 1000
icates that the line is directly below sensor 1, 2000
icates that it's below sensor 2000, etc.  Intermediate
ues indicate that the line is between two sensors.  The
mula is:

0*value0 + 1000*value1 + 2000*value2 + ...
--------------------------------------------
value0  +  value1  +  value2 + ...

default, this function assumes a dark line (high values)
rounded by white (low values).  If your line is light on
ck, set the optional second argument white_line to true.  In
s case, each sensor value will be replaced by (1000-value)
ore the averaging.
'''
def readLine(white_line = 0):

    sensor_values = px.get_grayscale_data()
    avg = 0
    sum = 0
    on_line = 0
    last_value=0
    numSensors=3

    #======= TO DO ======= 
    #depending of your calibration you may want to modify these values
    coef1=1000
    treshold1=200
    treshold2=50


    for i in range(0,numSensors):

        value = sensor_values[i]

        if(white_line):
            value = coef1-value
        # keep track of whether we see the line at all
        if(value > treshold1):
            on_line = 1

        # only average in values that are above a noise threshold
        if(value > treshold2):
            avg += value * (i * coef1);  # this is for the weighted total,
            sum += value;                  #this is for the denominator

    if(on_line != 1):
        # If it last read to the left of center, return 0.
        if(last_value < (numSensors - 1)*coef1/2):
            #print("left")
            return 0;

        # If it last read to the right of center, return the max.
    else:
        #print("right")
            return (numSensors - 1)*coef1

    last_value = avg/sum
    return last_value





def go_to_goal(px,cap,goal_id,goal,hit,deg_eps,dist_eps,last_proportional,angle_to_goal):
    print("Warming up the Line track sensors...")
    time.sleep(0.5)

    #======= TO DO ======= 
    #depending of your calibration you may want to modify these values (you would also have to modify it in: find_leave
    #and go_to_leave methods)

    coef=2000
    treshold=500


    for i in range(0,100):
        position = readLine()
        proportional = position - coef
        last_proportional = proportional
    time.sleep(0.5)
    print("Fininished warmup")
    while cap.isOpened():
        ret, frame = cap.read()
        position = readLine()
        proportional = position - coef
        derivative = proportional - last_proportional
        last_proportional = proportional
        # When the IR sensor detects the line...
        if abs(derivative) >= treshold:
            state = 1
            print("Hit line...")
            hit.x = p_gc[0]
            hit.z = p_gc[2]

            #======= TO DO =======: the robots needs to turn, therefore call here the function implemented before

            # Assume that we always go left
            turn_left(px)


            return state, goal, hit, last_proportional, angle_to_goal



        elif ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            if (len(corners)!=0):
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
                for i,_ in enumerate(rvecs):
                    if ids[i] == goal_id:
                        cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                        g_gc = utils.cvdata2transmtx(rvecs[i],tvecs[i])[0]
                        p_gc = g_gc[:,3]
                        th = utils.transmtx2twist(g_gc)[2]
                        if angle_to_goal is None:
                            goal.z = 0.1
                            # This is the angle between the robot's initial position to the Goal marker frame's origin
                            # The goal point will be set along this line near to the Goal marker
                            angle_to_goal = math.atan(p_gc[0]/p_gc[2])
                            goal.x = goal.z*math.tan(angle_to_goal)
                            print("Set Goal Point x:{} z:{}".format(goal.x,goal.z))
                        xdiff = p_gc[0]-goal.x
                        zdiff = p_gc[2]-goal.z
                        cur_dist = utils.distance(xdiff,zdiff)
                        if cur_dist <= dist_eps:
                            print("Reached goal point!")
                            cv2.destroyAllWindows()
                            stop(px)
                            state = 3
                            return state, goal, hit, last_proportional, angle_to_goal
                        # ======= TO DO =======:: fill out the which movements (left, right, forward, etc) should be running
                        #       in each case
                        if zdiff < 0:
                            move_backwards(px) # What???
                        else:
                            move_forwards(px)
                    else:
                        turn_left(px) # idk what to do here
            else:
                if 'th' in locals(): # 
                    if th - angle_to_goal > 0:
                        turn_left(px) # Turn if not aligned with goal
                                      # May need to be swapped
                    else:
                        turn_right(px)
                else:
                    turn_left(px) # Look around for the goal      

            cv2.imshow('aruco',frame)
            key = cv2.waitKey(100) & 0xFF
            # Press q to stop in the middle of the process
            if key == ord('q'):
                cv2.destroyAllWindows()
                stop(px)
                state = 3
                return state, goal, hit, last_proportional, angle_to_goal

def find_leave(px,cap,goal_id,helper1_id,helper2_id,goal,hit,leave,dist_eps,g_gh1,g_gh2,last_proportional):
    maximum = 35
    count = 0
    while cap.isOpened():

        #======= TO DO =======: call here the function to move the robot backward
        #
        #======================
        move_backward(px)

        #======= TO DO ======= 
        #depending of your calibration you may want to modify these values (you would also have to modify it in, find_leave
        #and go_to_leave methods)

        coef=2000


        position = readLine()
        # The "proportional" term should be 0 when we are on the line.
        proportional = position - 2000

        # Compute the derivative (change) and integral (sum) of the position.
        derivative = proportional - last_proportional

        # Remember the last position.
        last_proportional = proportional

        #======= TO DO =======
        power_difference = proportional/25 + derivative/100 #+integral/500
        if (power_difference > maximum):
            power_difference = maximum
        if (power_difference < - maximum):
            power_difference = - maximum
        if (power_difference < 0):

            #!!!!!!!!!!!!!!!!!CARE!!!!!!!!!!!!!!!!!! 
            #DEPENDING OF YOUR CALIBRATION YOU MAY HAVE TO INVERSE 1 AND 2 HERE BELOW
            #Also you may want to add a steering angle deping on how the robot is able to slip
            px.set_motor_speed(1,maximum + power_difference)
            px.set_motor_speed(2,maximum)

        else:

            #!!!!!!!!!!!!!!!!!CARE!!!!!!!!!!!!!!!!!! 
            #DEPENDING OF YOUR CALIBRATION YOU MAY HAVE TO INVERSE 1 AND 2 HERE BELOW
            #Also you may want to add a steering angle deping on how the robot is able to slip
            px.set_motor_speed(1,maximum)
            px.set_motor_speed(2,maximum - power_difference)


        time.sleep(0.5)
        stop(px)
        ret, frame = cap.read()
        # Current distance between goal and leave point
        gl_dist = utils.distance(goal.x-leave.x,goal.z-leave.z)
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            if (len(corners)!=0):
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
                for i,_ in enumerate(rvecs):
                    if ids[i] == goal_id:
                        cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                        g_gc = utils.cvdata2transmtx(rvecs[i],tvecs[i])[0]
                        p_gc = g_gc[:,3]
                        cur_dist_goal = utils.distance(p_gc[0]-goal.x,p_gc[2]-goal.z)
                        cur_dist_hit = utils.distance(p_gc[0]-hit.x,p_gc[2]-hit.z)
                        if cur_dist_goal <= gl_dist:
                            gl_dist = cur_dist_goal
                            leave.x = p_gc[0]
                            leave.z = p_gc[2]
                            count += 1
                        if cur_dist_hit <= dist_eps and count > 100:


                            stop(px)
                            print("Finished the track, now going to the leave point")
                            state = 2
                            return state, leave, last_proportional



                    # ======= TO DO =======:: Please fill out when other markers are detected.
                    #       The code is very similar to above where the Goal marker is detected.
                    elif ids[i] == helper1_id:
                        pass # what is this for
                    elif ids[i] == helper2_id:
                        pass

            cv2.imshow('aruco',frame)
            key = cv2.waitKey(100) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()


                stop(px)
                state = 3
                return state, leave, last_proportional







def go_to_leave(px,cap,goal_id,helper1_id,helper2_id,goal,leave,dist_eps,g_gh1,g_gh2,last_proportional):
    maximum = 35
    while cap.isOpened():


        #======= TO DO =======: call here below your backward method
        move_backward(px)
        #======================


        #======= TO DO ======= 
        #depending of your calibration you may want to modify these values (you would also have to modify it in, find_leave
        #and go_to_leave methods)
        coef=2000



        position = TR.readLine()

        # The "proportional" term should be 0 when we are on the line.
        proportional = position - coef

        # Compute the derivative (change) and integral (sum) of the position.
        derivative = proportional - last_proportional

        # Remember the last position.
        last_proportional = proportional
        power_difference = proportional/25 + derivative/100 #+integral/500


        if (power_difference > maximum):
            power_difference = maximum
        if (power_difference < - maximum):
            power_difference = - maximum
        if (power_difference < 0):

            #!!!!!!!!!!!!!!!!!CARE!!!!!!!!!!!!!!!!!! 
            #DEPENDING OF YOUR CALIBRATION YOU MAY HAVE TO INVERSE 1 AND 2 HERE BELOW
            #Also you may want to add a steering angle deping on how the robot is able to slip
            px.set_motor_speed(1,maximum + power_difference)
            px.set_motor_speed(2,maximum)

        else:

            #SAME THING HERE
            px.set_motor_speed(1,maximum)
            px.set_motor_speed(2,maximum - power_difference)

        time.sleep(0.5)
        stop(px)
        ret, frame = cap.read()
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            if (len(corners)!=0):
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
                for i,_ in enumerate(rvecs):
                    if ids[i] == goal_id:
                        cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                        g_gc = utils.cvdata2transmtx(rvecs[i],tvecs[i])[0]
                        p_gc = g_gc[:,3]
                        cur_dist_leave = utils.distance(p_gc[0]-leave.x,p_gc[2]-leave.z)
                        if cur_dist_leave <= dist_eps:
                            print("Reached leave point, going to the goal point again.")
                            for i in range(5):
                                #======= TO DO =======: call your forward function here below
                                move_forward(px)

                            state = 0
                            return state


                    # ======= TO DO =======:: Please fill out when other markers are detected.
                    #       The code is very similar to above where the Goal marker is detected.
                    elif ids[i] == helper1_id:
                        pass # again, idk what to do here
                    elif ids[i] == helper2_id:
                        pass
            cv2.imshow('aruco',frame)
            key = cv2.waitKey(100) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                stop(px)
                state = 3
                return state

# =========TO DO =========
#do any further variable declaration here  


last_proportional = 0

#with open('ir_calib.pkl', 'rb') as ir_calib_file:
#    TR = pickle.load(ir_calib_file)
#    stop(px)

# The different ArUco dictionaries built into the OpenCV library. 
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters_create()
aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

# Side length of the ArUco marker in meters 
marker_length = 0.05

# initialize the points
goal = point()
hit = point()
leave = point()

angle_to_goal = None 

# Set leave point (initially it is a very far point)
leave.x = 1000000
leave.z = 1000000

# Calibration parameters yaml file
with open(r'calib_data_bug1.yaml') as file:
    calib_data = yaml.load(file, Loader=yaml.FullLoader)

mtx = np.asarray(calib_data["camera_matrix"])
dist = np.asarray(calib_data["distortion_coefficients"])
g_gh1 = np.asarray(calib_data["g_gh1"])
g_gh2 = np.asarray(calib_data["g_gh2"])

cap = cv2.VideoCapture(cv2.CAP_V4L)

#marker ids
goal_id = 2
helper1_id = 1
helper2_id = 0

#epsilons
deg_eps = 0.1
dist_eps = 0.2  

# flags
state = 0

try:
    while True:
        if state == 0:
            state, goal, hit, last_proportional, angle_to_goal = go_to_goal(px,cap,goal_id,goal,hit,deg_eps,dist_eps,last_proportional,angle_to_goal)
        elif state == 1:
            state, leave, last_proportional = find_leave(px,cap,goal_id,helper1_id,helper2_id,goal,hit,leave,dist_eps,g_gh1,g_gh2,last_proportional)
        elif state == 2:
            state = go_to_leave(px,cap,goal_id,helper1_id,helper2_id,goal,leave,dist_eps,g_gh1,g_gh2,last_proportional)
        elif state == 3:
            print("Goal point x:{}, z:{}".format(goal.x,goal.z))
            print("Hit point x:{}, z:{}".format(hit.x,hit.z))
            print("Leave point x:{}, z:{}".format(leave.x,leave.z))
            print("Finished Bug 1!")
            break

except KeyboardInterrupt:
    cap.release()
    cv2.destroyAllWindows()
    stop(px)
