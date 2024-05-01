#Bug 1 main
import time
import sys
sys.path.append('Utilities')
from robot_control import RobotControl

robot = RobotControl()

def InitialScan():
    # Implement scanning logic
    pass

def DetermineGoal():
    # Logic to determine the goal
    pass

# Returns 'left', 'right', or 'forward'
def read_line():
    sensor_values = px.get_grayscale_data()
    threshold = 1000 # Arbitrary, set this later

    detected = []
    for i in range(3):
        detected.append(sensor_values[i] < threshold)
        
    # Is the line detected on both sides?
    if detected[0] and detected[2]:
        return 'forward'

    # Is the line detected by every sensor or no sensor?
    if detected[1] == detected[0] and detected[1] == detected[2]:
        return 'forward'

    if detected[1]:
        return 'forward'

    if detected[2]:
        return 'right'
    if detected[0]:
        return 'left'
    return 'forward' # Edge case, ensure it always returns something

import cv2
import math
import time
import utils  # assuming this contains utility functions like cvdata2transmtx

def go_to_goal(px, cap, goal_id, goal, hit, deg_eps, dist_eps, last_proportional, angle_to_goal):
    print("Warming up the Line track sensors...")
    time.sleep(0.5)

    coef1 = 2000
    threshold1 = 500

    for i in range(0,100):
        position = read_line()
        proportional = position - coef1
        last_proportional = proportional

    time.sleep(0.5)
    print("Finished warmup")

    while cap.isOpened():
        ret, frame = cap.read()
        position = read_line()
        proportional = position - coef1
        derivative = proportional - last_proportional
        last_proportional = proportional

        # When the IR sensor detects a line
        if abs(derivative) >= threshold1:
            state = 1
            print("Hit line...")
            hit.x = p_gc[0]
            hit.z = p_gc[2]
            turn(px)  # Call to function handling turning
            return state, goal, hit, last_proportional, angle_to_goal

        elif ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            if (len(corners)!=0):
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
                for i, _ in enumerate(rvecs):
                    if ids[i] == goal_id:
                        cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                        g_gc = utils.cvdata2transmtx(rvecs[i], tvecs[i])[0]
                        p_gc = g_gc[:, 3]
                        th = utils.transmtx2twist(g_gc)[2]
                        if angle_to_goal is None:
                            goal.z = 0.1
                            # This is the angle between the robot's initial position to the Goal marker frame's origin
                            # The goal point will be set along this line near to the Goal marker
                            angle_to_goal = math.atan2(p_gc[0], p_gc[2])
                            goal.x = goal.z * math.tan(angle_to_goal)
                            print("Set Goal Point x:{} z:{}".format(goal.x, goal.z))
                        xdiff = p_gc[0] - goal.x
                        zdiff = p_gc[2] - goal.z
                        cur_dist = utils.distance(xdiff, zdiff)
                        if cur_dist <= dist_eps:
                            print("Reached goal point!")
                            cv2.destroyAllWindows()
                            stop(px)
                            state = 2
                            return state, goal, hit, last_proportional, angle_to_goal
                        if zdiff < 0:
                            move(px, "forward")
                        else:
                            move(px, "backward")
                    else:
                        # Placeholder for additional logic related to helper markers
                        pass
            else:
                if 'th' in locals(): # 
                    if th - angle_to_goal > 0:
                        
                    #else:    
                        
                #else:

                        cv2.imshow('aruco', frame)
                        key = cv2.waitKey(100) & 0xFF
                        if key == ord('q'):
                            cv2.destroyAllWindows()
                            stop(px)
                            state = 3
                            return state, goal, hit, last_proportional, angle_to_goal

def ObstacleTrack():
    # Initial position
    H1 = GetPosn()
    L1 = H1
    CP = H1

    while CP != H1:
        # Read line position
        line_position = read_line()
        # Follow the line based on 'line_position'
        robot.current_state = line_position
        robot.navigate()
        # Code to move the robot based on line position
        time.sleep(1)
        CP = GetPosn()
        if CP < L1:
            L1 = CP

    while CP != L1:
        time.sleep(1)

    robot.stop()  # Ensure stop is also awaited if it becomes async

def is_goal_reached():
    # Implement the logic to determine if the goal is reached
    # This might involve checking sensors or internal state variables
    return robot.at_goal  # Example: Check an attribute that indicates goal status

def main():

    """Script
    scan for markers, denote locatons
    determine direction of Goal
    drive towards goal, while scanning for lines/obstacles
    
    encounter obstacle
        mark position, save as H1 and L1

        trace the obstacle until CP ~= H1
            move along the line for time
            stop
            Scan with camera and find position
            if positon CP < L1
                L1=CP
            else
                continue
    Determie direction of goal

    It would appera the functions that arise from this are

    def InitialScan():
        creates the world looking for markers

    def DetermineGoal():    
        determine direction of goal should save the camera degrees of the goal, this angle will be saved as the  direcion of travel
    
    def ObstacleTrack():
        H1 = GetPosn()
        L1=H1
        CP=H1

        while CP != H1
            Picar follow line for time
            stop
            CP =GetPosn()
            If CP < L1
                L1=CP
        
        while CP != L1
            Picar follow line

        stop
    """
    
    robot.initialize_robot()
    robot.set_power_level()
    InitialScan()

    while True:
        if is_goal_reached():
            print("Goal reached! Exiting loop.")
            break  # Break out of the loop once the goal is reached
        
        DetermineGoal()
        MoveGoal()
        if robot.current_state != 'forward':
            ObstacleTrack()
            #at the end of obstacle track the robot should be at L1

if __name__ == '__main__':
    main()
