#Bug 1 main
import time
import sys
sys.path.append('/utilities')
from robot_control import RobotControl

robot = RobotControl()

def InitialScan():
    # Implement scanning logic
    pass

def DetermineGoal():
    # Logic to determine the goal
    pass

def read_line(white_line=False):
    sensor_values = px.get_grayscale_data()
    numSensors = 3
    coef1 = 1000
    threshold1 = 200 # Helps in deciding if the line is detected at all
    threshold2 = 50 # Filters out noise from the sensor readings

    avg = 0
    sum_sensors = 0
    on_line = False
    global last_value

    for i in range(numSensors):
        value = sensor_values[i]

        if white_line:
            value = coef1 - value
        
        if value > threshold1:
            on_line = True
        
        if value > threshold2:
            avg += value * (i + 1) * coef1  # Assuming the positions are 1-indexed for sensors
            sum_sensors += value

    if not on_line:
        # Check last read position relative to the center
        if last_value < (numSensors * coef1) / 2:
            return 0  # Line to the left of center
        else:
            return (numSensors - 1) * coef1  # Line to the right of center

    if sum_sensors != 0:
        last_value = avg / sum_sensors
    return last_value

import cv2
import math
import time
import utils  # assuming this contains utility functions like cvdata2transmtx

def go_to_goal(px, cap, goal_id, goal, hit, deg_eps, dist_eps, last_proportional, angle_to_goal):
    print("Warming up the Line track sensors...")
    time.sleep(0.5)

    coef1 = 2000
    threshold1 = 500

    for i in range(100):
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
            if corners:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
                for i, _ in enumerate(rvecs):
                    if ids[i] == goal_id:
                        cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                        g_gc = utils.cvdata2transmtx(rvecs[i], tvecs[i])[0]
                        p_gc = g_gc[:, 3]
                        th = utils.transmtx2twist(g_gc)[2]
                        if angle_to_goal is None:
                            goal.z = 0.1
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
                # Handle scenarios when no markers are detected
                move(px, "search")  # Assume a 'search' function to scan for markers

            cv2.imshow('aruco', frame)
            key = cv2.waitKey(100) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                stop(px)
                state = 2
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
        # Code to move the robot based on line position
        time.sleep(1)
        CP = GetPosn()
        if CP < L1:
            L1 = CP

    while CP != L1:
        time.sleep(1)

    robot.stop()  # Ensure stop is also awaited if it becomes async

async def is_goal_reached():
    # Implement the logic to determine if the goal is reached
    # This might involve checking sensors or internal state variables
    return robot.at_goal  # Example: Check an attribute that indicates goal status

async def main():

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
