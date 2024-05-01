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
