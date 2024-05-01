#Bug 1 main
import asyncio
import sys
sys.path.append('/utilities')
from robot_control import RobotControl

robot = RobotControl()

async def InitialScan():
    # Implement scanning logic
    pass

async def DetermineGoal():
    # Logic to determine the goal
    pass

def read_line(white_line=True, threshold2=50): #threshold2 may need to be adjusted
    coef1 = 255 if white_line else 0 #coef1 may need to be adjusted
    numSensors = 3
    sensor_positions = [1, 2, 3]
    sensor_values = px.get_grayscale_data()
    
    avg = 0
    sum_sensors = 0
    on_line = False

    for i in range(numSensors):
        value = abs(coef1 - sensor_values[i])
        if value > threshold2:
            avg += value * sensor_positions[i]
            sum_sensors += value
            on_line = True

    if not on_line:
        if last_value < (max(sensor_positions) + 1) // 2:
            return 0  # Line to the left of center
        else:
            return max(sensor_positions)  # Line to the right of center

    last_value = avg / sum_sensors if sum_sensors != 0 else 0
    return last_value

async def ObstacleTrack():
    # Initial position
    H1 = await GetPosn()
    L1 = H1
    CP = H1

    while CP != H1:
        # Read line position
        line_position = read_line()
        # Follow the line based on 'line_position'
        # Code to move the robot based on line position
        await asyncio.sleep(1)  # Simulate following the line
        CP = await GetPosn()
        if CP < L1:
            L1 = CP

    while CP != L1:
        await asyncio.sleep(1)  # Continue following the line

    await robot.stop()  # Ensure stop is also awaited if it becomes async

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
    
    await robot.initialize_robot()
    await robot.set_power_level()
    await InitialScan()

    while True:
        if await is_goal_reached():
            print("Goal reached! Exiting loop.")
            break  # Break out of the loop once the goal is reached
        
        await DetermineGoal()
        await MoveGoal()
        if robot.current_state != 'forward':
            await ObstacleTrack()
            #at the end of obstacle track the robot should be at L1

if __name__ == '__main__':
    main()
