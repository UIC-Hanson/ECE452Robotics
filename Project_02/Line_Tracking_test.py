from picarx import Picarx
from time import sleep

# Global Variables
px = Picarx()
current_state = None
px_power = 10  # Default power level, can be adjusted via get_power_level()
offset = 10 #default turning max is 30
last_state = "stop"
distance = 0
alpha = 0.032048612 #alpha for pwr level 10
wheelsize = 0.0205  # wheel diameter in meters

def initialize_robot():
    """Initialize settings for the robot."""
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

def get_power_level():
    """Prompts the user for a power level between 1 and 100, used in 2mdrive.py."""
    while True:
        try:
            level = int(input("Enter power level (1-100): "))
            if 1 <= level <= 100:
                global px_power
                px_power = level
                print(f"Power level set to: {level}")
                return
            else:
                print("Please enter a value between 1 and 100.")
        except ValueError:
            print("Invalid input. Please enter a numerical value between 1 and 100.")

def turn():
    """Turn the robot based on the current state."""
    inboard_wheel = px_power * 0.95
    outboard_wheel = px_power

    if current_state == 'left':
        #px.set_dir_servo_angle(-offset)
        px.set_motor_speed(1, inboard_wheel)
        px.set_motor_speed(2, outboard_wheel)
        print("Turning left. Outboard wheel, inboard wheel: %s, %s" % (outboard_wheel, inboard_wheel))
    elif current_state == 'right':
        #px.set_dir_servo_angle(offset)
        px.set_motor_speed(1, outboard_wheel)
        px.set_motor_speed(2, inboard_wheel)
        print("Turning right. Outboard wheel, inboard wheel: %s, %s" % (outboard_wheel, inboard_wheel))

def get_status(val_list):
    """Get the current status of the robot based on grayscale sensor input."""
    _state = px.get_line_status(val_list)  # [bool, bool, bool], 0 means line, 1 means background
    if _state == [0, 0, 0]:
        return 'stop'
    elif _state[1] == 1:
        return 'forward'
    elif _state[0] == 1:
        return 'right'
    elif _state[2] == 1:
        return 'left'

if __name__ == '__main__':
    initialize_robot()
    #get_power_level()  # Get the power level from user input
    px_power_modifier=18000

    try:
        while True:
            gm_val_list = px.get_grayscale_data()
            current_state = get_status(gm_val_list)
            print("Grayscale values, current state: %s, %s" % (gm_val_list, current_state))

             # Increment distance based on power
            distance += alpha * (px_power / px_power_modifier)
            if distance / wheelsize >= 2:
                print("Travelled 2 meters, stopping.")
                break

            if current_state != "stop":
                last_state = current_state

            if current_state == 'forward':
                px.set_dir_servo_angle(0)
                px.forward(px_power)
               
                sleep(0.0001)
            elif current_state in ['left', 'right']:
                turn()
                sleep(0.0001)
            else:
                sleep(0.0001)
    finally:
        px.stop()
        print("Robot stopped.")
