from picarx import Picarx
from time import sleep

px = Picarx()

current_state = None

def initialize_robot():
    # Initialize settings
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

def get_status():
    """Determine the robot's state based on grayscale sensor data."""
    global current_state
    val_list = px.get_grayscale_data()
    state = px.get_line_status(val_list)
    
    if state == [0, 0, 0]:
        current_state = 'outstate'
    elif state[0] == 1:
        current_state = 'right'
    elif state[2] == 1:
        current_state = 'left'
    elif state[1] == 1:
        current_state = 'forward'
    
    else:
        # Handle unexpected state
        print("Charlie is in the bad place, State was: ", state)
        current_state = 'stop'
        return current_state
    # Print sensor data and current state
    print("val_list, state, current_state: %s, %s, %s" % (val_list, state, current_state))
    return current_state

def turn(px_power, offset):
    global current_state
    inboard_wheel=0
    px_power=px_power

    if current_state == 'left':
        px.set_dir_servo_angle(offset)
        px.set_motor_speed(1,inboard_wheel)
        px.set_motor_speed(2,px_power)
        sleep(.2)

    elif current_state == 'right':
        px.set_dir_servo_angle(-offset)
        px.set_motor_speed(2,inboard_wheel)
        px.set_motor_speed(1,px_power)
        sleep(.2)

def outHandle(offset):
    """Handle when robot is out of state"""
    global current_state
    last_state = current_state
    px.forward(0)
    if last_state == 'left':
        px.set_dir_servo_angle(-offset)
    elif last_state == 'right':
        px.set_dir_servo_angle(offset)
    px.forward(-5)
    sleep(.1)
    # Wait until the state changes
    while current_state == last_state:
        sleep(0.001)  # Short delay before checking again
        current_state = get_status()

def get_power_level():
    """Prompts the user for a power level between 1 and 100."""
    while True:
        try:
            px_power = int(input("Enter power level (1-100): "))
            if 1 <= px_power <= 100:
                return px_power
            else:
                print("Please enter a value between 1 and 100.")
        except ValueError:
            print("Invalid input. Please enter a numerical value between 1 and 100.")

def main():
    """Main function to control the robot."""
    global current_state
    px_power = get_power_level()

    px_power_for_alpha =px_power/1000
    print(f"Power level set to: {px_power}, Alpha power calc value: {px_power_for_alpha}")
    distance = 0
    alpha = 0.017881989
    wheelsize = 0.064  # Assuming this is the wheel diameter in meters
    offset = 20  # Steering angle offset for left/right corrections
    run=True
    current_state = get_status() #initial setup

    try:
        while run==True:
            current_state = get_status()
            distance += alpha * px_power_for_alpha  # Increment distance based on power
            
            if distance / wheelsize >= 2:
                print("We have crossed the desert to the holy land, 2 meters away.")
                run=False

            if current_state == 'forward':
                px.forward(px_power)
                px.set_dir_servo_angle(0)
            elif current_state == 'outstate':
                outHandle(offset)
            elif current_state == 'stop':
                px.forward(0)
                px.stop()
                print("Just doing what I was told.")
                break
            elif current_state == 'left':
                turn(px_power, offset)
            elif current_state == 'right':
                turn(px_power, offset)
                
            else:
                print("Frank let's play nightcrawlers: ", current_state)

            sleep(0.01)  # ensure small delay

    finally:
        # Reset servo and camera angles, stop motors
        px.set_cam_tilt_angle(0)
        px.set_cam_pan_angle(0)  
        px.set_dir_servo_angle(0)  
        px.stop()
        sleep(.2)
        print("Robot stopped.")

if __name__ == '__main__':
    current_state = None
    initialize_robot()
    main()
