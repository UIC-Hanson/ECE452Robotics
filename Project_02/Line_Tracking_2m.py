from picarx import Picarx
from time import sleep, time

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
    
    if state[1] == 1:
        current_state = 'forward'
    elif state == [0, 0, 0]:
        current_state = 'outstate'
    elif state[0] == 1:
        current_state = 'right'
    elif state[2] == 1:
        current_state = 'left'
    else:
        print("Charlie is in the bad place, State was: ", state)
        current_state = 'stop'
    return current_state

def outHandle(offset):
    global current_state
    last_state = current_state
    px.forward(0)
    if last_state == 'left':
        px.set_dir_servo_angle(-offset)
    elif last_state == 'right':
        px.set_dir_servo_angle(offset)
    px.forward(-5)
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
    global current_state
    initialize_robot()
    px_power = get_power_level()

    px_power_for_alpha =px_power/1000
    print(f"Power level set to: {px_power}, Alpha power calc value: {px_power_for_alpha}")
    distance = 0
    alpha = 0.017881989
    wheelsize = 0.0205  # Assuming this is the wheel diameter in meters
    offset = 30  # Steering angle offset for left/right corrections
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
                px.forward(px_power)
                px.set_dir_servo_angle(offset)
            elif current_state == 'right':
                px.forward(px_power)
                px.set_dir_servo_angle(-offset)
            else:
                print("Frank let's play nightcrawlers: ", current_state)

            sleep(0.01)  # Sleep at the end of the loop to ensure some delay

    finally:
        px.set_cam_tilt_angle(0)
        px.set_cam_pan_angle(0)  
        px.set_dir_servo_angle(0)  
        px.stop()
        sleep(.2)
        print("Robot stopped.")

if __name__ == '__main__':
    current_state = None
    main()
