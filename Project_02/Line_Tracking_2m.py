from picarx import Picarx
from time import sleep, time

px = Picarx()

def initialize_robot():
    # Initialize settings
    #px.set_grayscale_reference([1400, 1400, 1400])
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

def get_status():
    """Determine the robot's state based on grayscale sensor data."""
    val_list=px.get_grayscale_data()
    state = px.get_line_status(val_list)
    
    if state == [0, 0, 0]:
        return 'stop'
    elif state[1] == 1:
        return 'forward'
    elif state[0] == 1:
        return 'right'
    elif state[2] == 1:
        return 'left'
    else:
        print("Charlie is in the bad place, State was: ", state)
        return 'stop'
    return 'stop' 

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
    initialize_robot()
    px_power = get_power_level()
    print(f"Power level set to: {px_power}")
    distance = 0
    alpha = 0.012921758
    wheelsize = 0.0205  # Assuming this is the wheel diameter in meters
    run=True

    try:
        while run==True:
            gm_state = get_status()
            distance = (alpha * px_power/100 + distance)  # Increment distance based on power
            
            if distance / wheelsize >= 2:
                print("We have crossed the desert to the holy land, 2 meters away.")
                run=False
            elif gm_state == 'stop':
                px.forward(0)
                px.stop()
                print("Just doing what I was told.")
            elif gm_state == 'forward':
                px.forward(px_power)
                px.set_dir_servo_angle(0)
            elif gm_state == 'left':
                px.forward(px_power)
                px.set_dir_servo_angle(offset)
            elif gm_state == 'right':
                px.forward(px_power)
                px.set_dir_servo_angle(-offset)
            else:
                print("Frank let's play nightcrawlers")

            sleep(0.01)  # Sleep at the end of the loop to ensure some delay

    finally:
        px.stop()
        print("Robot stopped.")

if __name__ == '__main__':
    main()
