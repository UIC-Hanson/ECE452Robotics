from picarx import Picarx
from time import sleep, time

px = Picarx()

last_state = None

def initialize_robot():
    # Initialize settings
    #px.set_grayscale_reference([1400, 1400, 1400])
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

def handle_out():
    if last_state == 'left' or last_state == 'right':
        px.set_dir_servo_angle(-12 if last_state == 'left' else 12)
        px.backward(5)
        while px.get_line_status(px.get_grayscale_data()) == last_state:
            sleep(0.001)

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
    offset = 30
    wheelsize=0.0205

    px.forward(px_power)  # Start moving forward
    while True:
        gm_state = get_status()
        
        if gm_state == "stop"
            px.stop()
                print("Just doing what I was told")
        if gm_state == 'forward':
            px.set_dir_servo_angle(0)
            sleep(0.01)
        elif gm_state == 'left':
            px.set_dir_servo_angle(offset)
            sleep(0.01)
        elif gm_state == 'right':
            px.set_dir_servo_angle(-offset)
            sleep(0.01)
    finally:
        px.set_dir_servo_angle(0)
        px.stop()
        px.stop() 

if __name__ == '__main__':
    main()