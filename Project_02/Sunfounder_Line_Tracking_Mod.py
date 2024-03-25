from picarx import Picarx
from time import sleep

px = Picarx()
# px = Picarx(grayscale_pins=['A0', 'A1', 'A2'])

# Please run ./calibration/grayscale_calibration.py to Auto calibrate grayscale values
# or manual modify reference value by follow code
# px.set_line_reference([1400, 1400, 1400])

current_state = None
last_state = "stop"

def initialize_robot():
    # Initialize settings
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

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

def outHandle():
    global last_state, current_state
    if last_state == 'left':
        px.set_dir_servo_angle(-30)
        px.backward(10)
    elif last_state == 'right':
        px.set_dir_servo_angle(30)
        px.backward(10)
    while True:
        gm_val_list = px.get_grayscale_data()
        gm_state = get_status(gm_val_list)
        print("outHandle gm_val_list: %s, %s"%(gm_val_list, gm_state))
        currentSta = gm_state
        if currentSta != last_state:
            break
    sleep(0.001)

def get_status(val_list):
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

if __name__=='__main__':
    initialize_robot()
    px_power = get_power_level()
    
    px_power_for_alpha = px_power/1000
    print(f"Power level set to: {px_power}, Alpha power calc value: {px_power_for_alpha}")
    distance = 0
    alpha = 0.017881989
    wheelsize = 0.0205  # Wheel diameter in meters
    offset = 20  # Steering angle offset for left/right corrections
    #current_state = get_status(val_list) #initial setup
    
    try:
        while True:
            gm_val_list = px.get_grayscale_data()
            current_state = get_status(gm_val_list)
            distance += alpha * px_power_for_alpha  # Increment distance based on power
            print("Distance: " + str(distance))

            if current_state != "stop":
                last_state = current_state

            if current_state == 'forward':
                px.set_dir_servo_angle(0)
                px.forward(px_power)
            elif current_state == 'left':
                px.set_dir_servo_angle(offset)
                px.forward(px_power)
            elif current_state == 'right':
                px.set_dir_servo_angle(-offset)
                px.forward(px_power)
            else:
                outHandle()
    finally:
        # Reset servo and camera angles, stop motors
        px.set_cam_tilt_angle(0)
        px.set_cam_pan_angle(0)  
        px.set_dir_servo_angle(0)  
        px.stop()
        sleep(.2)
        print("Robot stopped.")
