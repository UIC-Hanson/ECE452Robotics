"""
Used to estimate alpha to determine the incremental travel displacement for the rear tires

Set up a 100 cm black line for the robot to follow
Run 5 iterations at 5 different speeds and record the distance travelled in each iteration
See Project 2 instructions on how to determine alpha

"""

from picarx import Picarx
from time import sleep, time

px = Picarx()

# Set servos to the 0 position
px.set_dir_servo_angle(0)  # Set the direction servo to 0
px.set_cam_pan_angle(0)    # camera, set the pan angle to 0
px.set_cam_tilt_angle(0)   # camera, set the tilt angle to 0

current_state = None
px_power = 10
offset = 20
last_state = "stop"
tracking_start_time = None

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
        print("outHandle gm_val_list: %s, %s" % (gm_val_list, gm_state))
        current_state = gm_state
        if current_state != last_state:
            break
    sleep(0.001)

def get_status(val_list):
    state = px.get_line_status(val_list)  # Assumes state returns a list like [0, 0, 0] where 0 means line, 1 means background
    #print("Sensor values: ", state)  # trying to identifty faults, comment out eventually
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

if __name__ == '__main__':
    try:
        tracking_start_time = time()  # Record the start time
        while True:  # Change this to an infinite loop
            gm_val_list = px.get_grayscale_data()
            gm_state = get_status(gm_val_list)
            print("gm_val_list: %s, %s" % (gm_val_list, gm_state))

            if gm_state == "stop":
                print("Stop state reached. Stopping robot.")
                break  # Exit the loop if "stop" state is detected

            if gm_state != "stop":
                last_state = gm_state

                if gm_state == 'forward':
                    px.set_dir_servo_angle(0)
                    px.forward(px_power)
                elif gm_state == 'left':
                    px.set_dir_servo_angle(offset)
                    px.forward(px_power)
                elif gm_state == 'right':
                    px.set_dir_servo_angle(-offset)
                    px.forward(px_power)
                else:
                    outHandle()
            else:
                px.stop()
                break  # Ensure the robot stops if no condition is met

    finally:
        end_time = time()  # Record the end time
        px.stop()
        print("stop and exit")
        sleep(0.1)
        duration = end_time - tracking_start_time
        print(f"Time taken to run the course: {duration:.2f} seconds")
