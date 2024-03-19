"""
Used to estimate alpha to determine the incremental travel displacement for the rear tires

Set up a 100 cm black line for the robot to follow
Run 5 iterations at 5 different speeds and record the distance travelled in each iteration
See Project 2 instructions on how to determine alpha

"""

from picarx import Picarx
from time import sleep, time
from datetime import datetime

px = Picarx()

# Initialize servos
px.set_dir_servo_angle(0)
px.set_cam_pan_angle(0)
px.set_cam_tilt_angle(0)

current_state = None
px_power = 10  # Adjust as needed for slow initial movement
offset = 30
last_state = "stop"
timer_started = False
tracking_start_time = None

def get_status(val_list):
    state = px.get_line_status(val_list)
    
    if state == [0, 0, 0]:
        return 'stop'
    elif state[1] == 1:
        return 'forward'
    elif state[0] == 1:
        print("Sensor values: ", state)
        return 'right'
    elif state[2] == 1:
        print("Sensor values: ", state)
        return 'left'
    else:
        print("Charlie is in the bad place, State was: ", state)
        return 'unknown'

if __name__ == '__main__':
    try:
        px.forward(px_power)  # Start moving forward slowly
        while True:
            gm_val_list = px.get_grayscale_data()
            gm_state = get_status(gm_val_list)
            #Robot has been stopping before the end of the track.
            current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # Get current time with milliseconds
            print(f"{current_time} - Grayscale sensor values:", gm_val_list)

            # Check if the line is detected for the first time
            if gm_state in ['forward', 'left', 'right'] and not timer_started:
                print("Line detected. Starting timer.")
                tracking_start_time = time()
                timer_started = True  # Prevent restarting the timer

            # If the end of the line is reached or the timer has not started yet
            if gm_state == "stop" and timer_started:
                print("End of line reached or lost line. Stopping robot.")
                break

            # Adjust direction based on the line position
            if gm_state == 'forward':
                px.set_dir_servo_angle(0)
            elif gm_state == 'left':
                print(gm_state)
                px.set_dir_servo_angle(offset)
            elif gm_state == 'right':
                print(gm_state)
                px.set_dir_servo_angle(-offset)
    finally:
        px.stop()
        if timer_started:
            duration = time() - tracking_start_time
            print(f"Time taken to follow the line: {duration:.2f} seconds")
        else:
            print("No line was detected.")
        sleep(0.1)
