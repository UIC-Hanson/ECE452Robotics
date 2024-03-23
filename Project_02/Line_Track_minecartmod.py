from picarx import Picarx
from time import sleep

px = Picarx()

px_power = .25
offset = 20
last_state = "stop"

def initialize_robot():
    # Initialize settings
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

def outHandle():
    global last_state
    if last_state == 'left':
        px.set_dir_servo_angle(-30)
        px.backward(10)
    elif last_state == 'right':
        px.set_dir_servo_angle(30)
        px.backward(10)
    while True:
        currentSta = get_status()
        if currentSta != last_state:
            break
    sleep(0.001)

def get_status():
    val_list=px.get_grayscale_data()
    _state = px.get_line_status(val_list)  # [bool, bool, bool], 0 means line, 1 means background
    print("val_list, _state: %s %s"%(val_list, _state))
    if _state == [0, 0, 0]:
        return 'stop'
    elif _state[2] == 1:
        return 'left'
    elif _state[0] == 1:
        return 'right'
    elif _state[1] == 1:
        return 'forward'

def main():
    global px_power, offset, last_state
    try:
        while True:
            gm_state = get_status()
            print("current_state: %s"%(gm_state))

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

    finally:
        px.stop()
        print("stop and exit")
        sleep(0.1)


if __name__=='__main__':
    initialize_robot()
    main()

