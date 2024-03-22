from picarx import Picarx
from time import sleep
from robot_control import get_power_level
px = Picarx()


# Define initial variables
px.set_grayscale_reference([1400, 1400, 1400]) #do we need this?
px_power = 0
alpha = 0.012921758 # Alpha value

last_state = None
offset = 30

# Initialize servos
px.set_dir_servo_angle(0)
px.set_cam_pan_angle(0)
px.set_cam_tilt_angle(0)

def handle_out():
    global last_state
    if last_state == 'left' or last_state == 'right':
        px.set_dir_servo_angle(-12 if last_state == 'left' else 12)
        px.backward(5)
        while px.get_line_status(px.get_grayscale_data()) == last_state:
            sleep(0.001)

def main():
    global px_power
    px_power = get_power_level()
    print(f"Power level set to: {px_power}")
    distance = 0
    
    try:
        px.forward(px_power)  # Start moving forward
        while True:
            gm_val_list = px.get_grayscale_data()
            gm_state = px.get_line_status(gm_val_list)
            print("Grayscale Data:%s, Line Status:%s" % (gm_val_list, gm_state))

            if gm_state != "stop":
                last_state = gm_state

            if distance / 0.0205 < 2:
                distance += alpha * px_power
                sleep(0.01)
            if distance > 0.04:
                px.stop()
            elif gm_state == 'forward':
                px.set_dir_servo_angle(0)
                px.forward(px_power)
                sleep(0.01)
            elif gm_state in ['left', 'right']:
                px.set_dir_servo_angle(offset if gm_state == 'left' else -offset)
                px.forward(px_power)
                sleep(0.01)
            else:
                handle_out()

            print('Distance: ' + str(distance / 0.0205))
            sleep(0.1)
    finally:
        px.stop()
        px.stop()

if __name__ == '__main__':
    main()
