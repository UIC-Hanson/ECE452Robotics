from picarx import Picarx
from time import sleep
#our scripts
from robot_control import get_power_level

# Define initial variables
picarx = Picarx()
picarx.set_grayscale_reference([1400, 1400, 1400])

# Ask the user for the power level and calculate the alpha value
px_power = get_power_level()
alpha = 0.012921758 # Alpha value

last_state = None
offset = 20
distance = 0

def handle_out():
    global last_state
    if last_state == 'left' or last_state == 'right':
        picarx.set_dir_servo_angle(-12 if last_state == 'left' else 12)
        picarx.backward(5)
        while picarx.get_line_status(picarx.get_grayscale_data()) == last_state:
            sleep(0.001)

if __name__ == '__main__':
    try:
        while True:
            gm_val_list = picarx.get_grayscale_data()
            gm_state = picarx.get_line_status(gm_val_list)
            print("Grayscale Data:%s, Line Status:%s" % (gm_val_list, gm_state))

            if gm_state != "stop":
                last_state = gm_state

            if distance / 0.0205 < 2:
                distance += alpha * px_power

            if distance > 0.04:
                picarx.stop()

            elif gm_state == 'forward':
                picarx.set_dir_servo_angle(0)
                picarx.forward(px_power)
            elif gm_state in ['left', 'right']:
                picarx.set_dir_servo_angle(offset if gm_state == 'left' else -offset)
                picarx.forward(px_power)
            else:
                handle_out()

            print('Distance: ' + str(distance / 0.0205))
            sleep(0.1)
    finally:
        picarx.stop()
