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
        picarx.set_dir_servo_angle(-12 if last_state == 'left' else 12)
        picarx.backward(5)
        while picarx.get_line_status(picarx.get_grayscale_data()) == last_state:
            sleep(0.001)

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

    try:
        while True:
            gm_val_list = picarx.get_grayscale_data()
            gm_state = picarx.get_line_status(gm_val_list)
            print("Grayscale Data:%s, Line Status:%s" % (gm_val_list, gm_state))

            if gm_state != "stop":
                last_state = gm_state
                sleep(0.01)

            if distance / 0.0205 < 2:
                distance = (alpha * px_power + distance)
                sleep(0.01)

            #if distance > 0.04:
            #    picarx.stop()

            elif gm_state == 'forward':
                picarx.set_dir_servo_angle(0)
                picarx.forward(px_power)
                sleep(0.01)
            elif gm_state in ['left', 'right']:
                picarx.set_dir_servo_angle(offset if gm_state == 'left' else -offset)
                picarx.forward(px_power)
                sleep(0.01)
            else:
                handle_out()

            print('Distance: ' + str(distance / 0.0205))
            sleep(0.1)
    finally:
        picarx.stop()
        picarx.stop()
        sleep(0.2)

if __name__ == '__main__':
    main()