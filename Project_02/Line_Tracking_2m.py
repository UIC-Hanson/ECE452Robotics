from picarx import Picarx
from time import sleep, time
from robot_control import get_power_level, get_status
from gpiozero import Device  # Add this import for cleanup

px = Picarx()

def initialize_robot():
    # Initialize settings
    px.set_grayscale_reference([1400, 1400, 1400])
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

def handle_out(px, last_state):
    # Modified handle_out with timeout
    correction_angle = -12 if last_state == 'left' else 12
    px.set_dir_servo_angle(correction_angle)
    px.backward(5)
    start_time = time()
    timeout = 2
    while get_status() == last_state:
        sleep(0.001)
        if time() - start_time > timeout:
            print("Timeout occurred in handle_out")
            break

def main():
    initialize_robot()
    px_power = get_power_level()
    print(f"Power level set to: {px_power}")
    distance = 0
    alpha = 0.012921758
    offset = 30
    last_state = None

    try:
        px.forward(px_power)
        while True:
            gm_state = get_status()

            if gm_state != "stop":
                last_state = gm_state

            if distance / 0.0205 < 2:
                distance += alpha * px_power
            
            if distance > 0.04:
                px.stop()
                break  # Stop the loop if the distance condition is met
            elif gm_state == 'forward':
                px.set_dir_servo_angle(0)
            elif gm_state in ['left', 'right']:
                px.set_dir_servo_angle(offset if gm_state == 'left' else -offset)
            else:
                handle_out(px, last_state)

            px.forward(px_power)  # Ensure continuous movement
            print(f'Distance: {distance / 0.0205}')
            sleep(0.1)
    finally:
        px.stop()
        Device.close_all()  # Cleanup GPIO resources

if __name__ == '__main__':
    main()