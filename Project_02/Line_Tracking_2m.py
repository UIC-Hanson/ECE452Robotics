from picarx import Picarx
from time import sleep, time

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

def main():
    initialize_robot()
    px_power = get_power_level()
    print(f"Power level set to: {px_power}")
    distance = 0
    alpha = 0.012921758
    offset = 30
    last_state = None

    try:
        px.forward(px_power)  # Start moving forward
        while True:
            gm_val_list = px.get_grayscale_data()
            gm_state = px.get_line_status(gm_val_list)
            print(f"Grayscale Data:{gm_val_list}, Line Status:{gm_state}")

            if gm_state != "stop":
                last_state = gm_state
                sleep(0.01)

            if distance / 0.0205 < 2:
                distance += alpha * px_power
                sleep(0.01)
                
            if distance > 0.04:
                px.stop()
                break  # Stop the loop if the distance condition is met

            elif gm_state == 'forward':
                px.set_dir_servo_angle(0)
                sleep(0.01)
            elif gm_state in ['left', 'right']:
                px.set_dir_servo_angle(offset if gm_state == 'left' else -offset)
                sleep(0.01)
            else:
                handle_out(px, last_state)
                sleep(0.01)

            px.forward(px_power)  # Continue moving forward
            print(f'Distance: {distance / 0.0205}')
            sleep(0.1)
    finally:
        px.stop()

if __name__ == '__main__':
    main()