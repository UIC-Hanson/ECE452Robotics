from picarx import Picarx
from time import sleep

px = Picarx()

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

def main():
    px_power = get_power_level()

    px_power_for_alpha =px_power/1000
    print(f"Power level set to: {px_power}, Alpha power calc value: {px_power_for_alpha}")
    distance = 0
    alpha = 0.017881989
    wheelsize = 0.0205  # Assuming this is the wheel diameter in meters
    offset = 20  # Steering angle offset for left/right corrections
    run=True

    try:
        while run==True:
            distance += alpha * px_power_for_alpha  # Increment distance based on power
            px.forward(px_power)
            
            if distance / wheelsize >= 2:
                print("We have crossed the desert to the holy land, 2 meters away.")
                run=False

            sleep(0.01)  # Sleep at the end of the loop to ensure some delay

    finally:
        px.set_cam_tilt_angle(0)
        px.set_cam_pan_angle(0)  
        px.set_dir_servo_angle(0)  
        px.stop()
        sleep(.2)
        print("Robot stopped.")

if __name__ == '__main__':
    initialize_robot()
    main()
