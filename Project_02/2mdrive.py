from picarx import Picarx
from time import sleep

px = Picarx()

offset = 20  # Steering angle offset for left/right corrections
px_power = .25 #speed

def initialize_robot():
    #Set all to zero
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

def main():
    global offset, px_power
    px_power_for_alpha=px_power*.95 #during testing it stopped just short of 2m, adjust this slightly
    distance = 0
    alpha = 0.000332
    wheelsize = 0.0205  # wheel diameter in meters
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
        px.stop()
        initialize_robot()
        sleep(.2)
        print("Robot stopped.")

if __name__ == '__main__':
    initialize_robot()
    main()
