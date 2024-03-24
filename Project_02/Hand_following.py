from picarx import Picarx
import time
from robot_hat import Pin

px = Picarx()

# Constants
MAX_DISTANCE = 120  # Maximum reliable reading distance of the ultrasonic sensor
GOAL_LOCATION = 1  # Goal location (distance from the robot)
FMAX = 100  # Maximum force

led = Pin('LED')

def initialize_robot():
    #Set all to zero
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

def calculate_potential_field(distance):
    # Function to calculate the potential field
    global FMAX

    velocity=1.5*distance
    print("Velocity:", velocity)
    force = min(velocity, FMAX)  # Limit the force to FMAX
    return force

#Use these two functions to make the def main easier to read.
def MoveForward(force):
    global led
    led.value(1)
    px.forward(force)

def StopMovement():
    global led
    led.value(0)
    px.stop()

def main():
    led.value(0)
    try:
        while True:
            distance = round(px.ultrasonic.read())
            print("Distance:", distance)
            
            if 1 <= distance <= MAX_DISTANCE:
                force = calculate_potential_field(distance)
                print("Force:", force)
                
                # Turn on LED if the goal location is beyond the current distance but within sensor's max range
                if GOAL_LOCATION < distance <= MAX_DISTANCE:
                    MoveForward(force)
                else:
                    StopMovement()
            else:
                StopMovement()

            time.sleep(0.01)  # Adjust as needed
                
    except KeyboardInterrupt:
        StopMovement()

    finally:
        StopMovement()
        
if __name__ == '__main__':
    initialize_robot()
    main()
