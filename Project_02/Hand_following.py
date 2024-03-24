from picarx import Picarx
import time
from robot_hat import Pin

px = Picarx()

# Constants
MAX_DISTANCE = 120  # Maximum reliable reading distance of the ultrasonic sensor
GOAL_LOCATION = 1  # Goal location (distance from the robot)
FMAX = 1.0  # Maximum force

led = Pin('LED')
led.value(0)

def initialize_robot():
    #Set all to zero
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

# Function to calculate the potential field
def calculate_potential_field(distance):
    attractive_potential = 0.5 * (GOAL_LOCATION - distance) ** 2
    force = max(2 * attractive_potential, FMAX)  # Limit the force to FMAX
    return force

def main():
    
    try:
        while True:
            distance = round(px.ultrasonic.read())
            print("Distance:", distance)
            
            if 1 <= distance <= MAX_DISTANCE:
                force = calculate_potential_field(distance)
                print("Force:", force)
                
                # Turn on LED if the goal location is beyond the current distance but within sensor's max range
                if GOAL_LOCATION < distance <= MAX_DISTANCE:
                    led.value(1)
                    px.forward(force)
                else:
                    led.value(0)
                    px.stop()
            else:
                led.value(0)
                px.stop()

            time.sleep(0.1)  # Adjust as needed for smoother operation
                
    except KeyboardInterrupt:
        px.stop()

    finally:
        px.stop()
        
if __name__ == '__main__':
    initialize_robot()
    main()
