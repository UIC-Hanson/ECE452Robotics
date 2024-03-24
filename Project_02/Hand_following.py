from picarx import Picarx
import time
from robot_hat import Pin

px = Picarx()

#Define initial variables
maxDistance = 0 # Maximum distance ultrasonic sensor can read
distance = 0 # Current reading from 
fmax = 1.0  # Maximum force
goal_location = 0.1  # Goal location (distance from the robot)

led = Pin('LED')
led.value(0)

def initialize_robot():
    #Set all to zero
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

# Function to calculate the potential field
def calculate_potential_field(distance):
    attractive_potential = 0.5 * (goal_location - distance) ** 2
    force = min(2 * attractive_potential, fmax)  # Limit the force to fmax
    return force

def main():
    try:
        
        
        
        #Read distance and calculate force based on distance
        while(True):
            distance = round(px.ultrasonic.read())
            if (distance >= 2 and distance <= 400):
                maxDistance = distance
            
            print(distance)
            force = calculate_potential_field(distance)
            
        # Turn on LED if hand is detected
        if distance < goal_location:
            led.value(1)
        else:
            led.value(0)
            
        # Control forward velocity proportional to the force
        # Stop if object is too close
        if(distance == goal_location):
            px.stop()
                
        # Move forward if obect is detected
        elif(distance >= goal_location and distance <= maxDistance):
            px.forward(force)
            
            time.sleep(0.1)
                
    #If user uses keyboard input such as ctrl + c, then the robot will stop

    except KeyboardInterrupt:
        px.stop()

    finally:
                
        px.stop()
        
if __name__ == '__main__':
    initialize_robot()
    main()