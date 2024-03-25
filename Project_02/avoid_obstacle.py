from picarx import Picarx
import time
from robot_hat import Pin

px = Picarx()

# Constants
MAX_DISTANCE = 25  # Maximum reliable reading distance of the ultrasonic sensor
GOAL_LOCATION = 5  # Goal location (distance from the robot)
FMAX = 100  # Maximum force
K_R = 1  # Constant for repulsive force calculation
GAMMA = 2  # Integer determining the shape of the piecewise function
ETA_0 = 10  # Range of influence of obstacles

led = Pin('LED')

def initialize_robot():
    # Set all to zero
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

def calculate_clearance(q, q_list):
    # Calculate clearance (minimum distance to obstacle)
    return min(abs(q - q_i) for q_i in q_list)

def calculate_repulsive_force(distance, obstacles):
    # Calculate repulsive force based on distance to obstacle
    global K_R, GAMMA, ETA_0
    
    clearance = calculate_clearance(distance, [obstacle['x'] for obstacle in obstacles])
    if clearance <= ETA_0:
        force = (K_R / (ETA_0 ** 2)) * ((1 / clearance) - (1 / ETA_0)) ** (GAMMA - 1)
    else:
        force = 0
    
    return force

def move_forward(force):
    global led
    led.value(1)
    px.forward(force)

def stop_movement():
    global led
    led.value(0)
    px.stop()

def mirror_motion(V_list, Θ_list):
    # Mirror the motion after encountering the obstacle
    px.set_dir_servo_angle(0)  # Fix the angle of the front wheel
    while px.get_angle() != -px.get_angle_threshold():  # Wait until the current heading becomes -θ_threshold
        pass
    px.set_dir_servo_angle(0)  # Set the front-wheel angle to 0 degrees
    px.set_velocity(-px.get_velocity_threshold())  # Reverse the velocity
    while px.get_y() != px.get_y_threshold():  # Keep the robot moving until y_k = y_threshold
        pass
    px.stop()  # Stop the robot

    # Mirror the steering angles in reverse order
    for Θ in reversed(Θ_list):
        Θ_mirrored = -Θ  # Reverse the direction of steering
        # Adjust robot's movement based on Θ_mirrored

def main():
    led.value(0)
    try:
        obstacles = []  # Define obstacles here
        
        V_list = []
        Θ_list = []

        while True:
            distance = round(px.ultrasonic.read())
            print("Distance:", distance)

            if 1 <= distance <= MAX_DISTANCE:
                force = calculate_repulsive_force(distance, obstacles)
                print("Force:", force)

                if force > 0:
                    # Record commanded velocities and steering angles
                    V_list.append(px.get_velocity())
                    Θ_list.append(px.get_angle())

                    # Implement rectilinear motion with a trajectory angle of 30 degrees
                    px.set_dir_servo_angle(30)
                    px.set_velocity(px.get_velocity_threshold())
                else:
                    stop_movement()
                    
                # After avoiding the obstacle and waiting for ∆t, mirror the previous movements
                mirror_motion(V_list, Θ_list)

            else:
                stop_movement()

            time.sleep(0.01)  # Adjust as needed

    except KeyboardInterrupt:
        stop_movement()

    finally:
        stop_movement()

if __name__ == '__main__':
    initialize_robot()
    main()
