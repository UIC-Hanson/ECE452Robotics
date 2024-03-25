from picarx import Picarx
import time

px = Picarx()

# Constants
SAFE_DISTANCE = 40
FORWARD_SPEED = 50
TURN_SPEED = 30
AVOIDANCE_TIME = 0.5  # Time after clearing an obstacle before taking further action

def initialize_robot():
    #Set all to zero
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

def avoidance_maneuver():
    """Executes the avoidance maneuver and logs the duration."""
    start_time = time.time()
    px.forward(TURN_SPEED)
    px.set_dir_servo_angle(30)  # Turn right to avoid the obstacle
    while round(px.ultrasonic.read(), 2) < SAFE_DISTANCE:
        time.sleep(0.01)
    duration = time.time() - start_time
    px.set_dir_servo_angle(0)  # Reset to straight ahead after avoiding the obstacle

    return duration

def reciprocal_maneuver(duration):
    """Executes the reciprocal maneuver based on the duration of the initial avoidance."""
      # Turn left to initiate reciprocal maneuver
    time.sleep(duration)  # Match the duration of the initial avoidance
    px.set_dir_servo_angle(0)  # Straighten up
    px.forward(TURN_SPEED)
    time.sleep(3)  # Continue straight for 3 seconds
    px.set_dir_servo_angle(-30)  # Turn right to get back on the original course
    time.sleep(duration)  # Match the duration to complete the reciprocal course
    px.forward(FORWARD_SPEED)

def main():
    try:
        while True:
            distance = round(px.ultrasonic.read(), 2)
            print("Distance:", distance)

            if distance >= SAFE_DISTANCE:
                px.set_dir_servo_angle(0)
                px.forward(FORWARD_SPEED)
                time.sleep(0.01)
            else:
                # Obstacle detected, execute avoidance maneuver
                duration = avoidance_maneuver()
                
                px.set_dir_servo_angle(-30)
                time.sleep(AVOIDANCE_TIME)  # Wait after avoiding the obstacle

                # After avoiding, execute reciprocal maneuver to attempt to return to original path
                reciprocal_maneuver(duration)
                break  # End loop after executing maneuvers; adjust as necessary for continuous operation

    except KeyboardInterrupt:
        px.stop()

    finally:
        px.stop()

if __name__ == '__main__':
    initialize_robot()
    main()
