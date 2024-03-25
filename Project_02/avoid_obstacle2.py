from picarx import Picarx
import time

px = Picarx()

# Constants
SAFE_DISTANCE = 40
FORWARD_SPEED = 60
TURN_SPEED = 30
AVOIDANCE_TIME = 1  # Time after clearing an obstacle before taking further action

def avoidance_maneuver():
    """Executes the avoidance maneuver and logs the duration."""
    start_time = time.time()
    px.forward(TURN_SPEED)
    px.set_dir_servo_angle(30)  # Turn right to avoid the obstacle
    
    while round(px.ultrasonic.read(), 2) < SAFE_DISTANCE:
        time.sleep(0.1)
    duration = time.time() - start_time
    px.set_dir_servo_angle(0)  # Reset to straight ahead after avoiding the obstacle
    time.sleep(AVOIDANCE_TIME)  # Wait for 1 second after avoiding the obstacle

    return duration

def reciprocal_maneuver(duration):
    """Executes the reciprocal maneuver based on the duration of the initial avoidance."""
    px.forward(TURN_SPEED)
    px.set_dir_servo_angle(-30)  # Turn left to initiate reciprocal maneuver
    time.sleep(.25)
    px.set_dir_servo_angle(0)
    time.sleep(AVOIDANCE_TIME)
    px.set_dir_servo_angle(30)  # Turn right to get back on the original course
    time.sleep(duration)  # Match the duration of the initial avoidance

def main():
    try:
        while True:
            distance = round(px.ultrasonic.read(), 2)
            
            if distance >= SAFE_DISTANCE:
                px.set_dir_servo_angle(0)
                px.forward(FORWARD_SPEED)
                time.sleep(0.1)

            else:
                print("Distance:", distance)
                # Obstacle detected, execute avoidance maneuver
                duration = avoidance_maneuver()
                print("duration:", duration)
                # After avoiding, execute reciprocal maneuver to attempt to return to original path
                reciprocal_maneuver(duration)
                px.forward(FORWARD_SPEED)
                break

    except KeyboardInterrupt:
        px.stop()

    finally:
        px.stop()

if __name__ == '__main__':
    main()
