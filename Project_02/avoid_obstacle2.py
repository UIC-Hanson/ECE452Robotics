from picarx import Picarx
import time

px = Picarx()

# Constants
DANGER_DISTANCE =15
SAFE_DISTANCE = 40
FORWARD_SPEED = 60
TURN_SPEED = 30
AVOIDANCE_TIME = 1  # Time after clearing an obstacle before taking further action

def avoidance_maneuver():
    """Executes the avoidance maneuver and logs the duration."""
    start_time = time.time()
    px.set_dir_servo_angle(30)  # Turn right to avoid the obstacle
    
    while round(px.ultrasonic.read(), 2) < SAFE_DISTANCE:
        time.sleep(0.1)
    duration = time.time() - start_time
    px.set_dir_servo_angle(0)  # Reset to straight ahead after avoiding the obstacle
    time.sleep(AVOIDANCE_TIME)  # Wait for 1 second after avoiding the obstacle

    return duration

def reciprocal_maneuver(duration):
    """Executes the reciprocal maneuver based on the duration of the initial avoidance."""
    
    px.set_dir_servo_angle(-60)  # Turn left to initiate reciprocal maneuver
    time.sleep(duration*1.2)
    px.set_dir_servo_angle(0)
    time.sleep(AVOIDANCE_TIME)
    px.set_dir_servo_angle(30)  # Turn right to get back on the original course
    time.sleep(duration)  # Match the duration of the initial avoidance

def main():
    run == True
    try:
        while run == True:
            distance = round(px.ultrasonic.read(), 2)
            
            if distance >= SAFE_DISTANCE:
                px.set_dir_servo_angle(0)
                px.forward(FORWARD_SPEED)
                time.sleep(0.1)

            elif distance <= DANGER_DISTANCE:
                run = FALSE
            else:
                print("Distance:", distance)
                # Obstacle detected, execute avoidance maneuver
                px.forward(TURN_SPEED)
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
