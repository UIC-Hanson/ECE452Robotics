from picarx import Picarx
import time

px = Picarx()

# Constants
SAFE_DISTANCE = 50  # Distance considered safe before taking avoidance measures
TOO_CLOSE_DISTANCE = 10  # Distance considered too close, requiring immediate stop
FORWARD_SPEED = 60
TURN_SPEED = 30
REVERSE_INTERVAL = 2  # Time to continue on the course after the object is out of detection range

# Movement memory for mirroring the motion
movements = []
logging = False  # Flag to start logging movements

def reverse_course():
    while movements:
        angle, speed, duration = movements.pop()
        # Reverse the steering angle for mirroring the turn
        px.set_dir_servo_angle(-angle)
        # No need to change the speed as we're only interested in reversing the turns
        time.sleep(duration)
    px.stop()

def main():
    global logging
    try:
        while True:
            distance = round(px.ultrasonic.read(), 2)
            print("Distance:", distance)

            if distance < TOO_CLOSE_DISTANCE:
                # Too close, stop immediately
                px.stop()
                time.sleep(1)  # Brief pause for any potential adjustment or decision
            elif distance < SAFE_DISTANCE:
                if not logging:
                    # Start logging movements as soon as an object is detected
                    logging = True
                    movements.clear()  # Ensure previous movements are cleared
                
                # Avoidance maneuver: Turn right
                px.set_dir_servo_angle(30)  # Adjust angle to turn right
                px.forward(TURN_SPEED)
                movements.append((30, TURN_SPEED, 0.1))  # Log movement
                time.sleep(0.1)
            else:
                if logging:
                    # Continue straight for a short interval after avoiding the obstacle
                    logging = False  # Stop logging movements
                    px.forward(TURN_SPEED)
                    px.set_dir_servo_angle(-30)
                    time.sleep(REVERSE_INTERVAL)
                    reverse_course()  # Reverse the course
                else:
                    # Safe, move forward
                    px.set_dir_servo_angle(0)
                    px.forward(FORWARD_SPEED)
                    time.sleep(0.1)

    except KeyboardInterrupt:
        px.stop()

    finally:
        px.stop()

if __name__ == '__main__':
    main()
