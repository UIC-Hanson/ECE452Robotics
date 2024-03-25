from picarx import Picarx
import time

px = Picarx()

# Constants
SAFE_DISTANCE = 40  # Distance considered safe before taking avoidance measures
TOO_CLOSE_DISTANCE = 10  # Distance considered too close, requiring immediate stop
FORWARD_SPEED = 70
TURN_SPEED = 30
REVERSE_INTERVAL = 1  # Time to continue on the course after the object is out of detection range

# Movement memory for mirroring the motion
movements = []
logging = False  # Flag to start logging movements

def initialize_robot():
    # Set all to zero
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

def reverse_course():
    px.set_dir_servo_angle(-30)
    while movements:
        angle, speed, duration = movements.pop()
        # Reverse the steering angle for mirroring the turn
        px.set_dir_servo_angle(-2*angle)
        px.forward(TURN_SPEED)
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
                time.sleep(1)
            elif distance < SAFE_DISTANCE:
                if not logging:
                    # Start logging movements as soon as an object is detected
                    #movements.clear()  # Moved to forward section
                    logging = True
                # Avoidance maneuver: Turn right
                px.set_dir_servo_angle(30)
                px.forward(TURN_SPEED)
                movements.append((30, TURN_SPEED, 0.1))  # Log movement
                time.sleep(0.1)
            else:
                if logging:
                    # Continue straight for a short interval after avoiding the obstacle
                    px.set_dir_servo_angle(0)
                    logging = False
                    reverse_course()  # Reverse the course
                else:
                    # Safe, move forward
                    movements.clear()  # Ensure previous movements are cleared
                    px.set_dir_servo_angle(0)
                    px.forward(FORWARD_SPEED)
                    time.sleep(0.1)

    except KeyboardInterrupt:
        px.stop()

    finally:
        px.stop()

if __name__ == '__main__':
    initialize_robot()
    main()
