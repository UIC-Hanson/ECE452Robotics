from picarx import Picarx
import time

POWER = 50
SafeDistance = 40   # > 40 safe
DangerDistance = 20 # > 20 && < 40 turn around,
                    # < 20 backward

def initialize_robot(px):
    # Initialize settings
    px.set_dir_servo_angle(0)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(0)

def main():
    try:
        px = Picarx()
        initialize_robot(px)

        # Variables to store traveled distance and angle
        distance_traveled = 0
        angle_traveled = 0
        obstacle_detected = False  # Track if obstacle was detected in previous iteration

        while True:
            distance = round(px.ultrasonic.read(), 2)
            print("distance: ", distance)

            if distance >= SafeDistance:
                px.set_dir_servo_angle(0)
                px.forward(POWER)
                distance_traveled += POWER * 0.1  # Assuming 0.1 second per iteration
                obstacle_detected = False
            elif distance >= DangerDistance:
                if not obstacle_detected:
                    px.set_dir_servo_angle(30)
                    px.forward(POWER)
                    time.sleep(0.1)
                    angle_traveled += 30  # Assuming the robot turns at 30 degrees
                    obstacle_detected = True
                else:
                    px.set_dir_servo_angle(0)  # Keep the direction straight while obstacle is still detected
            else:
                px.set_dir_servo_angle(-30)
                px.backward(POWER)
                time.sleep(0.5)
                obstacle_detected = False

            # Check if the obstacle is no longer detected and it was previously detected
            if distance >= SafeDistance and obstacle_detected:
                # Mirror the stored data to return to the original trajectory
                px.set_dir_servo_angle(-30)  # Reverse the last angle adjustment
                px.forward(POWER)  # Move forward to mirror the backward movement
                time.sleep(0.5)  # Assuming the same duration for backward movement
                px.set_dir_servo_angle(0)  # Set the trajectory angle to the original
                px.forward(POWER)  # Continue forward
                time.sleep(0.1)  # Adjust as needed

                # Reset traveled distance and angle after mirrored movement is completed
                distance_traveled = 0
                angle_traveled = 0

    finally:
        px.forward(0)


if __name__ == "__main__":
    main()
