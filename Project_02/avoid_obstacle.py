from picarx import Picarx
import time

POWER = 50
SafeDistance = 40   # > 40 safe
DangerDistance = 20 # > 20 && < 40 turn around,
                    # < 20 backward

def main():
    try:
        px = Picarx()
        # px = Picarx(ultrasonic_pins=['D2','D3']) # tring, echo

        # Variables to store traveled distance and angle
        distance_traveled = 0
        angle_traveled = 0

        while True:
            distance = round(px.ultrasonic.read(), 2)
            print("distance: ", distance)

            if distance >= SafeDistance:
                px.set_dir_servo_angle(0)
                px.forward(POWER)
                distance_traveled += POWER * 0.1  # Assuming 0.1 second per iteration
            elif distance >= DangerDistance:
                px.set_dir_servo_angle(30)
                px.forward(POWER)
                time.sleep(0.1)
                distance_traveled += POWER * 0.1  # Assuming 0.1 second per iteration
                angle_traveled += 30  # Assuming the robot turns at 30 degrees
            else:
                px.set_dir_servo_angle(-30)
                px.backward(POWER)
                time.sleep(0.5)

            # Check if the obstacle is no longer detected
            if distance >= SafeDistance:
                # Mirror the stored data to return to the original trajectory
                px.set_dir_servo_angle(-30)  # Reverse the last angle adjustment
                px.forward(POWER)  # Move forward to mirror the backward movement
                time.sleep(0.5)  # Assuming the same duration for backward movement
                px.set_dir_servo_angle(0)  # Set the trajectory angle to the original
                px.forward(POWER)  # Continue forward
                time.sleep(0.1)  # Adjust as needed

                # Reset traveled distance and angle
                distance_traveled = 0
                angle_traveled = 0

    finally:
        px.forward(0)


if __name__ == "__main__":
    main()
