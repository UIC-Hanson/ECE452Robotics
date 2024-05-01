import time

class RobotControl:
    def __init__(self):
        self.px = Picarx()
        self.power = 0
        self.current_state = 'stop'
        self.offset = 30
        self.camera_pan = 0  # Camera pan angle
        self.camera_tilt = 0  # Camera tilt angle
        self.initialize_robot()

    def initialize_robot(self):
        self.px.set_dir_servo_angle(0)
        self.px.set_cam_pan_angle(self.camera_pan)
        self.px.set_cam_tilt_angle(self.camera_tilt)

    def set_power_level(self):
        """Prompts the user to set the robot's speed."""
        attempts = 0
        while attempts < 5:
            try:
                power = int(input("Enter Robot Speed (1-100): "))
                if 1 <= power <= 100:
                    self.power = power
                    return
                else:
                    print("Please enter a value between 1 and 100.")
            except ValueError:
                print("Invalid input. Please enter a numerical value between 1 and 100.")
            attempts += 1
        print("Failed to set power level after several attempts.")

    def update_status(self):
        """Updates the robot's state based on sensor data."""
        val_list = self.px.get_grayscale_data()
        state = self.px.get_line_status(val_list)
    
        if state == [0, 0, 0]:
            self.current_state = 'outstate'
        elif state[0] == 1:
            self.current_state = 'right'
            self.offset = -30
        elif state[2] == 1:
            self.current_state = 'left'
            self.offset = 30
        elif state[1] == 1:
            self.current_state = 'forward'
        else:
            print(f"Unexpected state: {state}")
            self.current_state = 'stop'
        time.sleep(0.1)

    def stop(self):
        """Stop the robot."""
        self.px.stop()

    def navigate(self):
        """Continuously adjusts the robot's movements based on its state."""
        """Adjust the robot's movement based on the current state."""
        inboard_wheel = self.power * 0.8

        if self.current_state == 'left':
            self.px.set_dir_servo_angle(self.offset)
            self.px.set_motor_speed(1, inboard_wheel)
            self.px.set_motor_speed(2, self.power)

        elif self.current_state == 'right':
            self.px.set_dir_servo_angle(self.offset)
            self.px.set_motor_speed(2, inboard_wheel)
            self.px.set_motor_speed(1, self.power)

        elif self.current_state == 'forward':
            self.px.set_dir_servo_angle(0)
            self.px.forward(self.power)
        time.sleep(0.1)
        self.px.stop()
        time.sleep(0.1)

    def handle_outstate(self):
        """Handle when the robot is out of state."""
        last_state = self.current_state
        self.stop()
        self.px.set_dir_servo_angle(-self.offset if last_state == 'left' else self.offset)
        self.px.forward(-5)
        time.sleep(0.1)

        while self.current_state == last_state:
            self.update_status()

    def scan_camera(self):
        """Scans the environment by panning the camera left and right."""
        for angle in range(-90, 91, 10):  # Sweep from -90 to 90 degrees
            self.px.set_cam_pan_angle(angle)
            time.sleep(0.1)

        for angle in range(90, -91, -10):  # Sweep back to -90 degrees
            self.px.set_cam_pan_angle(angle)
            time.sleep(0.1)

def main():
    robot = RobotControl()
    robot.set_power_level()
    while True:
        robot.update_status()
        robot.navigate()
        robot.scan_camera()

if __name__ == "__main__":
    main()
