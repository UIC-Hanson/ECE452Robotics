from picarx import Picarx
from time import sleep

""" Example
robot = RobotControl()
robot.set_power_level()
while True:
    robot.update_status()
    if robot.current_state != 'stop':
        robot.navigate()
    else:
        robot.handle_outstate()"""

class RobotControl:
    def __init__(self):
        self.px = Picarx()
        self.power = 0
        self.current_state = None
        self.offset = 30  # Steering angle offset for left/right corrections
        self.initialize_robot()

    def initialize_robot(self):
        """Initialize the robot's servos to neutral positions."""
        self.px.set_dir_servo_angle(0)
        self.px.set_cam_pan_angle(0)
        self.px.set_cam_tilt_angle(0)

    def set_power_level(self):
        """Set the power level of the robot after getting user input."""
        while True:
            try:
                power = int(input("Enter Robot Speed (1-100): "))
                if 1 <= power <= 100:
                    self.power = power
                    return
                else:
                    print("Please enter a value between 1 and 100.")
            except ValueError:
                print("Invalid input. Please enter a numerical value between 1 and 100.")

    def update_status(self):
        """Update the robot's current state based on sensor data."""
        val_list = self.px.get_grayscale_data()
        state = self.px.get_line_status(val_list)
        
        if state == [0, 0, 0]:
            self.current_state = 'outstate'
        elif state[0] == 1:
            self.current_state = 'right'
        elif state[2] == 1:
            self.current_state = 'left'
        elif state[1] == 1:
            self.current_state = 'forward'
        else:
            print(f"Unexpected state: {state}")
            self.current_state = 'stop'
        print(f"Sensor data, State, Current state: {val_list}, {state}, {self.current_state}")

    def stop(self):
        """Stop the robot."""
        self.px.stop()

    def navigate(self):
        """Adjust the robot's movement based on the current state."""
        inboard_wheel = self.power * 0.75

        if self.current_state == 'left':
            self.px.set_dir_servo_angle(self.offset)
            self.px.set_motor_speed(1, inboard_wheel)
            self.px.set_motor_speed(2, self.power)

        elif self.current_state == 'right':
            self.px.set_dir_servo_angle(-self.offset)
            self.px.set_motor_speed(2, inboard_wheel)
            self.px.set_motor_speed(1, self.power)

        elif self.current_state == 'forward':
            self.px.set_dir_servo_angle(0)
            self.px.forward(self.power)

        sleep(0.002)

    def handle_outstate(self):
        """Handle when the robot is out of state."""
        last_state = self.current_state
        self.stop()
        self.px.set_dir_servo_angle(-self.offset if last_state == 'left' else self.offset)
        self.px.forward(-5)
        sleep(0.1)

        while self.current_state == last_state:
            sleep(0.001)
            self.update_status()

    def rotate_camers(self):
        #stuff goes here
