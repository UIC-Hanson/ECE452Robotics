from picarx import Picarx
from time import sleep, time
from datetime import datetime
import csv
import os

px = Picarx()

# Initialize servos
px.set_dir_servo_angle(0)
px.set_cam_pan_angle(0)
px.set_cam_tilt_angle(0)

current_state = None
# Ask the user to enter the power level, ensuring it's between 1 and 100
while True:
    try:
        px_power = int(input("Enter power level (1-100): "))
        if 1 <= px_power <= 100:
            break
        else:
            print("Please enter a value between 1 and 100.")
    except ValueError:
        print("Invalid input. Please enter a numerical value between 1 and 100.")

offset = 30
last_state = "stop"
timer_started = False
tracking_start_time = None
csv_file_path = os.path.expanduser("~/alpha_data.csv")

def append_to_csv(data, file_path):
    file_exists = os.path.isfile(file_path)
    with open(file_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(["date", "start_time", "px_power", "duration"])
        writer.writerow(data)

def get_status(val_list):
    state = px.get_line_status(val_list)
    
    if state == [0, 0, 0]:
        return 'stop'
    elif state[1] == 1:
        return 'forward'
    elif state[0] == 1:
        return 'right'
    elif state[2] == 1:
        return 'left'
    else:
        print("Charlie is in the bad place, State was: ", state)
        return 'unknown'

if __name__ == '__main__':
    start_time = datetime.now()  # Record script start time
    try:
        px.forward(px_power)  # Start moving forward slowly
        while True:
            gm_val_list = px.get_grayscale_data()
            gm_state = get_status(gm_val_list)
            current_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # Get current time with milliseconds

            if gm_state in ['forward', 'left', 'right'] and not timer_started:
                print("Line detected. Starting timer.")
                tracking_start_time = time()
                timer_started = True

            if gm_state == "stop" and timer_started:
                print("End of line reached or lost line. Stopping robot.")
                break

            if gm_state == 'forward':
                px.set_dir_servo_angle(0)
            elif gm_state == 'left':
                px.set_dir_servo_angle(offset)
            elif gm_state == 'right':
                px.set_dir_servo_angle(-offset)
    finally:
        px.set_dir_servo_angle(0)
        px.stop()
        px.stop()  # Call twice per documentation
        if timer_started:
            duration = time() - tracking_start_time
            print(f"Time taken to follow the line: {duration:.2f} seconds")
            # Ask if the user wants to save the data
            save_data = input("Do you want to commit the data to the CSV file? (yes/no): ").lower()
            if save_data in ['yes', 'y']:
                date_str = start_time.strftime("%y-%m-%d")
                start_time_str = start_time.strftime("%H:%M:%S")
                append_to_csv([date_str, start_time_str, px_power, f"{duration:.2f}"], csv_file_path)
                print("Data committed to the CSV file.")
            else:
                print("Data not saved.")
        else:
            print("No line was detected.")
        sleep(0.2)
