#API checks
# Import necessary classes from your robot_hat library
from robot_hat import Motors, Servo, ADC, PWM, RGB_LED, Buzzer, Ultrasonic, ADXL345, utils, I2C

# Initialize components
motors = Motors()
servo = Servo("P0") # Assume a servo is connected to PWM pin 0
adc = ADC(0) # Assume an ADC device is connected to channel 0
rgb_led = RGB_LED(PWM("P0"), PWM("P1"), PWM("P2")) # Assuming RGB LED is connected to PWM pins 0, 1, and 2
#buzzer = Buzzer(PWM(0)) # Assuming a passive buzzer is connected to PWM pin 0
#ultrasonic = Ultrasonic("D2", "D3") # Assuming ultrasonic sensor's TRIG and ECHO pins are connected to D2 and D3 respectively
accelerometer = ADXL345() # Initializing ADXL345 accelerometer

def check_battery_voltage():
    voltage = utils.get_battery_voltage()
    print(f"Battery Voltage: {voltage}V")
    if voltage < 3.5:
        print("Warning: Battery voltage is low. Please recharge or replace the battery.")

def motor_status():
    try:
        # Example: check if motors are functioning by running them at 50% speed for 1 second
        motors.forward(50)
        utils.run_command("sleep 1")
        motors.stop()
        print("Motors functioning correctly.")
    except Exception as e:
        print(f"Motor Error: {e}")

def servo_test():
    try:
        # Sweep servo from -90 to 90 degrees
        for angle in range(-90, 91, 10):
            servo.angle(angle)
            utils.run_command("sleep 0.1")
        print("Servo test completed successfully.")
    except Exception as e:
        print(f"Servo Error: {e}")

def sensor_readings():
    try:
        # Reading from ADC
        adc_value = adc.read()
        print(f"ADC Value: {adc_value}")

        # Ultrasonic distance
        #distance = ultrasonic.read()
        #print(f"Distance: {distance} cm")

        # Accelerometer readings
        x, y, z = accelerometer.read()
        print(f"Acceleration - X: {x}, Y: {y}, Z: {z}")

    except Exception as e:
        print(f"Sensor Reading Error: {e}")


if __name__ == "__main__":
    print("Starting robot system check...")
    check_battery_voltage()
    motor_status()
    servo_test()
    sensor_readings()
    print("System check completed.")
