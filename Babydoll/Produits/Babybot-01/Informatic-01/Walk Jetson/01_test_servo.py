import busio
import board
from adafruit_servokit import ServoKit

# Initialize the I2C bus and ServoKit
i2c_bus = busio.I2C(board.SCL_1, board.SDA_1)
kit = ServoKit(channels=16, i2c=i2c_bus, address=0x41)

# Main loop to request user input
while True:
    user_input = input("Enter servo index and angle (e.g., '0 25'), or 'q' to quit: ")

    if user_input.lower() == 'q':
        print("Exiting program.")
        break

    try:
        # Parse input into servo index and angle
        servo_index, angle = map(int, user_input.split())
        
        # Validate inputs
        if 0 <= servo_index < 16 and 0 <= angle <= 180:
            kit.servo[servo_index].angle = angle
            print(f"Set servo {servo_index} to angle {angle}°.")
        else:
            print("Invalid input. Ensure the servo index is between 0-15 and angle is between 0-180.")
    except ValueError:
        print("Invalid input format. Please enter as 'index angle' or 'q' to quit.")  