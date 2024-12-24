import math
import json
import busio
import board
from adafruit_servokit import ServoKit
import Jetson.GPIO as GPIO
import argparse

class ServoDriver:
    def __init__(self, offset_file='offsets.json', limits_file='angle_limits.json'):
        self.offset_file = offset_file
        self._load_offsets()
        self.limits_file = limits_file
        self._load_limits()
        self.mapping = {
            (0,0):13,
            (0,1):14,
            (0,2):15,
            (1,0):5,
            (1,1):6,
            (1,2):7,
            (2,0):0,
            (2,1):1,
            (2,2):2,
            (3,0):13,
            (3,1):14,
            (3,2):15,
            (4,0):5,
            (4,1):6,
            (4,2):7,
            (5,0):2,
            (5,1):1,
            (5,2):0
        }
        i2c_bus = busio.I2C(board.SCL_1, board.SDA_1)
        self.kits = [
            ServoKit(channels=16, i2c=i2c_bus, address=0x41),
            ServoKit(channels=16, i2c=i2c_bus, address=0x60)
            ]
        self.oe_pin = 'GP122'
        GPIO.setup(self.oe_pin, GPIO.OUT, initial=GPIO.HIGH)
    
    def _get_mapping(self, leg, servo):
        """Return the servo number for a specific leg and servo."""
        if (leg, servo) in self.mapping:
            return self.mapping[(leg, servo)]
        else:
            return None

    def _adjust_sign(self, leg, servo, angle):
        if leg >= 3:
            if servo >= 1:
                return 180 - angle
            
        return angle

    def _load_offsets(self):
        """Load the servo offsets from a JSON file."""
        try:
            with open(self.offset_file, 'r') as f:
                content = f.read().strip()
                if content:
                    self.offsets = json.loads(content)
                else:
                    print("Offset file is empty. Initializing with zero offsets.")
                    self.offsets = {f"leg_{i}_servo_{j}": 0 for i in range(6) for j in range(3)}
        except FileNotFoundError:
            print("Offset file not found. Initializing with zero offsets.")
            self.offsets = {f"leg_{i}_servo_{j}": 0 for i in range(6) for j in range(3)}
        except json.JSONDecodeError:
            print("Error decoding the offset file. Initializing with zero offsets.")
            self.offsets = {f"leg_{i}_servo_{j}": 0 for i in range(6) for j in range(3)}

    def _save_offsets(self):
        """Save the current offsets to the file."""
        with open(self.offset_file, 'w') as f:
            json.dump(self.offsets, f, indent=4)

    def _get_offset_key(self, leg, servo):
        """Generate the key for accessing a specific servo's offset."""
        return f"leg_{leg}_servo_{servo}"
    
    def _load_limits(self):
        """Load the angle limits from a JSON file."""
        try:
            with open(self.limits_file, 'r') as f:
                self.angle_limits = json.load(f)
        except FileNotFoundError:
            print("Angle limits file not found.")
            self.angle_limits = {}

    def _clip_angle_range(self, leg, servo, angle):
        """Clips the given angle to the valid range for a specific leg and servo."""
        key = f"leg_{leg}_servo_{servo}"
        
        if key in self.angle_limits:
            min_angle = self.angle_limits[key]['min_angle']
            max_angle = self.angle_limits[key]['max_angle']
            
            # Clip the angle to the valid range
            clipped_angle = max(min_angle, min(max_angle, angle))
        
            return clipped_angle
        
        else:
            print(f"No angle limits found for leg {leg}, servo {servo}.")
            return angle 

    def set_servo_angle(self, leg, servo, angle, is_radian=False):
        """Set the servo angle with an offset and print it."""
        if is_radian:
            angle = math.degrees(angle)

        clipped_angle = self._clip_angle_range(leg, servo, angle)
        
        offset_key = self._get_offset_key(leg, servo)
        offset = self.offsets.get(offset_key, 0)
        offset_angle = clipped_angle + offset

        adjusted_angle = self._adjust_sign(leg, servo, offset_angle)

        servo_index = self._get_mapping(leg,servo)

        kit = 0 if leg <= 2 else 1
        self.kits[kit].servo[servo_index].angle = adjusted_angle
        print(f"Leg {leg}, Servo {servo}: Setting angle to {adjusted_angle} degrees (Angle: {clipped_angle} + Offset: {offset})")
    
    def enable_servos(self):
        print("Enable servos")
        GPIO.output(self.oe_pin, GPIO.LOW)

    def disable_servos(self):
        print("Enable servos")
        GPIO.output(self.oe_pin, GPIO.LOW)

    def calibrate(self):
        """Interactively calibrate servo offsets."""
        while True:
            leg = input("Enter leg number [0-5] (or 'q' to quit calibration mode): ")
            if leg.lower() == 'q':
                break
            
            try:
                leg = int(leg)
                if not (0 <= leg <= 5):
                    print("Invalid leg number. Please enter a number between 0 and 5.")
                    continue
            except ValueError:
                print("Invalid input. Please enter a number between 0 and 5.")
                continue

            servo = input("Enter servo number [0-2]: ")
            try:
                servo = int(servo)
                if not (0 <= servo <= 2):
                    print("Invalid servo number. Please enter a number between 0 and 2.")
                    continue
            except ValueError:
                print("Invalid input. Please enter a number between 0 and 2.")
                continue

            self.enable_servos()
            self.set_servo_angle(leg,servo,90)
            print(f"Leg {leg} servo {servo} is set to 90deg, adjust to match the angle on the robot.")

            while True:
                action = input("Increase (i), Decrease (d), Save (s) new offset, or 'q' to quit calibration mode: ")
                if action.lower() == 'q':
                    break
                offset_key = self._get_offset_key(leg, servo)
                
                if action.lower() == 'i':
                    self.offsets[offset_key] += 1
                    self.set_servo_angle(leg,servo,90 + self.offsets[offset_key])
                    print(f"Offset for leg {leg}, servo {servo} increased to {self.offsets[offset_key]}")
                elif action.lower() == 'd':
                    self.offsets[offset_key] -= 1
                    self.set_servo_angle(leg,servo,90 + self.offsets[offset_key])
                    print(f"Offset for leg {leg}, servo {servo} decreased to {self.offsets[offset_key]}")
                elif action.lower() == 's':
                    self._save_offsets()
                    print(f"Offset for leg {leg}, servo {servo} saved as {self.offsets[offset_key]}")
                else:
                    print("Invalid action. Please choose 'i', 'd', 's', or 'q'.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Control Servo Motors")

    parser.add_argument('--set_angle', nargs=3, type=float, metavar=('LEG', 'SERVO', 'ANGLE'),
                        help="Set the angle of a specific servo (leg, servo, angle in degrees)")
    parser.add_argument('--calibrate', action='store_true', help="Start calibration mode")
    parser.add_argument('--enable', action='store_true', help="Enable servos")
    parser.add_argument('--disable', action='store_true', help="Disable servos")

    args = parser.parse_args()

    driver = ServoDriver()

    if args.set_angle:
        leg, servo, angle = int(args.set_angle[0]), int(args.set_angle[1]), args.set_angle[2]
        driver.set_servo_angle(leg, servo, angle)

    if args.calibrate:
        driver.calibrate()

    if args.enable:
        driver.enable_servos()

    if args.disable:
        driver.disable_servos()
