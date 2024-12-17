import board
import busio

# Initialize I2C (bus 7 on Jetson Nano)
i2c = busio.I2C(board.SCL_1, board.SDA_1)

if i2c.try_lock():
    print("I2C bus is working.")
    
    try:
        while True: 
            devices = i2c.scan() 
            if devices: 
                print("Found I2C devices ataddresses:", [hex(device) for device in devices]) 
            else: 
                print("No I2C devices found.") 
    finally:        
        i2c.unlock()
else:
    print("Failed to access I2C bus.")

###

import busio
import board

DEVICE_ADDRESS = 0x60

from adafruit_bus_device.i2c_device import I2CDevice
i2c_bus = busio.I2C(board.SCL_1, board.SDA_1)
device = I2CDevice(i2c_bus, DEVICE_ADDRESS)

#from adafruit_servokit import ServoKit 
#kit = ServoKit(channels=16, i2c=i2c_bus, address=DEVICE_ADDRESS)

###
# Bus simple test https://docs.circuitpython.org/projects/busdevice/en/latest/examples.html

import busio
import board
from adafruit_bus_device.i2c_device import I2CDevice

DEVICE_ADDRESS = 0x60  # device address of DS3231 board
A_DEVICE_REGISTER = 0x06  # device id register on the DS3231 board

# The follow is for I2C communications
comm_port = busio.I2C(board.SCL_1, board.SDA_1)
device = I2CDevice(comm_port, DEVICE_ADDRESS)

with device as bus_device:
    bus_device.write(bytes([A_DEVICE_REGISTER]))
    result = bytearray(1)
    bus_device.readinto(result)

print("".join("{:02x}".format(x) for x in result))


## test with kit

import busio
import board
from adafruit_servokit import ServoKit 

# Change board.SCL and board.SDA as necessary based on detected bus
i2c_bus = busio.I2C(board.SCL_1, board.SDA_1)  # Example for bus 7
kit = ServoKit(channels=16, i2c=i2c_bus, address=0x60)