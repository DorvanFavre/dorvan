
## Objectives

1. Assemble Jetson on the robot #Done
2. Power Jetson #Done
3. Learn how to access jetson from host computer via wifi #Done 
4. Learn how to control jetson IO with python
5. Setup 12c
6. Write a program to control one servo
7. Write a program to calibrate servos
8. Simple forward walk with CPG an IK (no RL)

## Tasks

### 1. Assemble Jetson on the robot

Ok

### 2. Power Jetson

Ok

### 3. Access Jetson via SSH

Jetson username: dorvan
Jetson password: 6956

Run from host
ssh dorvan@192.168.1.100
logout

Send file
scp /path/to/local/file dorvan@192.168.1.100:/home/dorvan/Documents
/home/dorvan/Documents


### 4 Control GPIO

https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/hardware_spec.html

Nvidia doc: (Configuration of the GPIO)
https://docs.nvidia.com/jetson/archives/r35.2.1/DeveloperGuide/text/HR/ConfiguringTheJetsonExpansionHeaders.html

```
sudo /opt/nvidia/jetson-io/jetson-io.py
```

jetson.GPIO (Python library to control GPIO)
https://github.com/NVIDIA/jetson-gpio

GPIO.setmode(GPIO.BCM) : Work
GPIO.setmode(GPIO.BOARD) : not working

[[Jetson-Orin-Nano-DevKit-Carrier-Board-Specification_SP-11324-001_v1.2.pdf]]
![[Pasted image 20241211143713.png]]

### 5 Setup I2c

Tuto:
https://jetsonhacks.com/2019/07/22/jetson-nano-using-i2c/

Servokit library: https://docs.circuitpython.org/projects/servokit/en/latest/

library Adafruit PCA9685
https://github.com/adafruit/Adafruit_CircuitPython_PCA9685?tab=readme-ov-file
https://docs.circuitpython.org/projects/pca9685/en/latest/


Read register of PCA9685
i2cget -y 7 0x40 0x00

**Follow tutorial:** Jetson Nano I2C Communication Tutorial
https://www.yahboom.net/public/upload/upload-html/1683344878/07.Jetson%20Nano%20I2C%20Communication%20Tutorial.html

```
sudo apt-get update
sudo apt-get install -y i2c-tools
apt-cache policy i2c-tools
i2cdetect -l
sudo i2cdetect -y -r -a 1
```

I am using pins 3 and 5 but it detects the PCA on bus 7. 
Same issue: https://forums.developer.nvidia.com/t/setting-up-i2c-on-the-jetson-orin-nano/260141/11
![[Pasted image 20241217094617.png]]


When I disconnect cable pin 3, nothing is detected. It means that the PCA is well found on bus 7.
![[Pasted image 20241217094901.png]]

I can try to find pins for i2c 0 or 1.
Try I2C bus 0 on pin 27 and 28: https://jetsonhacks.com/nvidia-jetson-nano-j41-header-pinout/
![[Pasted image 20241217095554.png]]

First, check the configuration of the pins. 
```
sudo /opt/nvidia/jetson-io/jetson-io.py
```
![[Pasted image 20241217095439.png]]

Connect PCA on pin 27 and 28.
It detects something a bus 1, but the addresses are unexpected.
![[Pasted image 20241217100214.png]]

Two option. Option A is to try to work with bus 1. Option B is to work with bus 7.

**Option A**
Find a library for PCA.

Follow tutorial "Jetson Nano - Using I2C"
https://jetsonhacks.com/2019/07/22/jetson-nano-using-i2c/
Install ServoKit
Look for some examples at https://docs.circuitpython.org/projects/servokit/en/latest/

**Python libraries:**
[Adafruit PCA9685 Library](https://docs.circuitpython.org/projects/pca9685/en/latest/index.html#)
[Adafruit’s Bus Device](https://github.com/adafruit/Adafruit_CircuitPython_BusDevice)

busio: https://docs.circuitpython.org/en/latest/shared-bindings/busio/
board: https://learn.adafruit.com/arduino-to-circuitpython/the-board-module

Try board.
```
dir(board)
print(board.SCL)
```

Try busio. I get a "working" message. But it does not mean that it detects the PCA so I still don't know what bus it is working on.
```
import board
import busio

# Initialize I2C (bus 7 on Jetson Nano)
i2c = busio.I2C(board.SCL, board.SDA)

if i2c.try_lock():
    print("I2C bus is working.")
    i2c.unlock()
else:
    print("Failed to access I2C bus.")

```

Try code from the tuto. It is not working.
```
from adafruit_servokit import ServoKit  
kit = ServoKit(channels=16)  
kit.servo[0].angle=137  
kit.servo[1].angle=25  
quit()
```

**Try to work with** [Adafruit’s Bus Device](https://github.com/adafruit/Adafruit_CircuitPython_BusDevice) library:
```
import busio
 5import board
 6from adafruit_bus_device.i2c_device import I2CDevice
 7
 8DEVICE_ADDRESS = 0x68  # device address of DS3231 board
 9A_DEVICE_REGISTER = 0x0E  # device id register on the DS3231 board
10
11# The follow is for I2C communications
12comm_port = busio.I2C(board.SCL, board.SDA)
13device = I2CDevice(comm_port, DEVICE_ADDRESS)
```
return error message that say that there is no device at adress 0x60

however, board.SCL is GP81_I2C9_CLK, which is bus 9. i have to give the bus 1 instead.
Here is the answer:
![[Pasted image 20241217112158.png]]

Install : pip3 install --upgrade adafruit-blinka

Not fucking working.

**Option B:**
Now I have to try making a python library for PCA working on bus 7.

**Try this tuto:** Setting up I2C on the Jetson Orin Nano https://nvidia-jetson.piveral.com/jetson-orin-nano/setting-up-i2c-on-the-jetson-orin-nano/

This on2 is good!
```python
board.SCL_1 : GP13_I2C2_CLK
```


Connect to pin 27,28
Use this code:file:///home/dorvan/Documents/dorvan/Babydoll/Produits/Babybot-01/Informatic-01/code.py

![[Pasted image 20241217120719.png]]
Results:
![[Pasted image 20241217120740.png]]

Try to connect to 0x60 then:
```
import busio
import board
from adafruit_servokit import ServoKit 

# Change board.SCL and board.SDA as necessary based on detected bus
i2c_bus = busio.I2C(board.SCL_1, board.SDA_1)  # Example for bus 7
kit = ServoKit(channels=16, i2c=i2c_bus, address=0x60)
```
No errors at 0x60 :D

Try to command servo motors.

kit.servo[0].angle=90

CCCCCCCCAAAAAA MMMMARCHE !!!!!!!!!!!!!!!!!!!!!!!¨
i2c1, pin 27,28, addresse 0x60de

## 6.  Create servo driver

**6.1 Investigate ServoKit capabilities**
https://docs.circuitpython.org/projects/servokit/en/latest/
https://github.com/adafruit/Adafruit_CircuitPython_ServoKit
Useless

**6.2 Map servo number**
resultats dans mon cahier

6.3 OE
pins: https://github.com/NVIDIA/jetson-gpio/blob/master/lib/python/Jetson/GPIO/gpio_pin_data.py

GPIO.setmode(GPIO.BCM)
output_pin = 18  # BCM pin 18, BOARD pin 12

Not working alongside 12c

mode is TEGRA_SOC
Therfoe use "module pin name" in the 40 pin header table:
![[Pasted image 20241218160631.png]]
oe_pin = 'GP122'
GPIO.setup(oe_pin, GPIO.OUT, initial=GPIO.HIGH)


**6.3 Write python interface**

 create a python class named "ServoDriver" that has a method that take as input the leg number [0,5], the servo number [0,2] and the angle  (default degree but can be specified as radian), return void. This method would print the angle with an offset for this specific servo/leg. the offset are specified in a separated file. There is another method named "calibrate". This method asks for a leg and servo number or to quit calibration mode. When servo and leg are given,  it asks ether to increase, decrease or save the new offset.

**6.4 Calibration**


