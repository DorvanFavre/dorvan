
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
scp /path/to/local/file dorvan@192.168.1.100:/path/to/destination/
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

