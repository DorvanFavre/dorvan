
## Objectives

1. Assemble Jetson on the robot #Done
2. Power Jetson #Done
3. Learn how to access jetson from host computer via wifi #Done 
4. Learn how to control jetson IO with python
5. Write a program to control one servo
6. Write a program to calibrate servos
7. Simple forward walk with CPG an IK (no RL)

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
**scp /path/to/local/file dorvan@192.168.1.100:/path/to/destination/**
/home/dorvan/Documents


### 4 Control GPIO

https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide/hardware_spec.html

Nvidia doc: (Configuration of the GPIO)
https://docs.nvidia.com/jetson/archives/r35.2.1/DeveloperGuide/text/HR/ConfiguringTheJetsonExpansionHeaders.html

jetson.GPIO (Python library to control GPIO)
https://github.com/NVIDIA/jetson-gpio
