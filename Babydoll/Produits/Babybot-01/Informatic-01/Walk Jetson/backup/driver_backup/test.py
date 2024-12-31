

from servo_driver import ServoDriver
import time
import numpy as np
sd = ServoDriver(verbose=0)


sd.enable_servos()

for l in range(6):
    for s in range(3):
        sd.set_servo_angle(l,s,90)
        
time.sleep(2)

for l in range(6):
    for s in range(3):
        sd.set_servo_angle(l,s,40)
time.sleep(2)


for j in range(3):
    for i in range(40,140,1):
        for l in range(6):
            for s in range(3):
                sd.set_servo_angle(l,s,i)
                
        time.sleep(0.02)

    for i in range(140,40,-1):
        for l in range(6):
            for s in range(3):
                sd.set_servo_angle(l,s,i)
                
        time.sleep(0.02)
