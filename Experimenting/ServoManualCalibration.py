#!/usr/bin/env/python

import time
import pigpio

SERVO = 4
servos = pigpio.pi()

for i in range(1,20):
	servos.set_servo_pulsewidth(SERVO, 1000)
	time.sleep(.06)
	servos.set_servo_pulsewidth(SERVO, 1500)
	time.sleep(.1)
