#!/usr/bin/env python

import time
import pigpio

SERVO = 4

servos = pigpio.pi()

pulse = 1500
while True:
	var = input("Enter 1000-2000: ")
	if not var:
		continue
	
	if var < 1000:
		continue
	
	if var > 2000:
		continue
		
	pulse = var
	servos.set_servo_pulsewidth(SERVO, pulse)
	time.sleep(.05)
