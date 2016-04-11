#!/usr/bin/python

import smbus
import math
import time

class MPUAccelerometer:
	'Class for MPU-6050 Acceleromter + Gyroscope'
	MPU_POWER_MGMT_1 = 0x6b
	MPU_POWER_MGMT_2 = 0x6c

	GYRO_SCALE = 131.0
	ACCEL_SCALE = 16384.0
	MPU_ADR = 0x68
	bus = smbus.SMBus(1)
