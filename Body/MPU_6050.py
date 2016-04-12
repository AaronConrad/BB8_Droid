#!/usr/bin/python

import smbus
import math
# TODO: CHECK IF THIS IS NEEDED -- import time

class MPU_6050:
	'Class for MPU-6050 Acceleromter + Gyroscope'
	
	MPU_POWER_MGMT_1 = 0x6b
	MPU_POWER_MGMT_2 = 0x6c

	GYRO_SCALE = 131.0
	ACCEL_SCALE = 16384.0
	MPU_ADR = 0x68
	bus = smbus.SMBus(1)
	
	# Indexing for data
	X = 0
	Y = 1
	Z = 2
	# ------------------------------------------------------------------

	# Constructor for MPU_6050
	def __init__(self, timeDiff, K):
		# Important data
		self.scaledGyro = [0, 0, 0]
		self.scaledAccel = [0, 0, 0]
		self.gyroDelta = [0, 0, 0]
		self.gyroRotation = [0, 0, 0]
		self.accelRotation = [0, 0, 0]
		self.combinedRotation = [0, 0, 0]
		
		# Used in calculations to minimize gyroscopic drift
		self.gyroOffset = [0, 0, 0]
		
		# Change this to match the frequency of the calling function
		# ex: a 100Hz rate coorelates to a timeDiff = 0.01
		self.timeDiff = timeDiff
		
		# Used by complementary filter to determine influence of
		# gyroscope and accelerometer data. K is gyroscope and K1 is
		# accelerometer
		self.K = K
		self.K1 = 1 - self.K
	# ------------------------------------------------------------------
	
	# Reads the raw data from the MPU via i2c, scales it appropriately,
	# and saves the values in scaledGyro and scaledAccel.
	def retreiveData(self):
		rawGyro = bus.read_i2c_block_data(MPU_ADR, 0x43, 6)
		rawAccel = bus.read_i2c_block_data(MPU_ADR, 0x3b, 6)
		
		self.scaledGyro[X] = twosCompliment((rawGyro[0] << 8) + rawGyro[1]) / GYRO_SCALE
		self.scaledGyro[Y] = twosCompliment((rawGyro[2] << 8) + rawGyro[3]) / GYRO_SCALE
		self.scaledGyro[Z] = twosCompliment((rawGyro[4] << 8) + rawGyro[5]) / GYRO_SCALE
		
		self.scaledAccel[X] = twosCompliment((rawAccel[0] << 8) + rawAccel[1]) / ACCEL_SCALE
		self.scaledAccel[Y] = twosCompliment((rawAccel[2] << 8) + rawAccel[3]) / ACCEL_SCALE
		self.scaledAccel[Z] = twosCompliment((rawAccel[4] << 8) + rawAccel[5]) / ACCEL_SCALE
	# ------------------------------------------------------------------
	
	# Helper function for retrieveData. Computes twos-compliment on bytes
	def twosCompliment(self, val):
		if (val >= 0x8000):
			return -((65535 - val) + 1)
		else:
			return val
	# ------------------------------------------------------------------
	
	# Helper function for updateAccelRotation.
	def dist(self, a, b):
		return math.sqrt((a * a) + (b * b))
	# ------------------------------------------------------------------

	# Determine the rotation angle based solely from the accelerometer
	# data and save to accelRotation. Call retreiveData separately.
	def updateAccelRotation(self):
		self.accelRotation[X] = math.degrees(math.atan2(self.scaledAccel[Y], dist(self.scaledAccel[X], self.scaledAccel[Z])))
		self.accelRotation[X] = -math.degrees(math.atan2(self.scaledAccel[X], dist(self.scaledAccel[Y], self.scaledAccel[Z])))
		# self.accelRotation[Z] = math.degrees(math.atan2(self.scaledAccel[Z], dist(self.scaledAccel[Z], self.scaledAccel[Y])))
	# ------------------------------------------------------------------
	
	# Determine the rotation angle based solely from the gyroscope data
	# and save to gyroRotation. Call retreiveData separately.
	def updateGyroRotation(self):
		self.gyroDelta[X] = (self.scaledGyro[X] - self.gyroOffset[X]) * self.timeDiff
		self.gyroDelta[Y] = (self.scaledGyro[Y] - self.gyroOffset[Y]) * self.timeDiff
		self.gyroDelta[Z] = (self.scaledGyro[Z] - self.gyroOffset[Z]) * self.timeDiff
		
		self.gyroRotation[X] += self.gyroDelta[X]
		self.gyroRotation[Y] += self.gyroDelta[Y]
		self.gyroRotation[Z] += self.gyroDelta[Z]
	# ------------------------------------------------------------------
	
	# Determine the rotation angle based from both the gyroscope and
	# accelerometer and saves to combinedRotation. Assumes the
	# calculations for gyroscope and accelerometer have not been done
	# yet. Call retreiveData separately.
	def updateCombinedRotation(self):
		self.updateAccelRotation()
		self.updateGyroRotation()
		
		self.combinedRotation[X] = self.K * (self.combinedRotation[X] + self.gyroDelta[X]) + (self.K1 * self.accelRotation[X])
		self.combinedRotation[Y] = self.K * (self.combinedRotation[Y] + self.gyroDelta[Y]) + (self.K1 * self.accelRotation[Y])
		self.combinedRotation[Z] = self.K * (self.combinedRotation[Z] + self.gyroDelta[Z]) + (self.K1 * self.accelRotation[Z])
	# ------------------------------------------------------------------
		
	# All-in-one functions that retrieves data and does all calculations.
	def updateAll(self):
		self.retrieveData()
		self.updateCombinedRotation()
	# ------------------------------------------------------------------
	
	# Recalculates gyroscope offset based on average from 'cycle' number
	# of reads, then resets. Assumes MPU is held completely still.
	def calibrateGyro(self, cycles):
		totalOffset = [0, 0, 0]
		for i in xrange(0, cycles):
			self.retreiveData()
			totalOffset[X] += self.scaledGyro[X]
			totalOffset[Y] += self.scaledGyro[Y]
			totalOffset[Z] += self.scaledGyro[Z]
		
		# TODO: CHECK WHETHER THIS IS INT OR DOUBLE MATH, AND WHETHER IT MATTERS
		self.gyroOffset[X] = totalOffset[X] / cycles
		self.gyroOffset[Y] = totalOffset[Y] / cycles
		self.gyroOffset[Z] = totalOffset[Z] / cycles
		self.resetMPU()
	# ------------------------------------------------------------------
	
	# Clears all previous MPU data and calculations, except gyroOffset,
	# then does one read/calculation cycle to set instance variables.
	def resetMPU(self):
		self.scaledGyro = [0, 0, 0]
		self.scaledAccel = [0, 0, 0]
		self.gyroDelta = [0, 0, 0]
		self.gyroRotation = [0, 0, 0]
		self.accelRotation = [0, 0, 0]
		self.combinedRotation = [0, 0, 0]
		self.updateAll()
	# ------------------------------------------------------------------
	
	# Prints the currently kept values
	def printDebug(self):
		print "scaledGyro: %.2f, %.2f, %.2f" % (self.scaledGyro[X], self.scaledGyro[Y], self.scaledGyro[Z])
		print "scaledAccel: %.2f, %.2f, %.2f" % (self.scaledAccel[X], self.scaledAccel[Y], self.scaledAccel[Z])
		print "gyroOffset: %.2f, %.2f, %.2f" % (self.gyroOffset[X], self.gyroOffset[Y], self.gyroOffset[Z])
		print "gyroDelta: %.2f, %.2f, %.2f" % (self.gyroDelta[X], self.gyroDelta[Y], self.gyroDelta[Z])
		print "gyroRotation: %.2f, %.2f, %.2f" % (self.gyroRotation[X], self.gyroRotation[Y], self.gyroRotation[Z])
		print "accelRotation: %.2f, %.2f, %.2f" % (self.accelRotation[X], self.accelRotation[Y], self.accelRotation[Z])
		print "combinedRotation: %.2f, %.2f, %.2f" % (self.combinedRotation[X], self.combinedRotation[Y], self.combinedRotation[Z])
	# ------------------------------------------------------------------
