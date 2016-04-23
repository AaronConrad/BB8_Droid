#!/usr/bin python
import MPU_6050
import time

def main(args):
	mpu = MPU_6050.MPU_6050()
	mpu.wakeMPU()
	now = time.time()
	while (time.time() == now):
		# Wait until the next tick (one second)
	
	now = time.time()
	i = 0
	while (time.time() == now):
		mpu.updateAll()
		i++
		
	print "Number of iterations: %d" % i
    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))
