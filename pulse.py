# -*- coding: utf-8 -*-
import time
import smbus
bus = smbus.SMBus(1)

def pulse(width): # width in msec

# now & finish in seconds from century 1970 1.Jan.
    width /= 1000.0 # width to seconds
    now = time.time()
    finish = now + width
    i = 0
    while now < finish:
        # How much bytes can we read in the interval
# uncomment the next 2 lines if u wish.
#       i += 1
#       bus.read_i2c_block_data(0x68,0x3A,8)
        now = time.time() # Go ahead
# now - finish!  overrun time in seconds
# we have not control this time
# the CPU is busy with other operations
    return i,now - finish

print "          Overrun in mikroseconds"
maxm = 0.0
bus.write_i2c_block_data(0x68,0x6B,[1]) # Disable sleep
for _ in range(18):
    i,rc = pulse(10) # n msec
    if rc > maxm:
        maxm = rc
    print " cnt=%5d %0.3f ms" % (i,float(rc) * 1000.0)
print " ==maxm=%0.3f ms" % (float(maxm) * 1000.0)

