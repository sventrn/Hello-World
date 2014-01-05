#!/usr/bin/python  -tt
# -*- coding: utf-8 -*-
# MPU-6050 Read Gyro & Accelerometer data.
# 07. Dec. 2013. 20:00 TS. OK!
# https://github.com/TKJElectronics/KalmanFilter/blob/master/ Kalman.h#L1-L16
# Original: Kristian Lauszus
# Ported code from "C" to python by Sven Turnau
# for Raspberry Pi
# e-mail - sventurnau@yahoo.com

import smbus
import Kalman
import time
import math
import os,sys
bus = smbus.SMBus(1)
RAD_TO_DEG = 57.295779513082320876798154814105
PI         = 3.1415926535897932384626433832795

kalman = Kalman.Kalman()  # Create the Kalman instances

def setXGyroOffset(offs):
    bus.write_i2c_block_data(0x68,0x13,[offs/256])
    bus.write_i2c_block_data(0x68,0x14,[offs & 0xFF])

def setYGyroOffset(offs):
    bus.write_i2c_block_data(0x68,0x15,[offs/256])
    bus.write_i2c_block_data(0x68,0x16,[offs & 0xFF])

def setZGyroOffset(offs):
    bus.write_i2c_block_data(0x68,0x17,[offs/256])
    bus.write_i2c_block_data(0x68,0x18,[offs & 0xFF])

def Gyro_Calibration(cnt): # Amount for sampling

    Zarr = []  # List for collecting data.
    Zmin = 0.0 # set min,max on the fly
    Zmax = sys.float_info.max # 1.7976931348623157e+308
    for x in range(cnt):
        GCal = read_bus_data(0x47,2)
        Zval = (GCal[0]<<8)|GCal[1]
        if Zval > Zmin: Zmin = Zval
        if Zval < Zmax: Zmax = Zval
        Zarr.append(Zval)
    Zsum = sum(Zarr)
    return Zsum/cnt

def Calibrate_Gyros():

    print ("     Calibrating Gyros")
    GYRO_XOUT_OFFSET_1000SUM = 0
    GYRO_YOUT_OFFSET_1000SUM = 0
    GYRO_ZOUT_OFFSET_1000SUM = 0

#   for x in range(1000):
    for x in range(200):

        GCal = read_bus_data(0x43,6)

        GYRO_XOUT_OFFSET_1000SUM +=\
 (GCal[0]<<8)|GCal[1]
        GYRO_YOUT_OFFSET_1000SUM +=\
 (GCal[2]<<8)|GCal[3]
        GYRO_ZOUT_OFFSET_1000SUM +=\
 (GCal[4]<<8)|GCal[5]
        time.sleep(0.001) # 1ms
        GYRO_XOUT_OFFSET = GYRO_XOUT_OFFSET_1000SUM/200
        GYRO_YOUT_OFFSET = GYRO_YOUT_OFFSET_1000SUM/200
        GYRO_ZOUT_OFFSET = GYRO_ZOUT_OFFSET_1000SUM/200

# http://www.i2cdevlib.com/forums/topic/10-why-is-the-dmp-yaw-stable/
        setXGyroOffset(220)
        setYGyroOffset(76)
        setZGyroOffset(-85)

    print("Gyro X offset sum: %8ld Gyro X offset: %6d"\
            % (GYRO_XOUT_OFFSET_1000SUM, GYRO_XOUT_OFFSET) )
    print("Gyro Y offset sum: %8ld Gyro Y offset: %6d"\
            % (GYRO_YOUT_OFFSET_1000SUM, GYRO_YOUT_OFFSET) )
    print("Gyro Z offset sum: %8ld Gyro Z offset: %6d"\
            % (GYRO_ZOUT_OFFSET_1000SUM, GYRO_ZOUT_OFFSET) )

def read_bus_data(rrr,nr): # Burst mode
    try:
      ret =bus.read_i2c_block_data(0x68,rrr,nr)
#     ret = [0x68,2,3,4,5,6]
    except: sys.exit("Bus READ Error!")
    return ret

def wait_bus_data(): # Busy wait (Poll the Bus).
# check data ready bit in MPU-6050
  while True: # Poll
    try:
      F = int(read_bus_data(0x3A,1)[0]) & 0x01
#     F = int(bus.read_i2c_block_data(0x68,0x3A,1)[0])\
#       & 0x01
      if F == 0x01: # 1 ==> DATA_RDY_INT{errupt} was
        break     # DATA_RDY_INT [data is ready to read.]
      else:
        continue
    except: sys.exit("= Bus Poll Error+++++")

def setup_Bus():

       #  IMU Data
    global     gyroXangle
    global     gyroYangle
    global     compAngleX
    global     compAngleY
    global     timer # time.time()#
    global     kalmanTimer
    kalmanTimer = time.time()
    i2cData = read_bus_data(0x75,1) #// Read "WHO_AM_I" reg-r
    if(i2cData[0] == 0x68):
      pass
      print("                I_AM an MPU-6050!\n")
    else: sys.exit("I_AM not MPU-6050!!")
    bus.write_i2c_block_data(0x68,0x6b,[1]) # NB! Disable     sleep mode
    i2cData = [7,0,0,0]
    i2cData[0] = 7 # Set the Sample rate up 1000Hz - 8kHz/(7+ 1)=1000Hz
    i2cData[1] = 0x00 # Disable FSYNC and set 260Hz Acc       filtering,\
                 # 256Hz Gyro filtering, 8 kHz sampling
    i2cData[2] = 0x00 # Set Gyro Full Scale Range to +/-      250deg/s
    i2cData[3] = 0x00 # Set Accelerometr Full Scale
     #Range to +/-2g
    # values
#   print "        Wait for sensor to stabilize\n"
    bus.write_i2c_block_data(0x68,0x38,[1]) # DATA_RDY_EN
    bus.write_i2c_block_data(0x68,0x19,i2cData) # set init
    time.sleep(0.01)#; #// Wait for sensor to stabilize
     # Set kalman and gyro starting angle #/
    wait_bus_data()  # Busy WAIT
    i2cData = read_bus_data(0x3D,4) # READ 4 Bytes
    accY = ((i2cData[0] << 8) | i2cData[1])
    accZ = ((i2cData[2] << 8) | i2cData[2])

# atan2 outputs the value of -π to π (radians) - see     http://en.wikipedia.org/wiki/Atan2
# We then convert it to 0 to 2π and then from radians to degrees
###### DBG 2 lines commented
    accAngle = 0
#   accAngle = (math.atan2(accY - cfg.accYzero,\
#   (double)accZ - cfg.accZzero) + PI) * RAD_TO_DEG

    kalman.setAngle(accAngle) # Set starting angle
    pitch = accAngle
    gyroAngle = accAngle

#i Find gyro zero value GROUND ZERO!
#   calibrateGyro()


def loop(tak):

  global     gyroXangle
  global     gyroYangle
  global     compAngleX
  global     compAngleY
  global     timer # time.time()
  global     kalmanTimer

  # Calculate pitch
# check data ready bit in MPU-605

  while True: # Poll
    try:
      F = int(bus.read_i2c_block_data(0x68,0x3A,1)[0])\
         & 0x01
      if F == 0x01: # 1 ==> DATA_RDY_INT{errupt} was
        break     # DATA_RDY_INT [data is ready to read.]
      else:
        continue
    except: sys.exit("= Bus Poll Error+++++")


  i2cBuffer = [0,1,2,3,4,5,6,7] # For Debugging
  i2cBuffer = read_bus_data(0x3D,8) # READ all of the data

  accY = ((i2cBuffer[0] << 8) | i2cBuffer[1])
  accZ = ((i2cBuffer[2] << 8) | i2cBuffer[3])
  gyroX = ((i2cBuffer[6] << 8) | i2cBuffer[7])

# atan2 outputs the value of -π to π (radians) - see
       #http://en.wikipedia.org/wiki/Atan2
# We then convert it to 0 to 2π and then from \
       #radians to   degrees
  accAngle = (math.atan2(accY - \
#   cfg.accYzero, accZ - cfg.accZzero) + PI) * RAD_TO_DEG
    accY, accZ - accZ) + PI) * RAD_TO_DEG
# timer = micros()
  timer = time.time()

# This fixes the 0-360 transition problem when the         accelerometer angle jumps between 0 and 360 degrees
  if (((accAngle  <  90) and (pitch > 270)) or \
      ((accAngle  > 270) and (pitch <  90))):
      kalman.setAngle(accAngle)
      pitch = accAngle
      gyroAngle = accAngle
  else:
    #        Convert to deg/s
#   gyroRate = (gyroX - gyroXzero) / 131.0
#  Why 131.0
# http://stackoverflow.com/questions/12298481/use-a-gyro-accelerometer-with-arduino
    gyroRate = (gyroX - gyroX) / 131.0 # ??
    dt = (timer - kalmanTimer) / 1000000.0
    #     Gyro angle is only used for debugging
    #gyroAngle += gyroRate * dt
    gyroAngle = 0 # BRRR DBG Crack
  if ((gyroAngle < 0) or (gyroAngle > 360)):
     gyroAngle = pitch # Reset the gyro angle\
        # when it has  drifted too much
  # Calculate the angle using a Kalman filter
  pitch = kalman.getAngle(accAngle, gyroRate, dt)

  kalmanTimer = timer
  '''
  print "%6.1f" % round(pitch,1)
  '''

def main():
#   if len(sys.argv()) < 3:
    if len(sys.argv) < 3:
        sys.exit(" %s nrofloops Lower_limit")
    setup_Bus()
    tik = 0
#while True:
#nr = input("Get nr for loop count\n")
    try:
        nr = int(sys.argv[1])
        lim = float(sys.argv[2])/1000 # dt > lim (ms) ## DeltaT
    except:
        pass
    print "  Miss cnt time for %d loops (ms)" % nr
    Tmaximorum = 0.0
    for i in range(7):
        Tmax = 0.0 # Tmin & Tmax are computed on the fly
        Tmin = sys.float_info.max # 1.7976931348623157e+308
        Tarrmax = [0.0] # List for Time max'm
        Tarrmin = [sys.float_info] # List for Time min'm
        cnt = 0
        tsav = t = time.time()
        for _ in range(nr):
            tik += 1
            loop(tik)
            # td time difference
            dt = (time.time() - t)
            if dt > lim: # GT lim
                cnt += 1
            if dt < Tmin:
                Tmin = dt
                Tarrmin.append(Tmin)
            elif Tmax < dt:
                if dt > Tmaximorum:
                    Tmaximorum = dt
                    isav = i + 1
                Tmax = dt
                Tarrmax.append(Tmax)
            t = time.time()
#           Tmin = min(Tarrmin)
#           Tmax = max(Tarrmax)
        print "cnt=%2d Avg=%4.2f ms;Min=%5.2f ms|\
Max_DT=%5.2f ms|" %\
            (cnt,round((time.time()-tsav)*1000.0/nr,2),\
            round(Tmin*1000.0,2),\
            round(Tmax*1000.0,2))
    print "\t\t     isav =%3d  Tmaximorum =%5.2f ms"\
            % (isav,Tmaximorum*1000)

#       out(Tarr)

def out(L):
#   print L
#   return
    for l in L:
        print " %6.3g" % (l),

if __name__ == '__main__':
    main()
#   Calibrate_Gyros()
#   res = Gyro_Calibration(int(sys.argv[1]))
#   print "\t       Average of my ZGyro DRIFT = ",
#   print "%5.0f" % res
#   sys.exit("Good Boy")
#   main()

'''
==========  useful links =============
http://www.instructables.com/id/Accelerometer-Gyro-Tutorial/step1/The-Accelerometer/

'''
