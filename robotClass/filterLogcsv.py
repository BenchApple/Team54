from MPU9250 import MPU9250
import numpy as np
import sys
import smbus
import time

from IMUFilters import AvgCali
from IMUFilters import genWindow
from IMUFilters import WindowFilterDyn
from IMUFilters import KalmanFilter
from IMUFilters import FindSTD
from IMUFilters import InvGaussFilter

# NOTE: This is sample code that reads acceleration, gyro, and magnetic data
# from the GrovePi IMU, filters it based on user parameters, and writes it to
# a CSV file.


mpu9250 = MPU9250()
feele = open("dataSet.csv","w")

#Parameters
width=1
depth=100
dly=0.01
adv = True
#/////////

accelx=genWindow(width,0)#Can play with width to adjust system
accely=genWindow(width,0)
accelz=genWindow(width,0)
gyrox=genWindow(width,0)#Can play with width to adjust system
gyroy=genWindow(width,0)
gyroz=genWindow(width,0)
magx=genWindow(width,0)#Can play with width to adjust system
magy=genWindow(width,0)
magz=genWindow(width,0)
flter=[[0.7,1.0],[0.7,1.0],[0.7,1.0],[0.7,1.0],[0.7,1.0],[0.7,1.0],[5,2],[5,2],[5,2]]
# [r,q]Will need to play with each filter value
# 
# A note on Kalman filter parameters: Each of these parameters represents the process and system noise respectively.
# In order to adjust the output of the Kalman filter, each of these parameters can be modified.  Do not set these
# parameters to zero, because a Kalman filter actually needs a little bit of noise, or it becomes unstable and 
# effectively useless.

# System Noise:  Setting the system noise to a high value will make the filter less responsive to subtle changes in
# the environment.  Practically speaking, if you tell your filter that the system is going to be very noisy, it will
# likely assume small changes are just noise.  Telling your filter that the system will have a low amount of noise
# will make it more "aggressive".

# Process Noise:  Process noise is a "natural noise" that grows over time proportional to how often you are making 
# measurements.  What it attempts to model is the fact that the states change between measurements, so there is an
# additional uncertainty on top of any noise in the sensors.  There are advanced methods for calculating good estimates
# process noise, but generally for this course, guessing and testing should be a decent method.

# Calibration and Filter Setup

biases=AvgCali(mpu9250,depth,dly)
state=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0,0,0,0,0,0,0,0,0]]#Estimated error (p) and measurement state (x) 
out=[0,0,0,0,0,0,0,0,0]
std=FindSTD(biases,mpu9250,dly)
pick = 1 #1 uses window filter, anything else uses Kalman
count = 3 #Number of standard deviations used for filtering

t0=time.time()

try:
        while True:
                if pick == 1:
                    # Data Read
                        accel = mpu9250.readAccel()
                        gyro = mpu9250.readGyro()
                        mag = mpu9250.readMagnet()
	
                        accelx=WindowFilterDyn(accelx,dly,InvGaussFilter(adv,accel['x'], biases[0],std[0],count))
                        accely=WindowFilterDyn(accely,dly,InvGaussFilter(adv,accel['y'], biases[1],std[1],count))
                        accelz=WindowFilterDyn(accelz,dly,InvGaussFilter(adv,accel['z'], biases[2],std[2],count))
                        gyrox=WindowFilterDyn(gyrox,dly,InvGaussFilter(adv,gyro['x'], biases[3],std[3],count))
                        gyroy=WindowFilterDyn(gyroy,dly,InvGaussFilter(adv,gyro['y'], biases[4],std[4],count))
                        gyroz=WindowFilterDyn(gyroz,dly,InvGaussFilter(adv,gyro['z'], biases[5],std[5],count))
                        magx=WindowFilterDyn(magx,dly,InvGaussFilter(adv,mag['x'], biases[6],std[6],count))
                        magy=WindowFilterDyn(magy,dly,InvGaussFilter(adv,mag['y'], biases[7],std[7],count))
                        magz=WindowFilterDyn(magz,dly,InvGaussFilter(adv,mag['z'], biases[8],std[8],count))
                        out[0]=accelx[0]
                        out[1]=accely[0]
                        out[2]=accelz[0]
                        out[3]=gyrox[0]
                        out[4]=gyroy[0]
                        out[5]=gyroz[0]
                        out[6]=magx[0]
                        out[7]=magy[0]
                        out[8]=magz[0]
                else:
                        state=KalmanFilter(mpu9250,state,flter,dly)
                        out[0]=InvGaussFilter(adv,state[1][0], biases[0],std[0],count)
                        out[1]=InvGaussFilter(adv,state[1][1], biases[1],std[1],count)
                        out[2]=InvGaussFilter(adv,state[1][2], biases[2],std[2],count)
                        out[3]=InvGaussFilter(adv,state[1][3], biases[3],std[3],count)
                        out[4]=InvGaussFilter(adv,state[1][4], biases[4],std[4],count)
                        out[5]=InvGaussFilter(adv,state[1][5], biases[5],std[5],count)
                        out[6]=InvGaussFilter(adv,state[1][6], biases[6],std[6],count)
                        out[7]=InvGaussFilter(adv,state[1][7], biases[7],std[7],count)
                        out[8]=InvGaussFilter(adv,state[1][8], biases[8],std[8],count)
                
                feele.write("acell,")
                feele.write(str(out[0]))
                feele.write (",")
                feele.write(str(out[1]))
                feele.write (",")
                feele.write(str(out[2]))
                feele.write(",")
                
                feele.write ("gyro,",)
                feele.write(str(out[3]))
                feele.write (",",)
                feele.write(str(out[4]))
                feele.write (",",)
                feele.write(str(out[5]))
                feele.write(",")

                feele.write ("mag,",)
                feele.write(str(out[6]))
                feele.write (",",)
                feele.write(str(out[7]))
                feele.write (",",)
                feele.write(str(out[8]))
                feele.write(",")

                t=time.time()
                delt=t-t0
                feele.write("time,")
                feele.write(str(delt))
                feele.write("\n")
                t0=time.time()

# Integrate a wait to allow for new data to be available
                time.sleep(0.5)

except KeyboardInterrupt:
        feele.close()
        sys.exit()

