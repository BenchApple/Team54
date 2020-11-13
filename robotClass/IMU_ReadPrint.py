from MPU9250 import MPU9250
import sys
import time
from math import sqrt

# This sample code reads the data from the acceleration, gyro, and magnetic
# sensors, breaks each into their components, and prints one of the results
# to the screen.

# For this code to run, the MPU9250.py library MUST be located on the Pi.

# Initialize the MPU9250 library
mpu9250 = MPU9250()


try:
        while True:
            # Performing the actual IMU data reads
            accel = mpu9250.readAccel()
            gyro = mpu9250.readGyro()
            mag = mpu9250.readMagnet()

            # Breaking each data read into X, Y, and Z components
            accel_x = accel['x']
            accel_y = accel['y']
            accel_z = accel['z']
            
            gyro_x = gyro['x']
            gyro_y = gyro['y']
            gyro_z = gyro['z']
            
            mag_x = mag['x']
            mag_y = mag['y']
            mag_z = mag['z']

            # Un-comment the desired print output.
            #print(accel)
            #print(gyro)
            print(sqrt((mag_x * mag_x) + (mag_y * mag_y) + (mag_z * mag_z)))
            
            time.sleep(0.25)

            s = input("rotate bot then hit enter")
            


except KeyboardInterrupt:
        sys.exit()

