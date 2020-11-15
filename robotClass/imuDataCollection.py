# Ben Chappell - Team 54
# This file gets a bunch of data regarding the magnitude of the magnetic feild surrouding the robot
# Uses the imu to calculate this

import time
from robot import Robot
import sys
import brickpi3

def main():
    try:
        i = 0 # Stores the number of iterations for the data collection.
        maxIter = 2500
        fid = open("passingrobotovermagnet.txt", 'w')
        testList = []

        # Initlailize our brickpi object
        bp = brickpi3.BrickPi3()

        # Initialize ports for motors
        right = bp.PORT_B
        left = bp.PORT_C
        steer = bp.PORT_D
        trailer = bp.PORT_A

        # Initalize sensor ports
        frontU = 8
        rightU = 4
        leftU = 7
        rightL = 3
        leftL = 8
        hall = 7

        dropSite = 'A'
        mindps = 30
        
        r = Robot(bp, left, right, steer, trailer, rightU, leftU, frontU, rightL, leftL, hall, dropSite, mindps) # Initialize the robot with all 0 values since we're only reading the imu data
        
        while i < maxIter:
            r.getMagMagnReading()
            print (r.getMagMagn())

            if r.getMagMagn() != 0:
                fid.write(str(r.getMagMagn()) + "\n")
                testList.append(r.getMagMagn())
            else:
                i -= 1

            #if i % 250 == 0:
             #   s = input("press enter once ready for new positioning")

            time.sleep(.1)
            i += 1

        print (len(testList))

    except KeyboardInterrupt:
        sys.exit()
        
            


if __name__ == "__main__":
    main()
