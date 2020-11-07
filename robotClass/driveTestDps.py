# Ben Chappell

# File to test robot.py and how effective it is

from robot import Robot
import brickpi3
import grovepi
import time
import traceback

def main():
    try:
        time.sleep(0)
        
        # Initlailize our brickpi object
        bp = brickpi3.BrickPi3()

        # Initialize ports for motors
        right = bp.PORT_B
        left = bp.PORT_C
        steer = bp.PORT_D
        trailer = bp.PORT_A

        # Initalize sensor ports
        frontU = 3
        rightU = 4
        leftU = 8
        rightL = 5
        leftL = 6
        hall = 7

        time.sleep(1)

        # initialize our robot value
        r = Robot(bp, left, right, steer, trailer, rightU, leftU, frontU, rightL, leftL, hall)
        r.diagnostics()
        #avoid(r)
        

        time.sleep(2)
        
        print("All done")

        bp.reset_all()
        
    except KeyboardInterrupt:
        bp.reset_all()
    except Exception:
        traceback.print_exc()
        bp.reset_all()