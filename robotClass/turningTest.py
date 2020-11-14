# Ben Chappell - Team 54

# This file is made to test the capabilities in the file turning.py.
# Eventually, a lot of the code in here may be used to create the final event loop and function.

from robot import Robot
import brickpi3
import grovepi
import time
import traceback
import turning as t

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
        frontU = 8
        rightU = 4
        leftU = 7
        rightL = 3
        leftL = 8
        hall = 7

        dropSite = 'A'
        minDPS = 30

        time.sleep(1)

        # initialize our robot value
        r = Robot(bp, left, right, steer, trailer, rightU, leftU, frontU, rightL, leftL, hall, dropSite, minDPS)
        r.diagnostics()

        rubberBandTests(r)
        #basicTurnTests(r)

        time.sleep(2)
        
        print("All done")

        bp.reset_all()
        
    except KeyboardInterrupt:
        r.stop()
        bp.reset_all()
    except Exception:
        traceback.print_exc()
        r.stop()
        bp.reset_all()

def rubberBandTests(r):
    d = 0
    activated = 0

    while True:
        r.driveMotors(r.getMinDps())
        activated = t.rubberBandTurning(r, activated)

        #results = t.rubberBandAccelTurning(r, activated, d)
        #activated = results[0]
        #d = results[1]


def basicTurnTests(r):
    speed = 30
    r.driveMotors(speed)
    
    while True:
        t.stopTurn(r)
        #t.slowTurn(r)
        #t.increaseTurnSpeed(r)
        #t.slowIncreaseTurnSpeed(r)
        #t.fastTurn(r)
        #t.fastIncreaseTurnSpeed(r)

    r.stop()