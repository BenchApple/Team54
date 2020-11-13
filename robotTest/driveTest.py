# Ben Chappell

# File to test the effectiveness of the drive.py file

import drive as drive
import sensors as s
import brickpi3
import grovepi
import time
import traceback

# Initlailize our brickpi object
bp = brickpi3.BrickPi3()

def main():
    try:
        time.sleep(3)
        
        # Initialize ports for motors
        right = bp.PORT_B
        left = bp.PORT_C
        steer = bp.PORT_D

        # Initalize sensor ports
        frontU = 3
        rightU = 4
        leftU = 8

        drive.setStraight(bp, steer, 1)
        #avoid(bp, right, left, steer, front)

        time.sleep(3)
        
        print("All done")

        bp.reset_all()
        
    except KeyboardInterrupt:
        bp.reset_all()
    except Exception:
        traceback.print_exc()
        bp.reset_all()

def wiggle(bp, right, left, steer):
    pos = bp.get_motor_encoder(steer)
    pos = drive.setStraight(bp, steer, pos)

    power = 0
    power = drive.accelerate(bp, right, left, power, 45, 2)
    time.sleep(1)

    pos = drive.rotateAxle(bp, steer, 150, pos)

    time.sleep(1)

    pos = drive.setStraight(bp, steer, pos)

    time.sleep(1)

    pos = drive.rotateAxle(bp, steer, -150, pos)

    time.sleep(1)

    pos = drive.setStraight(bp, steer, pos)

    time.sleep(2)

    power = drive.stop(bp, right, left, power, 1)

def circle(bp, right, left, steer):
    pos = bp.get_motor_encoder(steer)
    pos = drive.setStraight(bp, steer, pos)

    power = 0
    pos = drive.rotateAxle(bp, steer, 245, pos)
    power = drive.accelerate(bp, right, left, power, 50, 2)
    time.sleep(4)
    pos = drive.setStraight(bp, steer, pos)
    power = drive.stop(bp, right, left, power, 1)

def wallStop(bp, right, left, steer, frontSonic):
    pos = bp.get_motor_encoder(steer)
    pos = drive.setStraight(bp, steer, pos)

    power = 0
    power = drive.accelerate(bp, right, left, power, 60, 2)
    d = s.getUltrasonic(frontSonic)

    while (d) > 40:
        print(d)
        time.sleep(.02)
        d = s.getUltrasonic(frontSonic)

    power = drive.stop(bp, right, left, power, .1)

def wallSlow(bp, right, left, steer, frontSonic):
    pos = bp.get_motor_encoder(steer)
    pos = drive.setStraight(bp, steer, pos)

    power = 0
    maxPower = 60
    slowD = 40
    power = drive.accelerate(bp, right, left, power, maxPower, 2)
    d = s.getUltrasonic(frontSonic)

    while power >= 8:
        print(d)
        time.sleep(.01)
        d = s.getUltrasonic(frontSonic)

        if d <= slowD:
            newPower = maxPower - (slowD - d) 
            power = drive.accelerate(bp, right, left, power, newPower, .1)
        elif d <= 10:
            power = drive.stop(bp, right, left, power, .1)

    power = drive.stop(bp, right, left, power, .1)

# DEPRECATED AS OF ADDING ADDITIONAL ULTRASONICS
def avoidDeprecated(bp, right, left, steer, frontSonic):
    pos = bp.get_motor_encoder(steer)
    pos = drive.setStraight(bp, steer, pos)

    power = 0
    maxPower = 40
    slowDist = 60
    stopDist = 15
    degreeChange = 50
    power = drive.accelerate(bp, right, left, power, maxPower, 1)
    dist = s.getUltrasonic(frontSonic)

    while dist > 15:
        newDir = s.getSurroundings(bp, frontSonic, steer, pos, degreeChange)
        pos = drive.rotateAxle(bp, steer, degreeChange * newDir, pos, degreesps = 360)
        
        print(dist)
        time.sleep(.01)
        dist = s.getUltrasonic(frontSonic)

        if dist <= slowDist:
            newPower = maxPower - int(.5 * (slowDist - dist))
            power = drive.accelerate(bp, right, left, power, newPower, .1)

def avoid(bp, rightm, leftm, steer, front, rightu, leftu):
    pos = pos = bp.get_motor_encoder(steer)
    pos = drive.setStraight(bp, steer, pos)

    power = 0
    maxPower = 40
    slowDist = 60
    stopDist = 15
    degreeChange = 50
    power = drive.accelerate(bp, right, left, power, maxPower, 1)

    fDist = s.getUltrasonic(frontu)
    rDist = s.getUltrasonic(rightu)
    lDist = s.getUltrasonic(leftu)

    #while fdist
            
        
    
if __name__ == "__main__":
    main()
