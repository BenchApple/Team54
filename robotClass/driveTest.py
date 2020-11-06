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
        rightLine = 2
        leftLine = 5
        hall = 7

        time.sleep(1)

        # initialize our robot value
        r = Robot(bp, left, right, steer, trailer, rightU, leftU, frontU, rightLine, leftLine, hall)
        r.diagnostics()
        testLines(r)
        

        time.sleep(2)
        
        print("All done")

        bp.reset_all()
        
    except KeyboardInterrupt:
        bp.reset_all()
    except Exception as e:
        traceback.print_exc()
        bp.reset_all()

def testLines(r):
    while True:
        r.getLineReadings()
        print (r.getRightLineReading())
        print (r.getLeftLineReading())
        print(" ")
        r.reactToLineFinders()
        time.sleep(.25)

def testHall(r):
    while True:
        r.getHallReading()
        print (r.getHallStatus())
        time.sleep(.1)

def testUltrasonics(r):
    while True:
        r.getUltrasonics()

        print(r.getLeftDist(), r.getFrontDist(), r.getRightDist())

def wiggle(r):
    r.accelerate(60, 1)
    time.sleep(1)

    r.rotateAxle(150)

    time.sleep(1)

    r.setStraight()

    time.sleep(1)

    r.rotateAxle(-150)

    time.sleep(1)

    r.setStraight()

    time.sleep(2)

    r.stop(1)

def circle(r):
    power = 0
    r.rotateAxle(r.getMaxTurn() - 5)
    r.accelerate(50, 2)
    time.sleep(4)
    r.setStraight()
    r.stop(1)

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

# TODO finish writing this function with the three ultrasonics. avoidDeprecated will work when adjusted to new system.
def avoid(r):
    maxPower = 30
    slowDist = 60
    stopDist = 20
    degreeChange = 50
    r.accelerate(maxPower, 1)

    
    while r.getFrontDist() > stopDist:
        r.getUrgency()
        r.turnFromUrgency()

    r.stop(.1)
            
        
    
if __name__ == "__main__":
    main()
