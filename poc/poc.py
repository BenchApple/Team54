# Ben Chappell - Team 54

# File to demonstrate our competency in the code

from robot import Robot
import brickpi3
import grovepi
import time
import traceback

def main():
    bp = brickpi3.BrickPi3()

    try:
        time.sleep(2)

        # Initialize motor ports
        rightm = bp.PORT_B
        leftm = bp.PORT_C
        steer = bp.PORT_D
        trailer = bp.PORT_A

        # Initalize sensor ports. Remember to check these before running anything.
        frontU = 3
        rightU = 4
        leftU = 8
        rightLine = 2
        leftLine = 7
        hall = 6

        time.sleep(1)

        # initialize our robot object
        r = Robot(bp, leftm, rightm, steer, trailer, rightU, leftU, frontU, rightLine, leftLine, hall)
        r.diagnostics()

        # hill(r)

        # drumstick(r)

        # curvedLine(r)

        # deliverCargo(r)

    except KeyboardInterrupt:
        bp.reset_all()
    except Exception:
        traceback.print_exc()
        bp.reset_all()

def hill(r):
    hillPower = 55
    hillTime = 10
    accelTime = 2

    r.accelerate(hillPower, accelTime)

    time.sleep(hillTime)

    r.stop(1)

def drumstick(r):
    r.setCubeCount(1)

    # Im going to write this as if there is no line present, so we're just going to go straight here.
    p = 55
    t = 14
    aT = 2

    r.accelerate(p, aT)

    time.sleep(t)

    r.stop(1)

def curvedLine(r):
    maxPower = 55
    accelTime = .5
    lineCountBeforeSlowdown = 45

    r.accelerate(maxPower, accelTime)

    while True: # Need to come up with a proper stop conditino for this
        r.getLineReadings()
        r.reactToLineFinders()

        lineCount = max(r.getRightLineCount(), r.getLeftLineCount())
        # If either of the line finder counts are sufficiently high enough, lower the power proportionally
        if lineCount >= lineCountBeforeSlowdown:
            r.accelerate(maxPower - int(.2 * (lineCount - lineCountBeforeSlowdown)), .1)
        else:
            r.accelerate(maxPower, .1)

        time.sleep(.02)

    r.stop(.1)

def deliverCargo(r):
    iterGuess = 20000
    i = 0
    maxPower = 55
    
    lineCountBeforeSlowdown = 45

    r.accelerate(maxPower, .5)

    while i < iterGuess: # Run this loop iterGuess times, then drop off the cargo
        r.getLineReadings()
        r.reactToLineFinders()

        lineCount = max(r.getRightLineCount(), r.getLeftLineCount())
        # If either of the line finder counts are sufficiently high enough, lower the power proportionally
        if lineCount >= lineCountBeforeSlowdown:
            r.accelerate(maxPower - int(.2 * (lineCount - lineCountBeforeSlowdown)), .1)
        else:
            r.accelerate(maxPower, .1)

        time.sleep(.02)
        i += 1

    r.stop(.1)
    r.dismount()
    
    time.sleep(1)

    r.stop()

        