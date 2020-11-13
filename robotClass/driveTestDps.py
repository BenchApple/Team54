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
        frontU = 8
        rightU = 4
        leftU = 7
        rightL = 3
        leftL = 8
        hall = 7

        time.sleep(1)

        # initialize our robot value
        r = Robot(bp, left, right, steer, trailer, rightU, leftU, frontU, rightL, leftL, hall)
        r.diagnostics()
        #r.rotateAxle(-r.getMaxTurn())
        #turnTest(r)
        #r.dismount()
        #lineTurn(r)
        #ultrasonicTest(r)
        #rightLineCalib(r)
        #leftLineCalib(r)
        #schmoovin(r)

        time.sleep(2)
        
        print("All done")

        bp.reset_all()
        
    except KeyboardInterrupt:
        bp.reset_all()
    except Exception:
        traceback.print_exc()
        bp.reset_all()

# Experimentally, without trailer dps = 30 to 35 power is the best speed for turning, we have yet to determine a top speed.
# TODO test turning speed with trailer in order to determine optimal speed. Current one 40

def turnTest(r):
    #for i in range(0, -r.getMaxTurn() - 1, -10):
     #   r.rotateAxle(i)
      #  time.sleep(.1)
    r.rotateAxle(-r.getMaxTurn())

    time.sleep(1)

    r.setStraight()

    time.sleep(1)

    r.rotateAxle(r.getMaxTurn())

    time.sleep(1)

    r.setStraight()

    time.sleep(1)

def lineTurn(r):
    speed = 30
    r.driveMotors(speed)

    maxIter = 5000
    i = 0
    tStep = .02
    turning = True
    baseLineTurn = 5 # Keeps track of the basic line turn value
    lineDetectionStreak = 0 # Keeps track of the number of iterations in a row where a line was detected.

    while (True):
        r.driveMotors(speed) # Start the motors up to the designated speed
        r.getLineReadings() # Get the readings from the line finders
        r.setLineTurn(baseLineTurn) # Set the line turn of the robot object to the base value
        turning = True
        lineDetectionStreak = 0

        while ((r.getRightLineReading() or r.getLeftLineReading()) and turning):
            #r.driveMotors(speed - 5)
            r.getLineReadings()
            print (r.getRightLineReading(), r.getLeftLineReading())
            turning = r.reactToLineFinders()

            lineDetectionStreak += 1 # Increase the line turn based on the number of times in a row we detected a line.
            r.setLineTurn(baseLineTurn + lineDetectionStreak)

        if r.getPos() != 0:
            r.driveMotors(speed-5)

        print(" still running ")
        #time.sleep(tStep)

        i += 1

    r.stop()

def ultrasonicTest(r):
    while True:
        r.getUltrasonics()
        print(r.getFrontDist())

def rightLineCalib(r):
    while True:
        r.getLineReadings()
        print(r.getRightLineReading())
        time.sleep(.2)

def leftLineCalib(r):
    while True:
        r.getLineReadings()
        print(r.getLeftLineReading())
        time.sleep(.2)

def speedTrials(r):
    for s in range(180, 201, 10):
        print (s)
        
        st = input("Press the buttun to start the next trial")
        
        r.driveMotors(s)

        st = input("Press a button to stop the robot")

        r.stop()

def test(r):
    d = 1
    for speed in range(0, 270, 20):
        
        
        r.driveMotors(speed * d)

        time.sleep(.5)

        print(speed)
        r.stop()

        time.sleep(1)

        d *= -1

def schmoovin(r):
    speed = 250

    r.driveMotors(40)

    p = input("press button to stop robot")

    r.stop()


if __name__ == "__main__":
    main()
