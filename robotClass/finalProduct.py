# Ben Chappell - Team 54

# This file holds the last of our code, our main event loop, etc.

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
        rightL = 4
        leftL = 8
        hall = 7

        dropSite = 'A'
        minDPS = 35

        # Set the cargo counters.
        cyl = 0
        cone = 0
        cube = 0

        time.sleep(1)

        # initialize our robot value
        r = Robot(bp, left, right, steer, trailer, rightU, leftU, frontU, rightL, leftL, 
                      hall, dropSite, minDPS, _cyl=cyl, _cube=cube, _cone=cone)
        r.diagnostics()
        r.setStraight()

        # Set our basic parameters for the loop
        # Set the magnet flag equal to false. The magnetic tracker is only called IF a magnet is detected 
        # and the magnet flag is equal to false.    
        magnetDetected = False
        magnetThreshold = 900 # Stores the magnitude of a magnetic field at which we consider a magnet found.
        ignore = 0
        engageDismount = False
        results = [0,0]
        d = results[1]
        activated = results[0]
        dismountEngageStartTime = time.time()
        timeBeforeDismount = 5 # The amount of time between detection of the magnet and dismount start.

        # main event loop.
        while (r.getBeaconsDetected() < r.getDropAfter() + 2):
            # Drive the vehicle forwards.
            r.driveMotors(r.getMinDps())

            # Get all of the sensor readings we need.
            r.getMagMagnReading()
            r.getUltrasonics()
            r.getLineReadings()

            # Check to see if a magnet is detected.
            if r.getMagMagn() > magnetThreshold and not magnetDetected:
                r.incrementBeaconsDetected() # Increment the detection count by one

                # If the this is before the path we need to follow, ignore the branch
                if r.getBeaconsDetected() < r.getDropAfter():
                    ignore = 1
                # If this is the path we need to follow, make sure we don't ignore it.
                elif r.getBeaconsDetected() == r.getDropAfter():
                    ignore = 0
                elif r.getBeaconsDetected() > r.getDropAfter():
                    # This is where we put the dismount code. We need to figure out exactly how this will work
                    engageDismount = True
                    dismountEngageStartTime = time.time()

            # Start the dismount process if it's time to do so.
            if engageDismount:
                # Put all the dismount code here. I don't quite know what to do here yet.
                # Use the fact that we know the time of the start of the dismount in order to do this.
                currentTime = time.time()
                ignore = 0

                if currentTime - dismountEngageStartTime > timeBeforeDismount:
                    r.dismount()
                    dismountStartTime = time.time()
                elif currentTime - dismountStartTime > r.getDismountTime():
                    # TODO reset all cargo values after dismount.
                    r.stopDismount()
                    engageDismount = False

            # call the wall stop function to avoid running into a wall
            r.wallStop()

            # Execute the code to turn the robot.
            results = t.rubberBandAccelTurningFixedBack(r, activated, d, ignore)
            activated = results[0]
            d = results[1]

            # TODO somehow get this to actually realize when it should accelerate to the wall dps 
            # Probably do some sort of counter to detect the number of times in a row that activated = 0
            # That way you could just detect when you're straight and run it.
            # TODO still need to implement the ramp thing.

        r.stop()

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


if __name__ == "__main__":
    main()
