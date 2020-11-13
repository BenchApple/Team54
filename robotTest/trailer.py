# Ben Chappell - Team 54
# This file implements basic functions for the operation of the trailer

import brickpi3
import time
import sensors as s


# Dismounts the carried cargo from the trailer, idea to differ this function based on the cargo carried.
# bp - takes the brickpi object
# track - takes the brickpi id for the track motor
# clyCount - Takes the number of cylinders taken as cargo
# cubeCount - takes the number of cubes taken as cargo
# coneCount - takes the number of cones taken as cargo
def dismount(bp, track, cylCount, cubeCount, coneCount):
    maxPower = 40
    dismountTime = 5 # The amount of time it takes the fully dismount

    for i in range(0, maxPower):
        bp.set_motor_power(track, i)
        time.sleep(.05)

    time.sleep(dismountTime)

    for i in range(maxPower, 0, -1):
        bp.set_motor_power(track, i)
        time.sleep(0.01)
