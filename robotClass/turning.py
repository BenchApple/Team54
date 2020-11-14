# Ben Chappell - Team 54

# This file stores a bunch of turning files, for the various turning needs we have.

import time
import brickpi3
import grovepi
from robot import Robot

# Acts very basically, just turns the robot a certain amount when a line is detected.
# Maintains same speed, same turning every time, etc.
def basicReact(robot):
    d = 0 # Stores the direction that we will turn in.
    
    robot.getLineReadings()

    if robot.getRightLineReading():
        d = 1
    elif robot.getLeftLineReading():
        d = -1
    
    return robot.rotateAxle(robot.getPos() + (robot.getLineTurn() * d))
    
# When a line is detected, stop the robot until the line is no longer detected or the turning limit is reached, 
# then go back to the minimum dps.
def stopTurn(robot):
    robot.driveMotors(robot.minDps)
    robot.getLineReadings()
    turning = True

    while ((robot.getRightLineReading() or robot.getLeftLineReading()) and turning):
        robot.getLineReadings()
        turning = basicReact(robot)