# Ben Chappell - Team 54

# This file stores a bunch of turning files, for the various turning needs we have.

import time
import brickpi3
import grovepi
from robot import Robot

# Acts very basically, just turns the robot a certain amount when a line is detected.
# Maintains same speed, same turning every time, etc.
# This function is called in nearly every turn function, just as a basic driver.
def basicReact(robot):
    d = 0 # Stores the direction that we will turn in.
    
    robot.getLineReadings()

    if robot.getRightLineReading():
        d = 1
    elif robot.getLeftLineReading():
        d = -1

    print("Turning from line finders " + str(robot.getLineTurn() * d) + " degrees")
    
    return robot.rotateAxle(robot.getPos() + (robot.getLineTurn() * d))
    
# When a line is detected, stop the robot until the line is no longer detected or the turning limit is reached, 
# then go back to the minimum dps.
def stopTurn(robot):
    robot.driveMotors(robot.getMinDps())
    robot.getLineReadings()
    turning = True

    while ((robot.getRightLineReading() or robot.getLeftLineReading()) and turning):
        turning = basicReact(robot)

# Reduce the dps by 5 while turning to make the turning slower.
def slowTurn(robot):
    robot.driveMotors(robot.getMinDps())
    robot.getLineReadings()
    turning = True
    speedReduction = 5

    # Turn while a line is still detected.
    while ((robot.getRightLineReading() or robot.getLeftLineReading()) and turning):
        turning = basicReact(robot)
        robot.driveMotors(robot.getMinDps() - speedReduction)


# turn and increase turning speed while 
def increaseTurnSpeed(robot):
    robot.driveMotors(robot.getMinDps())
    robot.getLineReadings()
    turning = True
    lineDetectionStreak = 0
    robot.setLineTurn(robot.getBaseLineTurn())

    while ((robot.getRightLineReading() or robot.getLeftLineReading()) and turning):
        turning = basicReact(robot)
        lineDetectionStreak += 1
        robot.setLineTurn(robot.getBaseLineTurn() + lineDetectionStreak)

# Increase turn speed while slowing down the robot as well.
def slowIncreaseTurnSpeed(robot):
    robot.driveMotors(robot.getMinDps())
    robot.getLineReadings()
    turning = True
    lineDetectionStreak = 0
    robot.setLineTurn(robot.getBaseLineTurn())
    speedReduction = 5

    while ((robot.getRightLineReading() or robot.getLeftLineReading()) and turning):
        turning = basicReact(robot)
        lineDetectionStreak += 1
        robot.setLineTurn(robot.getBaseLineTurn() + lineDetectionStreak)
        robot.driveMotors(robot.getMinDps() - speedReduction)

# Marginally increase turning speed while in the turning period
def fastTurn(robot):
    robot.driveMotors(robot.getMinDps())
    robot.getLineReadings()
    turning = True
    speedIncrease = 5

    # Turn while a line is still detected.
    while ((robot.getRightLineReading() or robot.getLeftLineReading()) and turning):
        turning = basicReact(robot)
        robot.driveMotors(robot.getMinDps() + speedIncrease)

# Increase the turning speed while making the robot faster.
def fastIncreaseTurnSpeed(robot):
    robot.driveMotors(robot.getMinDps())
    robot.getLineReadings()
    turning = True
    lineDetectionStreak = 0
    robot.setLineTurn(robot.getBaseLineTurn())
    speedIncrease = 5

    while ((robot.getRightLineReading() or robot.getLeftLineReading()) and turning):
        turning = basicReact(robot)
        lineDetectionStreak += 1
        robot.setLineTurn(robot.getBaseLineTurn() + lineDetectionStreak)
        robot.driveMotors(robot.getMinDps() + speedIncrease)

# Rubber band effect - while continuously moving, the robot attempts to return the direction back to 0
# Effectively works like continuous moving turning until the turning line finder turns off.
# Once the line finder turns off (only the one that initiated the turn), the robot attempts to return
# back to position 0.
# Will likely require a number of parameters and return values to accomodate for other sensors.
# This function acts like it will be called within an event sensing loop.
# NOTE this function assumes that the turn radius is enough that the right line sensor will end up on the left side of the line, etc
# activatedLineFinder - takes an integer 1,0,-1 that indicates which line finder 
#                       activated last, allowing us to keep track of which direction to turn back to 0 from
#                       1 means right, -1 means left, 0 means neither (straight) gets reset upon the position being 0.
def rubberBandTurning(robot, activatedLineFinder):
    robot.getLineReadings()
    turning = True # keeps track of whether or not we have reached the edge of how much we can turn.
    d = 0 # Stores the direction in which we're turning 1 means right, -1 left, 0 no turn

    if abs(robot.getPos()) < robot.getLineTurn(): # close enough to straight, just in case there's some sort of error.
        activatedLineFinder = 0

    if activatedLineFinder == 0:
        if robot.getRightLineReading():
            activatedLineFinder = 1
            d = 1
        elif robot.getLeftLineReading():
            activatedLineFinder = -1
            d = -1
    elif activatedLineFinder == 1: # If the right line finder was the most recently activated one, go here
        if robot.getRightLineReading():
            activatedLineFinder = 1
            d = 1
        else: # if the right line finder isn't activated, start turning back towards center.
            d = -1
            print("turning back towards center")
    elif activatedLineFinder == -1: # If the left line finder was the one last activated, go here
        if robot.getLeftLineReading():
            activatedLineFinder = -1
            d = -1
        else: # if the left line finder isn't activated, turn back towards center.
            d = 1
            print("turning back towards center")

    print ("The last activated line finder is " + str(activatedLineFinder))
    # Turn based on what the line finders found. Should be easy to adapt to changine lineTurn values similar to other increaseTurnSpeed functions.
    print("Turning from line finders " + str(robot.getLineTurn() * d) + " degrees")
    turning = robot.rotateAxle(robot.getPos() + (robot.getLineTurn() * d))

    # Return the activated Line finder so it can be passed back into the function the next time it's called.
    return activatedLineFinder

# Implements the rubberBandTurning function but with the turning speed increasing every time d is the same as the previous call to the function.
def rubberBandAccelTurning(robot, activatedLineFinder, prevD):
    robot.getLineReadings()
    turning = True # keeps track of whether or not we have reached the edge of how much we can turn.
    d = 0 # Stores the direction in which we're turning 1 means right, -1 left, 0 no turn

    if abs(robot.getPos()) < robot.getLineTurn(): # close enough to straight, just in case there's some sort of error.
        activatedLineFinder = 0

    if (not activatedLineFinder):
        if robot.getRightLineReading():
            activatedLineFinder = 1
            d = 1
        elif robot.getLeftLineReading():
            activatedLineFinder = -1
            d = -1
    elif activatedLineFinder == 1: # If the right line finder was the most recently activated one, go here
        if robot.getRightLineReading():
            activatedLineFinder = 1
            d = 1
        else: # if the right line finder isn't activated, start turning back towards center.
            d = -1
    elif activatedLineFinder == -1: # If the left line finder was the one last activated, go here
        if robot.getLeftLineReading():
            activatedLineFinder = -1
            d = -1
        else: # if the left line finder isn't activated, turn back towards center.
            d = 1

    # Turn based on what the line finders found. Should be easy to adapt to changine lineTurn values similar to other increaseTurnSpeed functions.
    #print("Turning from line finders " + str(robot.getLineTurn() * d) + " degrees")
    turning = robot.rotateAxle(robot.getPos() + (robot.getLineTurn() * d))

    if d == prevD and turning:
        robot.setLineTurn(robot.getLineTurn() + 1)
    else:
        robot.setLineTurn(robot.getBaseLineTurn())

    # Return the activated Line finder so it can be passed back into the function the next time it's called.
    return [activatedLineFinder, d]

# Implements the rubberBandTurning function but with the turning speed increasing every time d is the same as the previous call to the function.
def rubberBandAccelTurningFixedBack(robot, activatedLineFinder, prevD):
    robot.getLineReadings()
    turning = True # keeps track of whether or not we have reached the edge of how much we can turn.
    # d - Stores the direction in which we're turning 1 means right, -1 left, 0 no turn

    if (not activatedLineFinder):
        if robot.getRightLineReading():
            activatedLineFinder = 1
            d = 1
        elif robot.getLeftLineReading():
            activatedLineFinder = -1
            d = -1
    elif activatedLineFinder == 1: # If the right line finder was the most recently activated one, go here
        if robot.getRightLineReading():
            activatedLineFinder = 1
            d = 1
        else: # if the right line finder isn't activated, start turning back towards center.
            d = -1
            robot.setLineTurn(robot.getBaseLineTurn()) # When going back to center, dont turn back as fast.
    elif activatedLineFinder == -1: # If the left line finder was the one last activated, go here
        if robot.getLeftLineReading():
            activatedLineFinder = -1
            d = -1
        else: # if the left line finder isn't activated, turn back towards center.
            d = 1
            robot.setLineTurn(robot.getBaseLineTurn())

    # Turn based on what the line finders found. Should be easy to adapt to changine lineTurn values similar to other increaseTurnSpeed functions.
    #print("Turning from line finders " + str(robot.getLineTurn() * d) + " degrees")
    turning = robot.rotateAxle(robot.getPos() + (robot.getLineTurn() * d))

    if d == prevD and turning:
        robot.setLineTurn(robot.getLineTurn() + 1)
    else:
        robot.setLineTurn(robot.getBaseLineTurn())

    if abs(robot.getPos()) < robot.getLineTurn(): # close enough to straight, just in case there's some sort of error.
        activatedLineFinder = 0

    # Return the activated Line finder so it can be passed back into the function the next time it's called.
    return [activatedLineFinder, d]


# The idea here is that the robot turns a specific amount before pulsing forward, until the max turn is reached.
# The robot stops in between pulses
# NOTE this could present issues if, while turning, other sensors have to be read (such as magnet finder)
# therefore, TODO adapt this algo to let other sensors also read while it's going
# NOTE potentially, use this pulsing design as a central paradigm for the whole thing (i don't think this should be the case)
# NOTE While programming it, I figured that it wouldn't be worthwhile just becuase of the sheer number of moving parts
# Maybe finish it later?
def stochasticTurning(robot):
    robot.driveMotors(robot.getMinDps())
    robot.getLineReadings()
    pulseDegrees = 30 # how far forward we pulse between turning values.
    oneStepTurnLimit = 60 # How many degrees you can turn per stochastic step


