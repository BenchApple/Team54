# Ben Chappell - Team 54
# Implements basic functions for interpreting sensor data and stuff.

import brickpi3
import grovepi
import time
import math
import drive as d
import numpy as np

# Gets the output of the ultrasonic at port port
def getUltrasonic(port):
    return grovepi.ultrasonicRead(port)

### DEPRECATED AS OF ADDING ADDITIONAL ULTRASONICS ###
# Gets the ultrasonic reading from the current forward, 5 degrees to the right
# and five degrees to the left. Returns 1 if turning should take place
# in the positive direction, -1 if the negative direction, and 0 if no turn should.
# take place. Only works with one ultrasonic for now.

# bp - takes the brickpi object
# port - takes the ultrasonic port
# steer - Takes the brickpi port for the steering motor.
# initialPos - takes the position of the steering motor.
# dChange - takes the change in degrees to measure at.
def getSurroundingsOld(bp, port, steer, initialPos, dChange):
    straight = getUltrasonic(port)

    # Get the ultrasonic reading from 5 degrees in the positive direction.
    d.rotateAxle(bp, steer, initialPos + dChange, initialPos)
    time.sleep(.01)
    pos = getUltrasonic(port)

    # Get the ultrasonic reading from 5 degrees in the negative direction.
    d.rotateAxle(bp, steer, initialPos - dChange, initialPos)
    time.sleep(.01)
    neg = getUltrasonic(port)

    d.setStraight(bp, steer, initialPos - 5)

    if straight > neg and straight < pos:
        return 0
    elif neg > pos:
        return -1
    else:
        return 1

# Gets the ultrasonic readings from all three front sonics. Returns 0 if
# forward facing sonic is longest distance, 1 if right is longest,
# -1 if left is longest. Used to determine what obstacles surround.

# bp - takes the brickpi object
# front - takes the forward facing ultrasonic port
# right - takes the right facing ultrasonic port
# left - takes the left facing ultrasonic port
def getSurroundings(bp, front, right, left):
    f = getUltrasonic(front)
    r = getUltrasonic(right)
    l = getUltrasonic(left)

    if f > r and f > l:
        return 0
    elif r > l:
        return 1
    else:
        return -1

# Returns a tuple of length three [front, right, left] of urgencies.
# An urgency of 1 (one) defines max urgency, meaning that the robot should turn away from that obstacle as quickly
# as possible. What this entails is yet to be determined, but can be up to the implementer.
# An urgency of 0 defines minimum urgency, likely meaning that direction is completely unoccupied.

def getUrgency(front, right, left):
    f = getUltrasonic(front)
    r = getUltrasonic(right)
    l = getUltrasonic(left)

    # Variable declaration - subject to change
    m = 20 # Max urgency distance. When urgency is at it's maximum. Minimum urgency is always at maximum distance
    k = -1 # The urgency constant, currently defined as negative 1.

    # Urgency is calculated by a numpy inverse tangent function. This acheives a gradual increase in urgency
    # or a nonlinear increase in urgency.
    # this urgency function is defined as u = (2/pi) * arctan(k(d - m)) + 1
    # the 2 / pi normalizes funtion to 1.
    # k is the urgency constant, which defines how sharp the increase is as distance approaches the max distance.
    # k is ALWAYS less than 1 (negative)
    # NOTE: We will have to experimentally determine the final value of k through a number of trials
    # d is defined as distance, and is what u is parameterized by.
    # m is defined as the max urgency distance, and is where urgency will be defined as one.
    # NOTE: theoretically, urgencies greater than 1 are possible, and can definitely be handled, but
    # getting an urgency greater than 1 is definitely just bad.
    fUrgency = (2 / np,pi) * arctan(k * (f - m)) + 1
    rUrgency = (2 / np,pi) * arctan(k * (r - m)) + 1
    lUrgency = (2 / np,pi) * arctan(k * (l - m)) + 1

    return (fUrgency, rUrgency, lUrgency)
    

    
    
