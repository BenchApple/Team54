# Ben Chappell - Team 54
# This file implements basic functions for driving the vehicle

import brickpi3
import time

# Accelerates the machine to a specific power value, from the current value over the course of time t.
# bp takes the brickpi object originally created.
# right takes the brickpi argument for the right motor
# left takes the brickpi argument for the left motor
# initialPower takes the power of the motors before this function was called
# targetPower takes the desired power for the motors, the top speed.
# t takes the amount of time that the acceleration should be done over.
# Note: Originally implemented by changing the power of both of the motors at the same time,
# but might be better to instead set the power of one of them then somehow make it so the other one
# constantly matches the first one.
def accelerate(bp, right, left, initialPower, targetPower, t):
    timeStep = abs(t / (targetPower - initialPower)) if targetPower != initialPower else 0 # Calculates the time step of the acceleration,
    # how much time in between power increase.

    # Determine the direction of the acceleration, positive or negative.
    direc = -1 if targetPower > initialPower else 1

    # Change the power of the motors by 1 per time step.
    # NOTE: since the motors are oriented backwards, in order to move forwards, we accelerate in the negative direction
    # In the input, however, a positive targetPower is associated with going forwards, for simplicity.
    for i in range(-initialPower, -targetPower, direc):
        # Set both of the motors to the new power.
        bp.set_motor_power(right, i)
        bp.set_motor_power(left, i)

        # Wait for the time step before the next increment. If the step is very small, ignore.
        if timeStep > .005:
            time.sleep(timeStep)

    # Return the final power of the motors. All calls to this function should set some power variable equal
    # to the return of this funciton.
    return targetPower

# Stops the vehible over the course of time t.
# bp takes the brickpi object
# right and left take the brickpi designations for the right and left motors respectively.
# intial power takes the power of the motors at the time of calling the function.
# t takes the amount of time the stopping should take place over
def stop(bp, right, left, initialPower, t):
    return accelerate(bp, right, left, initialPower, 0, t)

# Sets the position of the steering motor to 0, aka perfectly straight.
# bp takes the brickpi object originally created.
# steer takes the id of the steering motor according to brickPi.
# pos takes the current position of the steering motor

# NOTE: This could be a detrimental system IF the gear ever comes off the track.
# NOTE: we may need to devise a system to determine exactly what position the exact straight is.
def setStraight(bp, steer, pos):
    return rotateAxle(bp, steer, 0, pos)

# Rotates the axle motor to the target position. Returns the initial position if the target spot is greater than the limit.
# bp takes the brickpi object
# steer takes the motor designation of the steering motor according to brickpi
# target takes the target location of the axle motor.
# initialPos takes the initial position of the motor, idek if this has a use yet.
# degreesps takes the desired degrees per second of rotation.
# t takes the time in seconds for the rotation to occur over. This call is optional.

# NOTE: This could be a detrimental system IF the gear ever comes off the track.
# NOTE: we may need to devise a system to determine exactly what position the exact straight is.
# NOTE: We need to do tests on how exactly set_motor_position works, what the viable range of values could be, etc
# TODO: Determine exactly how far in degrees the robot can rotate before it hits its max. Implement stoppers to prevent calls past that point.
# With the current (7 hole) design, the limit in degrees is 200
# With the 9 hole design, the limit in degrees is 250
def rotateAxle(bp, steer, target, initialPos, degreesps = 200, t = 0):
    if abs(target) > 250:
        return initialPos
    
    if t:
        degreesps = target / t

    bp.set_motor_limits(steer, dps = degreesps)
    
    bp.set_motor_position(steer, target)

    return target
    

