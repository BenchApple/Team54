# Ben Chappell - Team 54
# This file implements basic functions for driving the vehicle

import brickpi3
import time

# accelerates the machine to a specific power value, from the current value over the course of time t.
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
    timeStep = abs(t / (targetPower - initialPower)) # Calculates the time step of the acceleration,
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

        # Wait for the time step before the next increment.
        time.sleep(timeStep)

    # Return the final power of the motors. All calls to this function should set some power variable equal
    # to the return of this funciton.
    return targetSpeed

# Stops the vehible over the course of time t.
# bp takes the brickpi object
# right and left take the brickpi designations for the right and left motors respectively.
# intial power takes the power of the motors at the time of calling the function.
# t takes the amount of time the stopping should take place over
def stop(bp, right, left, intialPower, t):
    return accelerate(bp, right, left, initialPower, 0, t)

# Sets the position of the steering motor to 0, aka perfectly straight.
# bp takes the brickpi object originally created.
# steer takes the id of the steering motor according to brickPi.

# NOTE: This could be a detrimental system IF the gear ever comes off the track.
# NOTE: we may need to devise a system to determine exactly what position the exact straight is.
def setStraight(bp, steer):
    return rotateAxle(bp, steer, 0)

# Rotates the axle motor to the target position.
# bp takes the brickpi object
# steer takes the motor designation of the steering motor according to brickpi
# target takes the target location of the axle motor.

# NOTE: This could be a detrimental system IF the gear ever comes off the track.
# NOTE: we may need to devise a system to determine exactly what position the exact straight is.
# NOTE: We need to do tests on how exactly set_motor_position works, what the viable range of values could be, etc
# TODO: let the caller customize degrees per second and have it ask for the starting position as well.
def rotateAxle(bp, steer, target):
    degreesPerSecond = 100
    start = bp.get_motor_encoder(steer)

    bp.set_motor_limits(steer, dps = degreesPerSecond)
    
    bp.set_motor_position(steer, target)

    return target
    

