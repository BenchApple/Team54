# Ben Chappell - Team 54
# This file contains an overarching class for the robot object.
# This class is meant to be easy to use for an implementation of the robot, so that the programmer has to do 
# minimum hard programming in order to control the robot.
# Hopefully this will properly contain all basic methods the robot requires to run

# imports
import brickpi3
import grovepi
import time
import numpy as np

class Robot:
    # Class variables - Anything initialized to false will be assigned by the user during object initialization
    bp = False # Stores the brickpi object
    leftM = False # Stores the left drive motor
    rightM = False # Stores the right drive motor
    steerM = False # Stores the bp id for the steering motor
    trailerM = False # Stores the bp id for the trailer motor
    power = 0 # Stores the persistent power of the two back motors NOTE: may change to degrees per second
    pos = 0 # Stores the position of the steering motor. abs(pos) always less than 250
    urgencies = [0, 0, 0] # Stores the urgency array in order [front, right, left]
    rightU = False # Stores the grove ID of the right ultrasonic sensor
    leftU = False # Stores the grove ID of the left ultrasonic sensor
    frontU = False # Stores the grove ID of the front ultrasonic sensor
    frontDist = 500 # Stores the distance read by the front ultrasonic
    rightDist = 500 # Stores the distance read by the right ultrasonic
    leftDist = 500 # Stores the distance read by the left ultrasonic
    dismountTime = 5 # Stores the time desired to let the dismount happen.
    trailerPower = 40 # Stores the power for the trailer motor.
    cylCount = False # Stores the number of cylinders carried
    cubeCount = False # Stores the number of cubes carried
    coneCount = False # Stores the number of cones carried.
    m = 20 # Max urgency distance, used in turning calculations
    k = -1 # Urgency constant, used in turning calculations
    maxTurn = 250 # Stores the abs val of the max distance from 0 that the robot can turn.

    # Initialization function, takes all motor and ultrasonic sensor arguments from above.
    def __init__(self, _bp, _lm, _rm, _sm, _tm, _ru, _lu, _fu, _cyl, _cube, _cone):
        self.bp = _bp
        self.leftM = _lm
        self.rightM = _rm
        self.steerM = _sm
        self.tralerM = _tm
        self.rightU = _ru
        self.leftU = _lu
        self.frontU = _fu
        self.cylCount = _cyl
        self.cubeCount = _cube
        self.coneCount = _cone

    # TODO create a setup function for basic instrument calibration.

    # Returns the value of the class variable bp
    def getBP(self):
        return self.bp

    # Sets a new value to bp, takes the new bp as an argument
    def setBP(self, newBP):
        self.bp = newBP

    # Returns the value of leftM
    def getLeftM(self):
        return self.leftM

    # Sets a new value for leftM
    def setLeftM(self, lm):
        self.leftM = lm

    # Returns the value of rightM
    def getRigthM(self):
        return self.rightM

    # Sets a new value for rightM
    def setRightM(self, rm):
        self.rightM = rm

    # Returns the value of steerM
    def getSteerM(self):
        return self.steerM

    # Sets a new value for steerM
    def setSteerM(self, sm):
        self.steerM = sm

    # Returns the value of trailerM
    def getTrailerM(self):
        return self.trailerM

    # Sets a new value for trailerM
    def setTrailerM(self, tm):
        self.trailerM = tm

    # reuturns the value of power
    def getPower(self):
        return self.power

    # Returns the position of the steering motor
    def getPos(self):
        return self.pos

    # Returns the list of urgencies
    def getUrgencies(self):
        return self.urgencies

    # Returns the id of the right ultrasonic
    def getRightU(self):
        return self.rightU

    # Sets a new value for the right sonic ID
    def setRightU(self, ru):
        self.rightU = ru

    # Returns the id of the left ultrasonic
    def getReftU(self):
        return self.leftU

    # Sets a new value for the left sonic ID
    def setReftU(self, lu):
        self.leftU = lu

    # Returns the id of the front ultrasonic
    def getFrontU(self):
        return self.frontU

    # Sets a new value for the front sonic ID
    def setFrontU(self, fu):
        self.frontU = fu

    # returns the current front distance
    def getFrontDist(self):
        return self.frontDist

    # Returns the right dist
    def getRightDist(self):
        return self.rightDist
    
    # returns the current left distance
    def getLeftDist(self):
        return self.leftDist

    # Returns the current dismount time.
    def getDismountTime(self):
        return self.dismountTime

    # Sets a new value for the dismount time
    def setDismountTime(self, dt):
        self.dismountTime = dt

    # Returns the value for the trailer power.
    def getTrailerPower(self):
        return self.trailerPower

    # Sets a new max trailer power.
    def setTrailerPower(self, tp):
        self.trailerPower = tp

    # Returns the number of cylinders carried
    def getCylCount(self):
        return self.cylCount

    # Sets a new value for the number of cylinders
    def setCylCount(self, cyl):
        self.cylCount = cyl

    # Returns the number of cubeinders carried
    def getCubeCount(self):
        return self.cubeCount

    # Sets a new value for the number of cubeinders
    def setCubeCount(self, cube):
        self.cubeCount = cube

    # Returns the number of coneinders carried
    def getConeCount(self):
        return self.coneCount

    # Sets a new value for the number of coneinders
    def setConeCount(self, cone):
        self.coneCount = cone

    # Returns the urgency distance
    def getM(self):
        return self.m

    # Lets the user set a new value for m
    def setM(self, _m):
        self.m = _m

    # Returns the current k value
    def getK(self):
        return self.k

    # Allows the user to replace the current k value
    def setK(self, _k):
        self.k = _k

    # Returns the current max turn value
    def getMaxTurn(self):
        return self.maxTurn

    # Allows the user to set a new value for the maxTurn
    def setMaxTurn(self, mt):
        self.maxTurn = mt

    # Accelerates the machine to a specific power value, from the current value over the course of time t
    # targetPower takes the desired power for the motors, the top speed.
    # t takes the amount of time that the acceleration should be done over.
    # Note: Originally implemented by changing the power of both of the motors at the same time,
    # but might be better to instead set the power of one of them then somehow make it so the other one
    # constantly matches the first one.
    def accelerate(self, targetPower, t):
        # Calculates the time step of the acceleration,
        timeStep = abs(t / (targetPower - self.power)) if targetPower != self.power else 0
        # how much time in between power increase.

        # Determine the direction of the acceleration, positive or negative.
        direc = -1 if targetPower > self.power else 1

        # Change the power of the motors by 1 per time step.
        # NOTE: since the motors are oriented backwards, in order to move forwards, we accelerate in the negative direction
        # In the input, however, a positive targetPower is associated with going forwards, for simplicity.
        for i in range(-self.power, -targetPower, direc):
            # Set both of the motors to the new power.
            self.bp.set_motor_power(self.rightM, i)
            self.bp.set_motor_power(self.leftM, i)

            # Wait for the time step before the next increment. If the step is very small, ignore.
            if timeStep > .005:
                time.sleep(timeStep)

        # Return the final power of the motors. All calls to this function should set some power variable equal
        # to the return of this funciton.
        self.power = targetPower

    # Stops the vehible over the course of time t.
    # bp takes the brickpi object
    # right and left take the brickpi designations for the right and left motors respectively.
    # intial power takes the power of the motors at the time of calling the function.
    # t takes the amount of time the stopping should take place over
    def stop(self, t):
        self.accelerate(0, t) # No need for assignment since acceleration does it anyways


    # Sets the position of the steering motor to 0, aka perfectly straight.
    # NOTE: This could be a detrimental system IF the gear ever comes off the track.
    # NOTE: we may need to devise a system to determine exactly what position the exact straight is.
    def setStraight(self):
        self.rotateAxle(0)
    
    # Rotates the axle motor to the target position. Returns the initial position if the target spot is greater than the limit.
    # target takes the target location of the axle motor.
    # degreesps takes the desired degrees per second of rotation.
    # t takes the time in seconds for the rotation to occur over. This call is optional.

    # NOTE: This could be a detrimental system IF the gear ever comes off the track.
    # NOTE: we may need to devise a system to determine exactly what position the exact straight is.
    # NOTE: We need to do tests on how exactly set_motor_position works, what the viable range of values could be, etc
    # TODO: Determine exactly how far in degrees the robot can rotate before it hits its max. Implement stoppers to prevent calls past that point.
    # With the current (7 hole) design, the limit in degrees is 200
    # With the 9 hole design, the limit in degrees is 250
    def rotateAxle(self, target, degreesps = 200, t = 0):
        if abs(target) > 250: # TODO: set some sort of proper way to handle this situation.
            self.pos = self.pos
    
        if t:
            degreesps = target / t

        self.bp.set_motor_limits(self.steerM, dps = degreesps)
        self.bp.set_motor_position(self.steerM, target)

        self.pos = target

    # Determines how much to turn based on the urgencies function from sensors.py
    # TODO Finish this function
    def turnFromUrgency(self):
        f = self.urgencies[0] # The front urgency
        r = self.urgencies[1] # the right urgency
        l = self.urgencies[2] # the left urgency

        # TODO determine the direction to turn based on the urgencies value. Fucking do this with
        # Annelise in the room please.

    # Gets the output of the ultrasonic at port port
    # Private function
    # port should be one of the class objects.
    # The public versions of these are the individual thingies.
    def getUltrasonics(self):
        self.rightDist = grovepi.ultrasonicRead(self.rightU)
        self.leftDist = grovepi.ultrasonicRead(self.leftU)
        self.frontDist = grovepi.ultrasonicRead(self.frontU)

    def getUrgency(self):
        # Get the updated ultrasonic readings
        self.getUltrasonics

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
        fUrgency = (2 / np.pi) * np.arctan(k * (self.frontDist - m)) + 1
        rUrgency = (2 / np.pi) * np.arctan(k * (self.rightDist - m)) + 1
        lUrgency = (2 / np.pi) * np.arctan(k * (self.leftDist - m)) + 1

        self.urgencies[0] = fUrgency
        self.urgencies[1] = rUrgency
        self.urgencies[2] = lUrgency

    # Dismounts the carried cargo from the trailer, idea to differ this function based on the cargo carried.
    # bp - takes the brickpi object
    # track - takes the brickpi id for the track motor
    # clyCount - Takes the number of cylinders taken as cargo
    # cubeCount - takes the number of cubes taken as cargo
    # coneCount - takes the number of cones taken as cargo
    def dismount(self, cylCount, cubeCount, coneCount):
        # The amount of time it takes the fully dismount

        for i in range(0, self.trailerPower):
            self.bp.set_motor_power(self.trailerM, i)
            time.sleep(.05)

        time.sleep(self.dismountTime)

        for i in range(self.trailerPower, 0, -1):
            self.bp.set_motor_power(self.trailerM, i)
            time.sleep(0.01)