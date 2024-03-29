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
    # Initialization function, takes all motor and ultrasonic sensor arguments from above.
    def __init__(self, _bp, _lm, _rm, _sm, _tm, _ru, _lu, _fu, rl_, ll_, hall_, _cyl=0, _cube=0, _cone=0):
        # Class variables - Anything initialized to false will be assigned by the user during object initialization
        self.power = 0 # Stores the persistent power of the two back motors NOTE: may change to degrees per second
        self.pos = 0 # Stores the position of the steering motor. abs(pos) always less than 250
        self.urgencies = [0, 0, 0] # Stores the urgency array in order [front, right, left]
        self.frontDist = 500 # Stores the distance read by the front ultrasonic
        self.rightDist = 500 # Stores the distance read by the right ultrasonic
        self.leftDist = 500 # Stores the distance read by the left ultrasonic
        self.dismountTime = 5 # Stores the time desired to let the dismount happen.
        self.dismountPower = 30 # Stores the power needed for the accel motors to dismount.
        self.trailerPower = 30 # Stores the power for the trailer motor.
        self.m = 20 # Max urgency distance, used in turning calculations
        self.k = -1 # Urgency constant, used in turning calculations
        self.maxTurn = 250 # Stores the abs val of the max distance from 0 that the robot can turn.
        self.lineTurn = 30 # Stores the base turn if the line finder finds a black line.
        self.rlReading = False # Stores the state of the right line finder. False if white line, true if black line
        self.llReading = False # Stores the state of the left line finder. False if while line, true if black line
        self.hallReading = False # Stores the state of the hall sensor. False if no reading, true if there is reading
        self.leftLineCount = 0 # Stores the number of times in a row the left line sensor has read True
        self.rightLineCount = 0 # Stores the number of times in a row the right line sensor ahs read true

        # All of these are variables determined by the user.
        self.bp = _bp # Stores the brickpi object
        self.leftM = _lm # Stores the left drive motor
        self.rightM = _rm # stores the right drive motor
        self.steerM = _sm # stores the bp id for the steering motor
        self.trailerM = _tm # stores the id for the trailer motor
        self.rightU = _ru # stores the port of the right ultrasonic
        self.leftU = _lu # Stores the port of the left ultrasonic 
        self.frontU = _fu # stores the port of the front ultrasonic
        self.rightLine = rl_ # Stores the ID for the right line finder.
        self.leftLine = ll_ # Stores the ID for the left line finder.
        self.hall = hall_ # Stores the ID for the hall sensor.
        self.cylCount = _cyl # Stores the number of cylinders
        self.cubeCount = _cube # stores the number of cubes
        self.coneCount = _cone # stores the number of comes.
        self.diagnostics() # Runs the diagnostics function.
        

        # Set the pin modes for the grovepi.
        grovepi.pinMode(self.rightLine, "INPUT")
        grovepi.pinMode(self.leftLine, "INPUT")
        grovepi.pinMode(self.hall, "INPUT")

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

    # Returns the ID for the right line finder
    def getRightLineID(self):
        return self.rightLine

    def setRightLineID(self, rl):
        self.rightLine = rl

    def getLeftLineID(self):
        return self.leftLine

    def setLeftLineID(self, ll):
        self.leftLine = ll

    # Gets the ID for the hall sensor
    def getHallID(self):
        return self.hall

    # Sets the hall id to a new value
    def setHallID(self, h):
        self.hall = h

    # returns the current reading of the hall sensor.
    def getHallStatus(self):
        return self.hallReading

    # Returns the current status of the right line reading
    def getRightLineReading(self):
        return self.rlReading
    
    # Returns the current status of the left line reading
    def getLeftLineReading(self):
        return self.llReading

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

    # Gets the current right line count
    def getRightLineCount(self):
        return self.rightLineCount

    # Gets the current left line count
    def getLeftLineCount(self):
        return self.leftLineCount

    # Performs basic diagnostics on the robot, such as setting wheels straight, etc.
    # Current functions. Set wheels straight.
    def diagnostics(self):
        self.setStraight()

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
    # TODO: Determine exactly how far in degrees the robot can rotate before it hits its max. Implement stoppers to prevent calls past that point.
    # With the current (7 hole) design, the limit in degrees is 200
    # With the 9 hole design, the limit in degrees is 250
    def rotateAxle(self, target, degreesps = 360, t = 0):
        if abs(target) > 250: # TODO: set some sort of proper way to handle this situation.
            target = self.maxTurn if target > 1 else -self.maxTurn
    
        if t:
            degreesps = target / t

        self.bp.set_motor_limits(self.steerM, dps = degreesps) # set some limits for the steering motor
        self.bp.set_motor_position(self.steerM, target)

        self.pos = target

    # Determines how much to turn based on the urgencies function from sensors.py
    # TODO Finish this function
    def turnFromUrgency(self):
        f = self.urgencies[0] # The front urgency
        r = self.urgencies[1] # the right urgency - positive direction
        l = self.urgencies[2] # the left urgency - negative direction

        if f > r > l:
            if r > l:
                self.rotateAxle(-self.maxTurn * f)
            else:
                self.rotateAxle(self.maxTurn * f)
        elif r > l:
            self.rotateAxle(-self.maxTurn * r)
        else:
            self.rotateAxle(self.maxTurn * l)

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
        self.getUltrasonics()

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
        fUrgency = (2 / np.pi) * np.arctan(self.k * (self.frontDist - self.m)) + 1
        rUrgency = (2 / np.pi) * np.arctan(self.k * (self.rightDist - self.m)) + 1
        lUrgency = (2 / np.pi) * np.arctan(self.k * (self.leftDist - self.m)) + 1

        self.urgencies[0] = fUrgency
        self.urgencies[1] = rUrgency
        self.urgencies[2] = lUrgency

    # Dismounts the carried cargo from the trailer, idea to differ this function based on the cargo carried.
    # bp - takes the brickpi object
    # track - takes the brickpi id for the track motor
    # clyCount - Takes the number of cylinders taken as cargo
    # cubeCount - takes the number of cubes taken as cargo
    # coneCount - takes the number of cones taken as cargo
    def dismount(self):
        # The amount of time it takes the fully dismount

        for i in range(0, self.trailerPower, 1 if self.trailerPower > 0 else -1):
            self.bp.set_motor_power(self.trailerM, i)
            time.sleep(.05)

        self.accelerate(self.dismountPower, self.dismountTime)

        for i in range(self.trailerPower, 0, -1 if self.trailerPower > 0 else 1):
            self.bp.set_motor_power(self.trailerM, i)
            time.sleep(0.01)

    # Gets the readings from the two line finders. True if black line detected, False if no line detected
    def getLineReadings(self):
        if grovepi.digitalRead(self.rightLine) == 1:
            self.rlReading = True
            self.rightLineCount += 1
        else:
            self.rlReading = False
            self.rightLineCount = 0

        if grovepi.digitalRead(self.leftLine) == 1:
            self.llReading = True
            self.rightLineCount += 1
        else:
            self.llReading = False
            self.rightLineCount = 0

    # Basic reaction to the data from the line finders. Just turns towards the side with the line reading.
    def reactToLineFinders(self):
        d = 0
        count = 0 # stores the count to add to the turn
        countMultiplier = 2 # Stores the count multiplier for the turn radius
        if self.rlReading:
            d = 1
            count = self.rightLineCount
        elif self.llReading:
            d = -1
            count = self.leftLineCount

        self.rotateAxle(self.pos + ((self.lineTurn * d) + (count * countMultiplier)))

    # Gets the readings from the hall sensor and sets the reading variable to true if detected, false if not
    def getHallReading(self):
        self.hallReading = True if grovepi.digitalRead(self.hall) == 1 else False


