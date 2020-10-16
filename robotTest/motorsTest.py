# Ben Chappell
# This file is for testing the motors and stuff

# Import all the shit we need
import brickpi3
import time

# Initialize brickpi object
bp = brickpi3.BrickPi3()

def main():
    try:
        # Create our sensor types and stuff
        leftDrive = bp.PORT_C
        rightDrive = bp.PORT_B
        steering = bp.PORT_D

        # Test steering
        #steeringTest(bp, steering)

        # Test right motor
        #rightTest(bp, rightDrive)

        # Test left motor
        #driveTest(bp, leftDrive)

        shimmy(bp, rightDrive, leftDrive, steering)

    except KeyboardInterrupt:
        bp.reset_all()

    bp.reset_all()

def steeringTest(bp, steering):
    # Test the steering motor.
    steeringSpeed = 50
    for i in range(0,20):
        # Set the steering motor's speed.
        bp.set_motor_power(steering, steeringSpeed)
        
        # Let the motor run for 1 second.
        time.sleep(.25 if not i else .5)

        # Reverse the steering direction.
        steeringSpeed *= -1

        try:
            # Print our motor encoders.
            print("Steering Encoder: " + str(bp.get_motor_encoder(steering)))
        except IOError as error:
            print(error)

        time.sleep(0.02)

def driveTest(bp, drive):
    # Set the speed to 20
    speed = 10
    finalSpeed = 100
    acceleration = 1 # The amount per tick that we increase the motor speed
    direction = 1 # Determines which direction we go in.
    
    for i in range(0,6):
        # Accelerate to target speed
        for i in range(speed, finalSpeed * direction, direction * acceleration):
            speed = i 
            bp.set_motor_power(drive, speed)

            time.sleep(.15)

        # Reverse the steering direction.
        direction *= -1

        try:
            # Print our motor encoders.
            print("Steering Encoder: " + str(bp.get_motor_encoder(drive)))
        except IOError as error:
            print(error)

        time.sleep(0.02)

def shimmy(bp, right, left, steer):
    speed = 0

    # Turn left, then go forward a little.
    bp.set_motor_power(steer, 20)
    time.sleep(.2)
    bp.set_motor_power(steer, 0)
    
    speed = accelBoth(bp, right, left, speed, 50, 1)
    time.sleep(0)
    # Decelerate
    speed = accelBoth(bp, right, left, speed, 0, 1)

    bp.reset_all()


# Goes from target speed to final speed in time, returns final speed. Time is in seconds.
def accelBoth(bp, right, left, startSpeed, targetSpeed, t): # Take the target speed in positive, turn it into negative in this function    
    # Calculate the needed time step.
    timeStep = abs(t / (targetSpeed - startSpeed))

    direc = 1
    # Determine whether increasing or decreasing.
    if targetSpeed > startSpeed:
        direc = -1

    for i in range(-startSpeed, -targetSpeed, direc):
        bp.set_motor_power(right, i)
        bp.set_motor_power(left, i)

        time.sleep(timeStep)

    return targetSpeed

if __name__ == "__main__":
    main()
