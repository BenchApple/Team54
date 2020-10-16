import time
import brickpi3
import grovepi

BP = brickpi3.BrickPi3()

ultrasonic = BP.PORT_A
BP.set_sensor_type(ultrasonic, BP.SENSOR_TYPE.NXT_ULTRASONIC) # assign ultrasonic sensor port to D4
right = (BP.PORT_C)
left = (BP.PORT_B)
touch = BP.PORT_D

BP.set_sensor_type(touch, BP.SENSOR_TYPE.TOUCH)  # Configure port 1 sensor type

def main():
    # Before touch sensor is pressed, your program will be stuck in this loop
    print("Press touch sensor on port 1 to run motors")
    value = 0
    while not value:
        try:
            value = BP.get_sensor(touch)
        except brickpi3.SensorError:
            value = 0
    print("Starting...")

    # Main logic    
    try:
        while True:
            # Our code goes here.
            currentPower = 0

            # Get distance from the ultrasonic\
            dist = BP.get_sensor(ultrasonic)

            time.sleep(.2) # hold each loop/iteration for .2 seconds

    except IOError as error:
        print(error)
    except TypeError as error:
        print(error)
    except KeyboardInterrupt:
        print("You pressed ctrl+C...")

    # use reset_all() to return all motors and sensors to resting states
    BP.reset_all()

# Takes the brickpi, right id, left id, and current speed to accelerate to the max speed.
def accelToMax(bp, right, left, curSpeed):
    maxPower = 30

    for i in range(curSpeed, maxPower):
        bp.set_motor_power(right, i)
        bp.set_motor_power(left, i)

        time.sleep(.2)

    return maxPower

def decelToSlow(bp, right, left, curSpeed):
    targetPower = 10

    direc = 1
    if targetPower < curSpeed:
        direc = -1

    for i in range(curSpeed, targetPower, direc):
        bp.set_motor_power(right, i)
        bp.set_motor_power(left, i)

        time.sleep(.2)

def turnRight(bp, right, left, curSpeed):
    curSpeed = decelToSlow(bp, right, left, curSpeed)

    bp.set_motor_power(right, curSpeed * 2)
    
    time.sleep(1)

    bp.set_motor_power(right, curSpeed)

    return curSpeed

def stop(bp, right, left, curSpeed):
    for i in range(curSpeed, 0, -1):
        bp.set_motor_power(right, i)
        bp.set_motor_power(left, i)

    bp.set_motor_power(right, i)
    bp.set_motor_power(left, i)

    return 0

def normal(bp, right, left, curSpeed):
    curSpeed = accelToMax(bp, right, left, curSpeed)
    return curSpeed

def withinCruiseDist(bp, right, left, curSpeed):
    curSpeed = decelToSlow(bp, right, left, curSpeed)
    return curSpeed

def crashAvoid(bp, right, left, curSpeed):
    curSpeed = decelToSlow(bp, right, left, curSpeed)

    curSpeed = turnRight(bp, right, left, curSpeed)

    time.sleep(2)

    curSpeed = stop(bp, right, left, curSpeed)

    return curSpeed

if __name__ == "__main__":
    main()
