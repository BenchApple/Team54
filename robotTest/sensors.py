# Ben Chappell - Team 54
# Implements basic functions for interpreting sensor data and stuff.

import brickpi3
import grovepi
import time

# Gets the output of the ultrasonic at port port
def getUltrasonic(port):
    return grovepi.ultrasonicRead(port)
    
