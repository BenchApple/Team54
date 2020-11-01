# BEn Chappell

# File to test the effectiveness of the drive.py file

import drive as drive
import brickpi3
import time

# Initlailize our brickpi object
bp = brickpi3.BrickPi3()

def main():
    try:
        # Initialize ports for motors
        right = bp.PORT_B
        left = bp.PORT_C
        steer = bp.PORT_D

        # Reset the motor encoders for everything
        #bp.reset_motor_encoder(right)
        #bp.reset_motor_encoder(left)
        #bp.reset_motor_encoder(steer)

        # Test the position stuff for the steering
        drive.setStraight(bp, steer)

        time.sleep(3)
        
        drive.rotateAxle(bp, steer, 100)

        time.sleep(3)
        
        print("All done")

        bp.reset_all()
        
    except KeyboardInterrupt:
        bp.reset_all()
    except NameError as error:
        print(error)
        bp.reset_all()
    except IOError as error:
        print(error)
        bp.reset_all()

if __name__ == "__main__":
    main()
