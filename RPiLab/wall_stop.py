# wallstop.py
import time
import brickpi3
import grovepi

BP = brickpi3.BrickPi3()

ultrasonic_sensor_port = 4

try:
	while grovepi.ultrasonicRead(ultrasonic_sensor_port) > 15:
		print("Sensor: %6d Motor A: %6d  B: %6d  C: %6d  D: %6d" \
			% (grovepi.ultrasonicRead(ultrasonic_sensor_port), \
				BP.get_motor_encoder(BP.PORT_A), \
				BP.get_motor_encoder(BP.PORT_B), \
				BP.get_motor_encoder(BP.PORT_C), \
				BP.get_motor_encoder(BP.PORT_D)))
		BP.set_motor_power(BP.PORT_A+BP.PORT_D,30)
except IOError as error:
	print(error)
except TypeError as error:
	print(error)
except KeyboardInterrupt:
	print("You pressed ctrl+C...")


BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))
BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))
BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D))

BP.reset_all()