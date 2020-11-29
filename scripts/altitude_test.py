#!/usr/bin/env python
"""
	Reads distance from the laser based VL53L1X,
    converts to meters, and publishes on the Altitude_msg
    type on the altitude_data topic
"""

#import qwiic
import qwiic_vl53l1x 
import time
import sys

def altitude_publisher():
    ToF = qwiic_vl53l1x.QwiicVL53L1X()
    if (ToF.sensor_init() == None):					 # Begin returns 0 on a good init
        run = True
        print "Sensor online!"
        ToF.set_distance_mode(2)            # Sets distance mode to Long
    else:
        print "Sensor not detected!"
        run = False
    while run:
        ToF.start_ranging()				# Write configuration bytes to initiate measurement
        time.sleep(.005)
        distance = ToF.get_distance()   # Get the result of the measurement from the sensor
        time.sleep(.005)
        ToF.stop_ranging()
        distance = float(distance*0.001)  # Convert to meters
        print distance
        time.sleep(0.25)


if __name__ == '__main__':
    altitude_publisher()