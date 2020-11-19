#!/usr/bin/env python
"""
	Reads distance from the laser based VL53L1X,
    converts to meters, and publishes on the Altitude_msg
    type on the altitude_data topic
"""

import qwiic
import time
import rospy
from picopter.msg import Altitude_msg

def altitude_publisher():
    ToF = qwiic.QwiicVL53L1X()
    ToF.set_distance_mode(2)            # Sets distance mode to Long
    pub = rospy.Publisher('altitude_data', Altitude_msg, queue_size=1)
    rospy.init_node('altitude_sensor', anonymous=True)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        ToF.start_ranging()				# Write configuration bytes to initiate measurement
        time.sleep(.005)
        distance = ToF.get_distance()   # Get the result of the measurement from the sensor
        time.sleep(.005)
        ToF.stop_ranging()
        distance = float(distance*0.001)  # Convert to meters
        pub.publish(distance)
        rate.sleep()


if __name__ == '__main__':
    try:
        altitude_publisher()
    except rospy.ROSInterruptException:
        pass