#!/usr/bin/env python
"""
	Reads gps data from the BerryGPS-IMU Module over serial,
    converts lat/lon to northings/eastings, and publishes
    on the Gps_msg type on the gps_data topic
"""
from gps import *
import time
import rospy
from picopter.msg import Gps_msg

def gps_publisher():
    gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE) 
    pub = rospy.Publisher('gps_data', Gps_msg, queue_size=1)
    rospy.init_node('gps_sensor', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        report = gpsd.next() #
        if report['class'] == 'TPV':
            lat = float(getattr(report,'lat',0.0))
            lon = float(getattr(report,'lon',0.0))
            northings = float(0.0)
            eastings = float(0.0)
            pub.publish(lat, lon, northings, eastings)
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass