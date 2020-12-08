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

def gps_to_ecef(lat, lon, alt):
    rad_lat = lat * (math.pi / 180.0) # convert to radians
    rad_lon = lon * (math.pi / 180.0) # covert to radians

    a = 6378137.0 # Radius of the Earth (in meters)
    finv = 298.257223563 # Flattening factor WGS84 Model
    f = 1 / finv
    e2 = 1 - (1 - f) * (1 - f)
    v = a / math.sqrt(1 - e2 * math.sin(rad_lat) * math.sin(rad_lat))

    x = (v + alt) * math.cos(rad_lat) * math.cos(rad_lon)
    y = (v + alt) * math.cos(rad_lat) * math.sin(rad_lon)
    z = (v * (1 - e2) + alt) * math.sin(rad_lat)

    return x, y, z

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
            alt = float(getattr(report, 'alt', 0.0))
            northings, eastings, vertical = gps_to_ecef(lat, lon, alt)
            pub.publish(lat, lon, northings, eastings)
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass