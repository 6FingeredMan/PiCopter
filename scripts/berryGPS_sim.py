#!/usr/bin/env python
"""
	Simulates the BerryGPS-IMU Module by processing the sim_data topic.
    Publishes on the Gps_msg type on the gps_data topic
"""
import ConfigParser
from gps import *
import time
import rospy
from picopter.msg import Gps_msg
from picopter.msg import Sim_msg

class gps_module:

    def __init__(self):
        config = ConfigParser.RawConfigParser(allow_no_value=True)
        config.read('/root/ros_catkin_ws/src/picopter/config/simulation.ini')
        self.pub_rate = config.getfloat("GPS", "update rate (Hz)")
        
        self.start()

    def ros_callback(self, data):
        self.lat = data.latitude
        self.lon = data.longitude
        self.northings = data.northings
        self.eastings = data.eastings

    def start(self):
        self.sub = rospy.Subscriber('sim_data', Sim_msg, self.ros_callback)
        self.pub = rospy.Publisher('gps_data', Gps_msg, queue_size=1)
        rospy.init_node('gps_module', anonymous=True)
        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            self.pub.publish(self.lat, self.lon, self.northings, self.eastings)
            rate.sleep()

if __name__ == '__main__':
    sensor = gps_module()