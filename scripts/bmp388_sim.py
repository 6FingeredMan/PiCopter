#!/usr/bin/env python
import rospy
from picopter.msg import Sim_msg
from picopter.msg import Elevation_msg

class bmp388_sim:

    def __init__(self):
        self.elevation = 0.0
        self.start()

    def start(self):

        # start a subcriber to the sim topic
        self.sub = rospy.Subscriber("sim_data", Sim_msg, self.ros_callback)

        # start the publisher that publishes elevation data
        pub = rospy.Publisher('elevation_data', Elevation_msg, queue_size=1)
        rospy.init_node('elevation_sensor', anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pub.publish(self.elevation)
            rate.sleep()

    def ros_callback(self, data):
        self.elevation = data.elevation
        # TO DO - Add Noise to elevation data
        
if __name__ == '__main__':
    sensor = bmp388_sim()