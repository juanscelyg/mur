#! /usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
import message_filters
from common import mur_common
from geometry_msgs.msg import WrenchStamped, PoseStamped, AccelStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Imu

class MURBarometerParse():
    def __init__(self):

        # ROS infrastructure
        self.sub_pres = rospy.Subscriber('/mavros/imu/diff_pressure', FluidPressure, self.call_pres)
        self.pub_cmd_depth = rospy.Publisher('/mur/depth', PoseWithCovarianceStamped, queue_size=2)

    def call_pres(self, msg_pres):
        depth_msg = PoseWithCovarianceStamped()
        depth_msg.header.stamp = rospy.Time.now()
        depth_msg.header.frame_id = 'odom'
        depth_msg.pose.pose.position.z = mur_common.pressure_to_meters(msg_pres.fluid_pressure) # barometer
        # rospy.loginfo("Depth :=\n %s" %depth_msg.pose.pose.position)
        # To publish the message
        self.pub_cmd_depth.publish(depth_msg)


if __name__ == '__main__':
    rospy.init_node('mur_barometer_parse')
    try:
        node = MURBarometerParse()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
