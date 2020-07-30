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

class MURStopMotors():
    def __init__(self):

        # ROS infrastructure
        self.pub_force = rospy.Publisher('/mur/force_input', WrenchStamped, queue_size=5)

        # Callback
        self.epochs = 2
        for i in range(self.epochs):
            self.shutdown_motors()
        rospy.logwarn("The motors have been stopped by themselves.")

    def shutdown_motors(self):
        force_msg = WrenchStamped()
        force_msg.header.stamp = rospy.Time.now()
        force_msg.header.frame_id = '/mur/base_link'
        force_msg.wrench.force.x = 0.0
        force_msg.wrench.force.y = 0.0
        force_msg.wrench.force.z = 0.0
        force_msg.wrench.torque.x = 0.0
        force_msg.wrench.torque.y = 0.0
        force_msg.wrench.torque.z = 0.0
        # rospy.loginfo("Depth :=\n %s" %depth_msg.pose.pose.position)
        # To publish the message
        self.pub_force.publish(force_msg)


if __name__ == '__main__':
    rospy.init_node('mur_stop_motors')
    try:
        node = MURStopMotors()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
