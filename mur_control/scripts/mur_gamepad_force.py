#!/usr/bin/env python

import numpy as np
import math
import rospy
import logging
import sys
import tf
import tf2_ros
from common import mur_common
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from geometry_msgs.msg import WrenchStamped, PoseStamped, TransformStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry

class MURGamepadForceNode():
    def __init__(self):
        # Init constants
        self.pitch_gain = 350
        self.roll_gain =  350
        self.yaw_gain = 350
        self.z_gain = 350

        # ROS infraestucture
        self.pub_actuators = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.get_values)


    def get_values(self, msg_joy):
        pitch_d = msg_joy.axes[0]*self.pitch_gain+1500
        roll_d  = msg_joy.axes[1]*self.roll_gain+1500
        yaw_d   = msg_joy.axes[3]*self.yaw_gain+1500
        z_d     = msg_joy.axes[4]*self.z_gain+1500
        msg_actu = OverrideRCIn()
        msg_actu.channels = np.array([pitch_d,roll_d,yaw_d,z_d,0,0,0,0])
        rospy.loginfo(msg_actu.channels)
        self.pub_actuators.publish(msg_actu)


if __name__ == '__main__':
    rospy.init_node('mur_gamepad_force')
    try:
        node = MURGamepadForceNode()
        rate = rospy.Rate(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
