#!/usr/bin/env python

import numpy as np
import math
import rospy
import logging
import sys
import tf
import tf2_ros
from common import mur_common
from sensor_msgs.msg import Joy
from geometry_msgs.msg import WrenchStamped, PoseStamped, TransformStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry

class MURGamepadParseNode():
    def __init__(self):
        # Init constants
        self.pitch_gain = -0.95
        self.roll_gain = -0.95
        self.yaw_gain = 0.15
        self.z_gain = 0.005
        self.yaw_1 = 0.0
        self.z_1 = -0.1
        self.z_offset = -1.65 # Max Depth

        # ROS infraestucture
        self.pub_pose = rospy.Publisher('/mur/cmd_pose', PoseStamped, queue_size=1)
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.get_values)


    def get_values(self, msg_joy):
        pitch_d = msg_joy.axes[0]*self.pitch_gain
        roll_d  = msg_joy.axes[1]*self.roll_gain
        # Yaw Part
        yaw_d   = msg_joy.buttons[3]*self.yaw_gain-msg_joy.buttons[1]*self.yaw_gain+self.yaw_1
        if yaw_d>np.pi:
            yaw_d=np.pi
        elif yaw_d<-np.pi:
            yaw_d=-np.pi
        # Z Part
        z_d     = msg_joy.axes[4]*self.z_gain+self.z_1
        if z_d>0.1:
            z_d=-0.1
        elif z_d<self.z_offset:
            z_d=self.z_offset
        # Up to zero level
        if msg_joy.buttons[4] == 1:
            z_d = -0.10
        # To go down until -1.0 meter
        if msg_joy.buttons[5] == 1:
            z_d = -1.0
        if msg_joy.buttons[2] == 1:
            z_d = -0.50
        msg_pose = PoseStamped()
        msg_pose.header.stamp = rospy.Time.now()
        msg_pose.header.frame_id = 'odom'
        msg_pose.pose.position.z = z_d
        q = quaternion_from_euler(pitch_d, roll_d, yaw_d)
        msg_pose.pose.orientation.x = q[0]
        msg_pose.pose.orientation.y = q[1]
        msg_pose.pose.orientation.z = q[2]
        msg_pose.pose.orientation.w = q[3]
        self.pub_pose.publish(msg_pose)
        self.yaw_1=yaw_d
        self.z_1 = z_d


if __name__ == '__main__':
    rospy.init_node('mur_gamepad_parse')
    try:
        node = MURGamepadParseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
