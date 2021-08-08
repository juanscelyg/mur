#!/usr/bin/env python

import numpy as np
import math
import rospy
import logging
import sys
import tf
import tf2_ros
from common import mur_common
from sensor_msgs.msg import Joy, Imu
from geometry_msgs.msg import WrenchStamped, PoseStamped, TransformStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry

class MURGamepadParseNode():
    def __init__(self):
        # ROS Infraestructure
        self.mytime = 0.1

        # Init constants
        self.pitch_gain = 0.75
        self.roll_gain = -0.75
        self.yaw_gain = 0.2
        self.z_gain = 0.005
        self.yaw_1 = 1.0
        self.z_1 = -0.05
        self.z_offset = -1.65 # Max Depth
        self.flag = False

        # Raw values
        self.pitch_raw = 0.0
        self.roll_raw = 0.0
        self.yaw_raw_r = 0.0
        self.yaw_raw_l = 0.0
        self.z_raw_h = 0.0
        self.z_raw_d = 0.0
        self.special_option = 0

        # ROS infraestucture
        self.sub_imu = rospy.Subscriber('/mavros/imu/data', Imu, self.call_imu)
        self.pub_pose = rospy.Publisher('/mur/cmd_pose', PoseStamped, queue_size=1)
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.get_values)

    def call_imu(self, msg_imu_int):
        # IMU
        r,p,y = euler_from_quaternion([msg_imu_int.orientation.x,msg_imu_int.orientation.y,msg_imu_int.orientation.z,msg_imu_int.orientation.w])
        self.roll = -p
        self.pitch = r
        self.yaw = y


    def get_values(self, msg_joy):
        self.pitch_raw = msg_joy.axes[0]
        self.roll_raw = msg_joy.axes[1]
        self.yaw_raw_r = msg_joy.buttons[5]
        self.yaw_raw_l = msg_joy.buttons[4]
        self.z_raw_h = msg_joy.axes[2]
        self.z_raw_d = msg_joy.axes[5]
        #Special buttons
        if msg_joy.buttons[2] == 1:
            self.special_option = 2; # To surface
        if msg_joy.buttons[3] == 1:
            self.special_option = 3; # To add +0.5 to reference
        if msg_joy.buttons[1] == 1:
            self.special_option = 1; # To less -0.5 to reference
        if msg_joy.buttons[6] == 1 and msg_joy.buttons[7] == 1:
            self.flag = True # Unlocked
            rospy.logwarn("Gamepad unlocked")


    def pub_values(self, event):
        if self.flag == True:
            # Balance reference
            pitch_d = self.pitch_raw*self.pitch_gain
            roll_d  = self.roll_raw*self.roll_gain
            # Yaw Part
            yaw_d = self.yaw_raw_l*self.yaw_gain-self.yaw_raw_r*self.yaw_gain
            if yaw_d>np.pi:
                yaw_d=np.pi
            elif yaw_d<-np.pi:
                yaw_d=-np.pi
            # Z Joystick
            z_d = self.z_raw_h*self.z_gain - self.z_raw_d*self.z_gain + self.z_1
            # Special Options
            if self.special_option == 1:
                z_d = self.z_1 - 0.5
            if self.special_option == 2:
                z_d = -0.05
            if self.special_option == 3:
                z_d =  self.z_1 + 0.5
            if z_d>-0.01:
                z_d=-0.01
            elif z_d<self.z_offset:
                z_d=self.z_offset

            # To publish
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
            self.special_option = 0


if __name__ == '__main__':
    rospy.init_node('mur_gamepad_parse')
    try:
        node = MURGamepadParseNode()
        rospy.Timer(rospy.Duration(node.mytime), node.pub_values)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
