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
        self.mode = 0
        # Forces
        self.pitch = 1500
        self.roll =  1500
        self.yaw = 1500
        self.z = 1500
        # Past forces
        self.pitch_1 = 1500
        self.roll_1 =  1500
        self.yaw_1 = 1500
        self.z_1 = 1500
        # Timer
        self.mytime = 0.3
        #limite
        self.limite=100;

        # ROS infraestucture
        self.pub_actuators = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.get_values)


    def get_values(self, msg_joy):
        self.mode = msg_joy.buttons[4]
        self.pitch = msg_joy.axes[0]*self.pitch_gain+1500
        self.roll  = msg_joy.axes[1]*self.roll_gain+1500
        self.yaw   = msg_joy.axes[3]*self.yaw_gain+1500
        self.z     = msg_joy.axes[4]*self.z_gain+1500
        if abs(self.pitch_1-self.pitch)>self.limite:
            if self.pitch ==1500:
                self.pitch == 1500
            else:
                self.pitch=self.pitch_1
        if abs(self.roll_1-self.roll)>self.limite:
            if self.roll ==1500:
                self.roll == 1500
            else:
                self.roll=self.roll_1
        if abs(self.yaw_1-self.yaw)>self.limite:
            if self.yaw ==1500:
                self.yaw == 1500
            else:
                self.yaw=self.yaw_1
        if abs(self.z_1-self.z)>self.limite:
            if self.z ==1500:
                self.z == 1500
            else:
                self.z=self.z_1
        rospy.loginfo("Modo: %u" %self.mode)
        rospy.loginfo("Valor: %u" %self.z)


    def set_force(self, event):
        msg_actu = OverrideRCIn()
        if self.mode==0:
            msg_actu.channels = np.array([self.pitch,self.roll,self.yaw,self.z,0,0,0,0])
        elif self.mode==1:
            msg_actu.channels = np.array([self.z,self.z,self.z,self.z,0,0,0,0])
        rospy.loginfo(msg_actu.channels)
        self.pub_actuators.publish(msg_actu)
        self.pitch_1 = self.pitch
        self.roll_1 =  self.roll
        self.yaw_1 = self.yaw
        self.z_1 = self.z


if __name__ == '__main__':
    rospy.init_node('mur_gamepad_force')
    try:
        node = MURGamepadForceNode()
        rospy.Timer(rospy.Duration(node.mytime), node.set_force)
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
