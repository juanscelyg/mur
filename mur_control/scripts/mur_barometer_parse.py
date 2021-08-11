#! /usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
import tf2_ros
import message_filters
from common import mur_common
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped, PoseWithCovarianceStamped, TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Imu

class MURBarometerParse():
    def __init__(self):
        #Globals
        self.z_1 = 0.0
        self.t_1 = 0.0

        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0
        self.q_w = 1.0

        # ROS infrastructure
        self.sub_pres = rospy.Subscriber('/mavros/imu/diff_pressure', FluidPressure, self.call_pres)
        self.sub_imu = rospy.Subscriber('/mur/imu/data', Imu, self.get_imu)
        self.pub_depth = rospy.Publisher('/mur/depth/pose', PoseWithCovarianceStamped, queue_size=1)
        self.pub_vel = rospy.Publisher('/mur/depth/velocity', TwistStamped, queue_size=1)

    def get_imu(self, msg_imu):
        self.q_x = msg_imu.orientation.x
        self.q_y = msg_imu.orientation.y
        self.q_z = msg_imu.orientation.z
        self.q_w = msg_imu.orientation.w


    def call_pres(self, msg_pres):
        # DEPTH
        depth_msg = PoseWithCovarianceStamped()
        depth_msg.header.stamp = rospy.Time.now()
        depth_msg.header.frame_id = 'world'
        depth_msg.pose.pose.position.z = mur_common.pressure_to_meters(msg_pres.fluid_pressure) # barometer
        depth_msg.pose.pose.orientation.x = self.q_x
        depth_msg.pose.pose.orientation.y = self.q_y
        depth_msg.pose.pose.orientation.z = self.q_z
        depth_msg.pose.pose.orientation.w = self.q_w
        self.pub_depth.publish(depth_msg)
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.frame_id = "world"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "odom"
        t.transform.translation.z = depth_msg.pose.pose.position.z
        t.transform.rotation.x = self.q_x
        t.transform.rotation.y = self.q_y
        t.transform.rotation.z = self.q_z
        t.transform.rotation.w = self.q_w
        br.sendTransform(t)
        # VELOCITY
        t_now = rospy.get_time()
        dt = t_now - self.t_1
        vel_msg = TwistStamped()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.header.frame_id = 'world'
        vel_msg.twist.linear.z = (depth_msg.pose.pose.position.z - self.z_1)/dt
        self.pub_vel.publish(vel_msg)
        # next iteration
        self.z_1 = depth_msg.pose.pose.position.z
        self.t_1 = t_now



if __name__ == '__main__':
    rospy.init_node('mur_barometer_parse')
    try:
        node = MURBarometerParse()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
