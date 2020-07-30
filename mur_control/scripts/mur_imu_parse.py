#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
from common import mur_common
from sensor_msgs.msg import Imu

class MURImuParse():
    def __init__(self):

        # ROS infrastructure
        self.sub_imu = rospy.Subscriber('/mavros/imu/data', Imu, self.call_imu)
        self.pub_imu = rospy.Publisher('/mur/imu/data', Imu, queue_size=1)

    def call_imu(self, msg_imu_int):
        msg_imu = Imu();
        msg_imu.header.stamp = rospy.Time.now()
        msg_imu.header.frame_id = "imu_link"
        r,p,y = euler_from_quaternion([msg_imu_int.orientation.x,msg_imu_int.orientation.y,msg_imu_int.orientation.z,msg_imu_int.orientation.w])
        self.roll = -p
        self.pitch = r
        self.yaw = y
        q_new = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        #rospy.loginfo(q_new)
        rospy.loginfo(euler_from_quaternion(q_new))
        msg_imu.orientation.x = q_new[0]
        msg_imu.orientation.y = q_new[1]
        msg_imu.orientation.z = q_new[2]
        msg_imu.orientation.w = q_new[3]
        msg_imu.orientation_covariance = msg_imu_int.orientation_covariance
        msg_imu.angular_velocity.x = msg_imu_int.angular_velocity.y
        msg_imu.angular_velocity.y = msg_imu_int.angular_velocity.x
        msg_imu.angular_velocity.z = -msg_imu_int.angular_velocity.z
        msg_imu.angular_velocity_covariance = msg_imu_int.angular_velocity_covariance
        msg_imu.linear_acceleration.x = msg_imu_int.linear_acceleration.y # The same configuration in the PIXHAWK
        msg_imu.linear_acceleration.y = -msg_imu_int.linear_acceleration.x
        msg_imu.linear_acceleration.z = -msg_imu_int.linear_acceleration.z
        msg_imu.linear_acceleration_covariance = msg_imu_int.linear_acceleration_covariance
        self.pub_imu.publish(msg_imu)


if __name__ == '__main__':
    rospy.init_node('mur_imu_parse')
    try:
        node = MURImuParse()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
