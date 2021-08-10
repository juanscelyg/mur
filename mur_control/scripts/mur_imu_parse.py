#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
from common import mur_common
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, TransformStamped, PoseWithCovarianceStamped

class MURImuParse():
    def __init__(self):
        self.z = 0.0

        # ROS infrastructure
        self.sub_imu = rospy.Subscriber('/mavros/imu/data', Imu, self.call_imu)
        self.pub_depth = rospy.Subscriber('/mur/depth/pose', PoseWithCovarianceStamped, self.call_pose)
        self.pub_imu = rospy.Publisher('/mur/imu/data', Imu, queue_size=1)
        self.pub_ori = rospy.Publisher('/mur/angular/orientation', PointStamped, queue_size=1)
        self.pub_omega = rospy.Publisher('/mur/angular/velocity', PointStamped, queue_size=1)

    def call_pose(self, msg_pose):
        self.z=msg_pose.pose.pose.position.z

    def call_imu(self, msg_imu_int):
        # IMU
        msg_imu = Imu();
        msg_imu.header.stamp = rospy.Time.now()
        msg_imu.header.frame_id = "world"
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
        msg_imu.angular_velocity.x = -msg_imu_int.angular_velocity.x
        msg_imu.angular_velocity.y = -msg_imu_int.angular_velocity.y
        msg_imu.angular_velocity.z = msg_imu_int.angular_velocity.z
        msg_imu.angular_velocity_covariance = msg_imu_int.angular_velocity_covariance
        msg_imu.linear_acceleration.x = msg_imu_int.linear_acceleration.y # The same configuration in the PIXHAWK
        msg_imu.linear_acceleration.y = -msg_imu_int.linear_acceleration.x
        msg_imu.linear_acceleration.z = msg_imu_int.linear_acceleration.z
        msg_imu.linear_acceleration_covariance = msg_imu_int.linear_acceleration_covariance
        self.pub_imu.publish(msg_imu)
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.frame_id = "world"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "odom"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.z
        t.transform.rotation.x = q_new[0]
        t.transform.rotation.y = q_new[1]
        t.transform.rotation.z = q_new[2]
        t.transform.rotation.w = q_new[3]
        br.sendTransform(t)
        # ORIENTATION
        msg_ori = PointStamped()
        msg_ori.header.stamp = rospy.Time.now()
        msg_ori.header.frame_id = "world"
        msg_ori.point.x = self.pitch
        msg_ori.point.y = self.roll
        msg_ori.point.z = self.yaw
        self.pub_ori.publish(msg_ori)
        # ANGULAR VELOCITY
        msg_vel = PointStamped()
        msg_vel.header.stamp = rospy.Time.now()
        msg_vel.header.frame_id = "world"
        msg_vel.point.x = msg_imu.angular_velocity.x
        msg_vel.point.y = msg_imu.angular_velocity.y
        msg_vel.point.z = msg_imu.angular_velocity.z
        self.pub_omega.publish(msg_vel)


if __name__ == '__main__':
    rospy.init_node('mur_imu_parse')
    try:
        node = MURImuParse()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
