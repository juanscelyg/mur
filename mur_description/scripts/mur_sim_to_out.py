#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
import message_filters
from simtools import mur_simtools
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped,Vector3, Quaternion, Pose
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Imu

class MURSimToOutNode:
    def __init__(self):
        # ROS infrastructure
        self.pub_pose = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=1)
        self.pub_pres = rospy.Publisher('/mavros/imu/pressure', FluidPressure, queue_size=1)
        self.pub_imu = rospy.Publisher('/mavros/imu/data', Imu, queue_size=1)
        self.sub_pose = message_filters.Subscriber('/mur/pose_gt', Odometry)
        self.sub_pres = message_filters.Subscriber('/mur/pressure', FluidPressure)
        self.sub_imu = message_filters.Subscriber('/mur/imu', Imu)
        self.ts = message_filters.TimeSynchronizer([self.sub_pose, self.sub_pres, self.sub_imu], 10)
        self.ts.registerCallback(self.cmd_sync_callback)

    def cmd_sync_callback(self, msg_pose_int, msg_pres_int, msg_imu_int):
        msg_pose = PoseStamped();
        msg_pose.header.stamp = rospy.Time.now()
        msg_pose.header.frame_id = 'mur/imu'
        msg_pose.pose = msg_pose_int.pose.pose;
        self.pub_pose.publish(msg_pose)

        msg_pres = FluidPressure();
        msg_pres.header.stamp = rospy.Time.now()
        msg_pres.header.frame_id = 'mur/barometer'
        msg_pres.fluid_pressure = msg_pres_int.fluid_pressure;
        self.pub_pres.publish(msg_pres)

        msg_imu = Imu();
        msg_imu.header.stamp = rospy.Time.now()
        msg_imu.header.frame_id = 'mur/imu'
        msg_imu.orientation = msg_imu_int.orientation;
        msg_imu.angular_velocity = msg_imu_int.angular_velocity;
        msg_imu.linear_acceleration = msg_imu_int.linear_acceleration;
        self.pub_imu.publish(msg_imu)

if __name__ == '__main__':
    rospy.init_node('mur_sim_to_out_node')
    try:
        node = MURSimToOutNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')

