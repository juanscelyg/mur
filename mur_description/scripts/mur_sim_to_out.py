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
        self.pub_pres = rospy.Publisher('/mavros/imu/diff_pressure', FluidPressure, queue_size=1)
        self.pub_imu = rospy.Publisher('/mavros/imu/data', Imu, queue_size=1)
        self.sub_pres = message_filters.Subscriber('/mur/pressure', FluidPressure)
        self.sub_imu = message_filters.Subscriber('/mur/imu', Imu)
        self.ts = message_filters.TimeSynchronizer([self.sub_pres, self.sub_imu], 10)
        self.ts.registerCallback(self.cmd_sync_callback)

    def cmd_sync_callback(self, msg_pres_int, msg_imu_int):
        msg_pres = FluidPressure();
        msg_pres.header.stamp = rospy.Time.now()
        msg_pres.header.frame_id = msg_pres_int.header.frame_id
        msg_pres.fluid_pressure = msg_pres_int.fluid_pressure
        self.pub_pres.publish(msg_pres)

        msg_imu = Imu();
        msg_imu.header.stamp = rospy.Time.now()
        msg_imu.header.frame_id = "base_link"
        msg_imu.orientation = msg_imu_int.orientation
        msg_imu.orientation_covariance = np.array([0,0,0,0,0,0,0,0,0])
        msg_imu.angular_velocity = msg_imu_int.angular_velocity
        msg_imu.angular_velocity_covariance = np.array([1.2184E-7,0.0,0.0,0.0,1.2184E-7,0.0,0.0,0.0,1.2184E-7])
        msg_imu.linear_acceleration.x = msg_imu_int.linear_acceleration.x # The same configuration in the PIXHAWK
        msg_imu.linear_acceleration.y = msg_imu_int.linear_acceleration.y
        msg_imu.linear_acceleration.z = msg_imu_int.linear_acceleration.z
        msg_imu.linear_acceleration_covariance = np.array([9.0E-8,0.0,0.0,0.0,9.0E-8,0.0,0.0,0.0,9.0E-8])
        self.pub_imu.publish(msg_imu)

if __name__ == '__main__':
    rospy.init_node('mur_sim_to_out_node')
    try:
        node = MURSimToOutNode()
        rate = rospy.Rate(50)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')

