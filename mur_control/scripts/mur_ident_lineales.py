#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
from common import mur_common, mur_PID
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Vector3Stamped, Quaternion
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class MURIdentLinealesNode:
    def __init__(self):

        # ROS infrastructure
        self.sub_cmd_pose = rospy.Subscriber('/mavros/imu/data', Imu, self.cmd_pose_callback)
        self.pub_cmd_angle = rospy.Publisher('/mur/angles', Vector3Stamped, queue_size=10)

    def cmd_pose_callback(self, msg):
        # Get the position and velocities
        self.pose_rot = np.array([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
        # Convert to SNAME Position
        self.nita = np.rad2deg(euler_from_quaternion(self.pose_rot))
        rospy.loginfo("Angles :=%s", self.nita)
        # Get position error
        angle_msg = Vector3Stamped()
        angle_msg.header.stamp = rospy.Time.now()
        angle_msg.header.frame_id = 'mur/body'
        angle_msg.vector.x = self.nita[0]
        angle_msg.vector.y = self.nita[1]
        angle_msg.vector.z = self.nita[2]
        # To publish the message
        self.pub_cmd_angle.publish(angle_msg)


if __name__ == '__main__':
    rospy.init_node('mur_ident_lineales')
    try:
        node = MURIdentLinealesNode()
        rospy.Rate(20)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')

