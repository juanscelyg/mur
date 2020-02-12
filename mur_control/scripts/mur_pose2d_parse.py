#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
import message_filters
from common import mur_common
from geometry_msgs.msg import WrenchStamped, PoseStamped, AccelStamped, PoseWithCovarianceStamped

class MURPose2DParse():
    def __init__(self):

        # ROS infrastructure
        self.sub_pose2d = rospy.Subscriber('/pose_stamped', PoseStamped, self.call_pose)
        self.pub_pose = rospy.Publisher('/pose2D_stamped', PoseWithCovarianceStamped, queue_size=2)

    def call_pose(self, msg_pose):
        pose_stamped_msg = PoseWithCovarianceStamped()
        pose_stamped_msg.header.stamp = msg_pose.header.stamp
        pose_stamped_msg.header.frame_id = msg_pose.header.frame_id
        pose_stamped_msg.pose.pose.position.x = msg_pose.pose.position.x
        pose_stamped_msg.pose.pose.position.y = msg_pose.pose.position.y
        pose_stamped_msg.pose.pose.orientation.x = msg_pose.pose.orientation.x
        pose_stamped_msg.pose.pose.orientation.y = msg_pose.pose.orientation.y
        pose_stamped_msg.pose.pose.orientation.z = msg_pose.pose.orientation.z
        pose_stamped_msg.pose.pose.orientation.w = msg_pose.pose.orientation.w
        # rospy.loginfo("Depth :=\n %s" %depth_msg.pose.pose.position)
        # To publish the message
        self.pub_pose.publish(pose_stamped_msg)


if __name__ == '__main__':
    rospy.init_node('mur_pose2d_parse')
    try:
        node = MURPose2DParse()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
