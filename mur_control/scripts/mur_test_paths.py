#! /usr/bin/env python

import numpy as np
import time
import rospy
import logging
import sys
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mavros_msgs.srv import CommandBool, CommandBoolResponse

class MURTestPaths():
    def __init__(self):
        # MODEL
        self.total_time=180
        self.acc_time=20

        #ROS INFRASTRUCTURE
        self.stabilized_srv = rospy.Service('/mur/path/stabilized', CommandBool, self.stabilized_req)
        self.simple_srv = rospy.Service('/mur/path/simple', CommandBool, self.simple_req)
        self.pub_cmd = rospy.Publisher('/mur/cmd_pose', PoseStamped, queue_size=1)

    def stabilized_req(self, req):
        segments = 3
        n_points = 20
        z_path=-1.0
        z_d=0
        for i in range(segments):
            rospy.loginfo("Trajectory Phase: %s", i+1)
            if i == 1:
                delta_path = 0.0
                delta_time = (self.total_time-2*self.acc_time)/n_points
            elif i == 2:
                delta_path = -z_path/n_points
                delta_time = self.acc_time/n_points
            else:
                delta_path = z_path/n_points
                delta_time = self.acc_time/n_points
            for j in range(int(n_points)):
                z_d=z_d+delta_path
                self.pub_pose(z_d,0.0, 0.0, 0.0)
                d =rospy.Duration(delta_time,0)
                rospy.sleep(d)
        rospy.loginfo("Trajectory Finished")
        return CommandBoolResponse(True,0)

    def simple_req(self, req):
        segments = 8
        n_points = 20
        z_path=-1.0
        pitch_path=1.0
        roll_path=1.0
        z_d=0.0
        p_d=0.0
        r_d=0.0
        for i in range(segments):
            rospy.loginfo("Trajectory Phase: %s", i+1)
            if i == 0:
                delta_z = z_path/n_points
                delta_p = 0.0
                delta_r = 0.0
                delta_time = self.acc_time/n_points
            elif i == segments-1:
                delta_z = -z_path/n_points
                delta_p = 0.0
                delta_r = 0.0
                delta_time = self.acc_time/n_points
            elif i == 1 or i == segments-2:
                delta_z = 0.0
                delta_p = 0.0
                delta_r = 0.0
                delta_time = self.acc_time/n_points
            else:
                delta_z = 0.0
                delta_p = (2*pitch_path)/n_points
                delta_r = (2*roll_path)/n_points
                delta_time = (self.total_time-2*self.acc_time)/((segments-4)*n_points)
            for j in range(int(n_points)):
                z_d=z_d+delta_z
                if i==2:
                    if j>((n_points-2)/2):
                        r_d=r_d-delta_r
                    else:
                        r_d=r_d+delta_r
                if i==3:
                    if j>((n_points-2)/2):
                        p_d=p_d-delta_p
                    else:
                        p_d=p_d+delta_p
                if i==4:
                    if j>((n_points-2)/2):
                        r_d=r_d+delta_r
                    else:
                        r_d=r_d-delta_r
                if i==5:
                    if j>((n_points-2)/2):
                        p_d=p_d+delta_p
                    else:
                        p_d=p_d-delta_p
                self.pub_pose(z_d,p_d, r_d, 0.0)
                d =rospy.Duration(delta_time,0)
                rospy.sleep(d)
        rospy.loginfo("Trajectory Finished")
        return CommandBoolResponse(True, 0)

    def pub_pose(self,z_d, pitch_d, roll_d, yaw_d):
        msg_pose = PoseStamped()
        msg_pose.header.stamp = rospy.Time.now()
        msg_pose.header.frame_id = 'world'
        msg_pose.pose.position.z = z_d
        q = quaternion_from_euler(pitch_d, roll_d, yaw_d)
        msg_pose.pose.orientation.x = q[0]
        msg_pose.pose.orientation.y = q[1]
        msg_pose.pose.orientation.z = q[2]
        msg_pose.pose.orientation.w = q[3]
        self.pub_cmd.publish(msg_pose)














if __name__ == '__main__':
    np.set_printoptions(suppress=True)
    rospy.init_node('mur_test_paths')
    try:
        node = MURTestPaths()
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
