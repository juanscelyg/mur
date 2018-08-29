#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
import message_filters
from common import mur_common
from dynamic_reconfigure.server import Server
from mur_control.cfg import MurControlMixerConfig
from mur_control.msg import FloatStamped
from geometry_msgs.msg import WrenchStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry

class MURControlMixerNode():

    def __init__(self):
        # Init constants
        self.num_thrusters = 4
        self.angle = 0
        self.x_bar = 0.197
        self.y_bar = 0.18022

        # Desire parameters
        self.saturation = 30.0

        # Variables
        self.force_vel = np.zeros(shape=(6,1))
        self.force_yaw = np.zeros(shape=(6,1))
        self.force_height = np.zeros(shape=(6,1))

        # Convert parameters
        self.T = self.get_t_matrix()

        # ROS parameter server
        self.config = {}

        # ROS infraestucture
        self.srv_reconfigure = Server(MurControlMixerConfig, self.config_callback)
        self.pub_thruster_0 = rospy.Publisher('/mur/thrusters/0/input', FloatStamped, queue_size=1)
        self.pub_thruster_1 = rospy.Publisher('/mur/thrusters/1/input', FloatStamped, queue_size=1)
        self.pub_thruster_2 = rospy.Publisher('/mur/thrusters/2/input', FloatStamped, queue_size=1)
        self.pub_thruster_3 = rospy.Publisher('/mur/thrusters/3/input', FloatStamped, queue_size=1)
        self.sub_pose = message_filters.Subscriber('/mur/pose_gt', Odometry)
        self.sub_fvel = message_filters.Subscriber('/control/Wrench/velocity', WrenchStamped)
        self.sub_fyaw = message_filters.Subscriber('/control/Wrench/yaw', WrenchStamped)
        self.sub_falt = message_filters.Subscriber('/control/Wrench/height', WrenchStamped)
        self.ts = message_filters.TimeSynchronizer([self.sub_pose, self.sub_fvel, self.sub_fyaw, self.sub_falt], 10)
        self.ts.registerCallback(self.cmd_force_callback)

    def config_callback(self, config, level):
        self.saturation = config['saturation']
        # To refresh the config value
        self.config = config
        # Return the config value
        return config

    def get_force_callback(self, msg):
        force = np.array([[msg.wrench.force.x], [msg.wrench.force.y], [msg.wrench.force.z], [msg.wrench.torque.x], [msg.wrench.torque.y], [msg.wrench.torque.z]])
        return force

    def get_t_matrix(self):
        t_matrix = np.matrix([[0, 0, 0, 0], [np.sin(np.deg2rad(self.angle)), np.sin(np.deg2rad(-self.angle)), np.sin(np.deg2rad(-self.angle)), np.sin(np.deg2rad(self.angle))], [1,1,1,1], [np.cos(np.deg2rad(self.angle))*self.y_bar, -np.cos(np.deg2rad(-self.angle))*self.y_bar, -np.cos(np.deg2rad(-self.angle))*self.y_bar, np.cos(np.deg2rad(self.angle))*self.y_bar], [-self.x_bar, -self.x_bar, -self.x_bar, -self.x_bar], [-np.sin(np.deg2rad(self.angle))*self.x_bar, np.sin(np.deg2rad(-self.angle))*self.x_bar, -np.sin(np.deg2rad(-self.angle))*self.x_bar, np.sin(np.deg2rad(self.angle))*self.x_bar]])
        return t_matrix

    def set_force_thrusters(self):
        msg_thruster_0 = FloatStamped()
        msg_thruster_1 = FloatStamped()
        msg_thruster_2 = FloatStamped()
        msg_thruster_3 = FloatStamped()
        msg_thruster_0.header.stamp = rospy.Time.now()
        msg_thruster_0.header.frame_id = 'mur/thrusters/0'
        msg_thruster_1.header.stamp = rospy.Time.now()
        msg_thruster_1.header.frame_id = 'mur/thrusters/1'
        msg_thruster_2.header.stamp = rospy.Time.now()
        msg_thruster_2.header.frame_id = 'mur/thrusters/2'
        msg_thruster_3.header.stamp = rospy.Time.now()
        msg_thruster_3.header.frame_id = 'mur/thrusters/3'
        msg_thruster_0.data = self.thrusters[0]
        msg_thruster_1.data = self.thrusters[1]
        msg_thruster_2.data = self.thrusters[2]
        msg_thruster_3.data = self.thrusters[3]
        self.pub_thruster_0.publish(msg_thruster_0)
        self.pub_thruster_1.publish(msg_thruster_1)
        self.pub_thruster_2.publish(msg_thruster_2)
        self.pub_thruster_3.publish(msg_thruster_3)

    def saturator_thruster(self,force):
        thrusters = np.empty_like(force)
        for i in range(len(force)):
            if force[i]<=-self.saturation:
                thrusters[i]=(-self.saturation)
                rospy.logwarn("The motor %s was min saturated %s",i,force[i])
            elif force[i]>=self.saturation:
                thrusters[i]=self.saturation
                rospy.logwarn("The motor %s was max saturated %s",i,force[i])
            else:
                thrusters[i]=force[i]
        return thrusters


    def cmd_force_callback(self, msg_pose, msg_fvel, msg_fyaw, msg_falt):
        # Get the position and velocities
        self.pose_pos = np.array([msg_pose.pose.pose.position.x, msg_pose.pose.pose.position.y, msg_pose.pose.pose.position.z])
        self.pose_rot = np.array([msg_pose.pose.pose.orientation.x,msg_pose.pose.pose.orientation.y,msg_pose.pose.pose.orientation.z,msg_pose.pose.pose.orientation.w])
        #rospy.loginfo("Pos := \n %s" %self.pose_pos)
        self.J = mur_common.convert_body_world(self.pose_rot)
        self.force_vel = self.get_force_callback(msg_fvel)
        self.force_yaw = self.get_force_callback(msg_fyaw)
        self.force_height = self.get_force_callback(msg_falt)
        tau = self.force_height + self.force_vel #+ self.force_yaw + self.force_vel
        # Thrusters matrix on the world frame
        A = np.linalg.inv(np.transpose(self.J))
        Tt = np.matmul(A,self.T)
        thrusters_forces = np.matmul(np.transpose(Tt),tau)
        rospy.loginfo("tau := \n %s" %tau)
        rospy.loginfo("thrusters_forces := \n %s" %thrusters_forces)
        self.thrusters = self.saturator_thruster(thrusters_forces)
        self.set_force_thrusters()

if __name__ == '__main__':
    rospy.init_node('mur_control_mixer')
    try:
        node = MURControlMixerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
