#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
import tf2_ros
import message_filters
from common import mur_common
from dynamic_reconfigure.server import Server
from mur_control.cfg import MurControlMixerConfig
from mur_control.msg import FloatStamped
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import WrenchStamped, PoseStamped, TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Imu

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
        self.pose_rot = np.array([0,0,0,1])
        self.J = mur_common.convert_body_world(self.pose_rot)

        # Convert parameters
        self.T = self.get_t_matrix()

        # ROS parameter server
        self.config = {}

        # ROS infraestucture
        self.srv_reconfigure = Server(MurControlMixerConfig, self.config_callback)
        self.pub_actuators = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
        self.sub_odometry = rospy.Subscriber('/mur/odom_filtered', Odometry, self.get_odometry)
        self.sub_force = rospy.Subscriber('/mur/force_input', WrenchStamped, self.cmd_force_callback)

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
        t_matrix = np.matrix([[0, 0, 0, 0],
            [np.sin(np.deg2rad(self.angle)), np.sin(np.deg2rad(-self.angle)), np.sin(np.deg2rad(-self.angle)), np.sin(np.deg2rad(self.angle))],
            [1,1,1,1],
            [np.cos(np.deg2rad(self.angle))*self.y_bar, -np.cos(np.deg2rad(-self.angle))*self.y_bar, -np.cos(np.deg2rad(-self.angle))*self.y_bar, np.cos(np.deg2rad(self.angle))*self.y_bar],
            [-self.x_bar, -self.x_bar, self.x_bar, self.x_bar],
            [-np.sin(np.deg2rad(self.angle))*self.x_bar, np.sin(np.deg2rad(-self.angle))*self.x_bar, -np.sin(np.deg2rad(-self.angle))*self.x_bar, np.sin(np.deg2rad(self.angle))*self.x_bar]])
        return t_matrix

    def set_force_thrusters(self):
        msg_actuators = OverrideRCIn()
        msg_actuators.channels = np.array([mur_common.push_to_pwm(self.thrusters[0]),mur_common.push_to_pwm(self.thrusters[1]),mur_common.push_to_pwm(self.thrusters[2]),mur_common.push_to_pwm(self.thrusters[3]),0,0,0,0])
        self.pub_actuators.publish(msg_actuators)

    def saturator_thruster(self,force):
        thrusters = np.empty_like(force)
        for i in range(len(force)):
            if force[i]<=-self.saturation:
                thrusters[i]=(-self.saturation)
                rospy.logwarn("The motor %s is min saturated %s",i,force[i])
            elif force[i]>=self.saturation:
                thrusters[i]=self.saturation
                rospy.logwarn("The motor %s is max saturated %s",i,force[i])
            else:
                thrusters[i]=force[i]
        return thrusters

    def get_odometry(self, msg_pose):
        # Get the position and velocities
        self.pose_rot = np.array([msg_pose.pose.pose.orientation.x, msg_pose.pose.pose.orientation.y, msg_pose.pose.pose.orientation.z, msg_pose.pose.pose.orientation.w])
        #rospy.loginfo("Pos := \n %s" %self.pose_pos)
        self.J = mur_common.convert_body_world(self.pose_rot)


    def cmd_force_callback(self, msg_force):
        self.force_attitude = np.array([[msg_force.wrench.force.x], [msg_force.wrench.force.y], [msg_force.wrench.force.z], [msg_force.wrench.torque.x], [msg_force.wrench.torque.y], [msg_force.wrench.torque.z]])
        # Thruster forces
        Tt = np.matmul(np.transpose(self.J),self.force_attitude)
        B = np.linalg.pinv(self.T)
        thrusters_forces = np.matmul(B,Tt)
        self.thrusters = self.saturator_thruster(thrusters_forces)
        ### self.thrusters = thrusters_forces
        ### rospy.loginfo("Thrusters :=\n %s" %self.thrusters)
        self.set_force_thrusters()

if __name__ == '__main__':
    rospy.init_node('mur_control_mixer')
    try:
        node = MURControlMixerNode()
        rate = rospy.Rate(25)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
