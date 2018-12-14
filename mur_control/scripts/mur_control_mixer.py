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
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import WrenchStamped, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure

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
        self.pub_actuators = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
        self.sub_pose = message_filters.Subscriber('/mavros/local_position/pose', PoseStamped)
        self.sub_pres = message_filters.Subscriber('/mavros/imu/diff_pressure', FluidPressure)
        self.sub_force = message_filters.Subscriber('/control/force', WrenchStamped)
        self.ts = message_filters.TimeSynchronizer([self.sub_pose, self.sub_pres, self.sub_force], 10)
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

    def cmd_force_callback(self, msg_pose, msg_pres, msg_force):
        # Get the position and velocities
        self.pose_rot = np.array([msg_pose.pose.orientation.x, msg_pose.pose.orientation.y, msg_pose.pose.orientation.z, msg_pose.pose.orientation.w])
        #rospy.loginfo("Pos := \n %s" %self.pose_pos)
        self.J = mur_common.convert_body_world(self.pose_rot)
        self.force_attitude = self.get_force_callback(msg_force)
        tau = self.force_attitude
        # Thruster forces
        Tt = np.matmul(np.transpose(self.J),tau)
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
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
