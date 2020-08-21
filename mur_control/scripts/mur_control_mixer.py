#! /usr/bin/env python

import numpy as np
import time
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
from geometry_msgs.msg import WrenchStamped, PoseStamped, TransformStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Imu

class MURControlMixerNode():
    def __init__(self):
        # Init constants
        self.num_thrusters = 4
        self.angle = 10.0
        self.x_bar = 0.197
        self.y_bar = 0.18022
        self.b = 0.038

        # Desire parameters
        self.saturation = 20.0

        # Variables
        self.force_attitude = np.zeros(shape=(6,1))
        self.wrench_attitude = np.zeros(shape=(6,1))
        self.force_height = np.zeros(shape=(6,1))
        self.pose_rot = np.array([0,0,0,1])
        self.thrusters = np.zeros(shape=(4,1))
        self.thrusters_1 = np.zeros(shape=(4,1))
        self.J,_ = mur_common.convert_body_world(self.pose_rot)

        # Convert parameters
        self.T = self.get_t_matrix(0,self.force_attitude, self.wrench_attitude)

        # Smooth Signal
        self.mytime = 0.05
        self.n_points = 5
        self.ts = self.mytime/(self.n_points+1.0)
        self.limit = 0.5

        # ROS parameter server
        self.config = {}

        # ROS infraestucture
        self.srv_reconfigure = Server(MurControlMixerConfig, self.config_callback)
        self.pub_actuators = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
        self.sub_odometry = rospy.Subscriber('/mavros/imu/data', Imu, self.get_odometry)
        self.sub_force = rospy.Subscriber('/mur/force_input', WrenchStamped, self.cmd_force_callback)
        self.sub_wrench = rospy.Subscriber('/mur/wrench_input', WrenchStamped, self.cmd_wrench_callback)

    def config_callback(self, config, level):
        self.saturation = config['saturation']
        # To refresh the config value
        self.config = config
        # Return the config value
        return config

    def get_force_callback(self, msg):
        force = np.array([[msg.wrench.force.x], [msg.wrench.force.y], [msg.wrench.force.z], [msg.wrench.torque.x], [msg.wrench.torque.y], [msg.wrench.torque.z]])
        return force

    def get_t_matrix(self, mode, force_vector, wrench_vector):
        if mode!=0:
            t_matrix = np.matrix([[0, 0, 0, 0],
                [-np.sin(np.deg2rad(self.angle)), np.sin(np.deg2rad(self.angle)), np.sin(np.deg2rad(self.angle)), -np.sin(np.deg2rad(self.angle))],
                [np.cos(np.deg2rad(self.angle)), np.cos(np.deg2rad(self.angle)), np.cos(np.deg2rad(self.angle)), np.cos(np.deg2rad(self.angle))],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0]])
            ### TX ###
            if wrench_vector[3]>0:
                t_matrix[3]=np.array([0, -np.cos(np.deg2rad(self.angle))*self.y_bar, -np.cos(np.deg2rad(self.angle))*self.y_bar, 0])
            else:
                t_matrix[3]=np.array([np.cos(np.deg2rad(self.angle))*self.y_bar, 0, 0, np.cos(np.deg2rad(self.angle))*self.y_bar])
            ### TY ###
            if wrench_vector[4]>0:
                t_matrix[4]=np.array([-np.cos(np.deg2rad(self.angle))*self.x_bar, -np.cos(np.deg2rad(self.angle))*self.x_bar, 0, 0])
            else:
                t_matrix[4]=np.array([0, 0, np.cos(np.deg2rad(self.angle))*self.x_bar, np.cos(np.deg2rad(self.angle))*self.x_bar])
            ### TZ ###
            if wrench_vector[5]>0:
                t_matrix[5]=np.array([-np.sin(np.deg2rad(self.angle))*self.x_bar, 0, -np.sin(np.deg2rad(self.angle))*self.x_bar, 0])
            else:
                t_matrix[5]=np.array([0, np.sin(np.deg2rad(self.angle))*self.x_bar, 0, np.sin(np.deg2rad(self.angle))*self.x_bar])
            return t_matrix
        else:
            t_matrix = np.matrix([[0, 0, 0, 0],
                [-np.sin(np.deg2rad(self.angle)), np.sin(np.deg2rad(self.angle)), np.sin(np.deg2rad(self.angle)), -np.sin(np.deg2rad(self.angle))],
                [np.cos(np.deg2rad(self.angle)), np.cos(np.deg2rad(self.angle)), np.cos(np.deg2rad(self.angle)), np.cos(np.deg2rad(self.angle))],
                [np.cos(np.deg2rad(self.angle))*self.y_bar, -np.cos(np.deg2rad(self.angle))*self.y_bar, -np.cos(np.deg2rad(self.angle))*self.y_bar, np.cos(np.deg2rad(self.angle))*self.y_bar],
                [-np.cos(np.deg2rad(self.angle))*self.x_bar, -np.cos(np.deg2rad(self.angle))*self.x_bar, np.cos(np.deg2rad(self.angle))*self.x_bar, np.cos(np.deg2rad(self.angle))*self.x_bar],
                [-np.sin(np.deg2rad(self.angle))*self.x_bar, np.sin(np.deg2rad(self.angle))*self.x_bar, -np.sin(np.deg2rad(self.angle))*self.x_bar, np.sin(np.deg2rad(self.angle))*self.x_bar]])
            return t_matrix


    def set_force_thrusters(self, event):
        # Thruster forces
        Ft = np.matmul(self.J,self.force_attitude)
        Wt = np.matmul(self.J,self.wrench_attitude)
        self.T=self.get_t_matrix(0,Ft,Wt)
        B = np.linalg.pinv(self.T)
        #rospy.loginfo("B := %s" %B)
        #rospy.loginfo("Tt :=\n %s" %Tt)
        thrusters_forces = np.matmul(B,Ft)
        thrusters_wrenchs = np.matmul(B,Wt)
        #max_limit = np.amax(thrusters_wrenchs)
        #min_array = np.dot(np.ones(shape=(4,1)),max_limit)
        #thrusters_wrenchs = thrusters_wrenchs - min_array
        #rospy.loginfo("Thrusters :=\n %s" %self.thrusters)
        #rospy.loginfo("max_limit :=\n %s" %min_array)
        #rospy.loginfo("F:=\n %s", thrusters_forces)
        #rospy.loginfo("W:=\n %s", thrusters_wrenchs)
        self.thrusters = self.saturator_thruster(thrusters_forces+thrusters_wrenchs)
        self.pub_force()
        self.thrusters_1=self.thrusters

    def pub_force(self):
        rospy.loginfo("Thrusters :=\n %s" %self.thrusters)
        msg_actuators = OverrideRCIn()
        msg_actuators.channels = np.array([mur_common.push_to_pwm(self.thrusters[0]),mur_common.push_to_pwm(self.thrusters[1]),mur_common.push_to_pwm(self.thrusters[2]),mur_common.push_to_pwm(self.thrusters[3]),0,0,0,0])
        rospy.loginfo("Motors Values:= %s", msg_actuators.channels[0:self.num_thrusters])
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
        self.pose_rot = np.array([msg_pose.orientation.x, msg_pose.orientation.y, msg_pose.orientation.z, msg_pose.orientation.w])
        #rospy.loginfo("Pos := \n %s" %self.pose_pos)
        self.J, orientation = mur_common.convert_body_world(self.pose_rot)
        self.pitch=orientation[0]
        self.roll=orientation[1]
        self.yaw=orientation[2]
        #rospy.loginfo("Orientation:= %s", orientation)


    def cmd_force_callback(self, msg_force):
        self.force_attitude = np.array([[msg_force.wrench.force.x], [msg_force.wrench.force.y], [msg_force.wrench.force.z], [msg_force.wrench.torque.x], [msg_force.wrench.torque.y], [msg_force.wrench.torque.z]])

    def cmd_wrench_callback(self, msg_wrench):
        self.wrench_attitude = np.array([[msg_wrench.wrench.force.x], [msg_wrench.wrench.force.y], [msg_wrench.wrench.force.z], [msg_wrench.wrench.torque.x], [msg_wrench.wrench.torque.y], [msg_wrench.wrench.torque.z]])



if __name__ == '__main__':
    np.set_printoptions(suppress=True)
    rospy.init_node('mur_control_mixer')
    try:
        node = MURControlMixerNode()
        rospy.Timer(rospy.Duration(node.mytime), node.set_force_thrusters)
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
