#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
from common import mur_common, mur_PID
from dynamic_reconfigure.server import Server
from mur_control.cfg import MurAttitudeControlConfig
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped,Vector3, Quaternion, Pose
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class MURAttitudeControlNode:
    def __init__(self):
        # Init constants
        self.dt_vel = 2
        self.station_keeping = 2

        # State vectors
        self.nitad = np.zeros(shape=(6,1))
        self.error_pos = np.zeros(shape=(6,1))
        self.error_vel = np.zeros(shape=(6,1))

        # Desire values
        self.roll_d = 0.0
        self.pitch_d = 0.0
        self.yaw_d = 0.0
        self.nitad = np.array([0.0, 0.0, 0.0, self.pitch_d, self.roll_d, self.yaw_d])

        # ROS param server
        self.config = {}

        # Control gains
        self.p_r = 0.001
        self.i_r = 1
        self.d_r = 1

        self.p_p = 0.001
        self.i_p = 0
        self.d_p = 0

        self.p_y = 1
        self.i_y = 0
        self.d_y = 0

        # PID Control
        self.pid_pitch  = mur_PID.mur_PID(self.p_p, self.i_p, self.d_p, 6)
        self.pid_roll   = mur_PID.mur_PID(self.p_r, self.i_r, self.d_r, 6)
        self.pid_yaw    = mur_PID.mur_PID(self.p_y, self.i_y, self.d_y, 6)

        # ROS infrastructure
        self.srv_reconfigure = Server(MurAttitudeControlConfig, self.config_callback)
        self.sub_cmd_pose = rospy.Subscriber('/mur/pose_gt', Odometry, self.cmd_pose_callback)
        self.pub_cmd_force = rospy.Publisher('/control/wrench/torque', WrenchStamped, queue_size=10)

    def cmd_pose_callback(self, msg):
        if not bool(self.config):
            return
        # Get the position and velocities
        self.pose_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.pose_rot = np.array([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.twist_pos = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        self.twist_rot = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        # Convert to SNAME Position
        self.nita2_t = euler_from_quaternion(self.pose_rot)
        self.nita2 = np.array([self.nita2_t[0],self.nita2_t[1],self.nita2_t[2]])
        self.nita = np.array([self.pose_pos[0],self.pose_pos[1],self.pose_pos[2], self.nita2[0],self.nita2[1],self.nita2[2]]).reshape(self.nitad.shape)
        # Global rotation
        self.J = mur_common.convert_body_world(self.pose_rot)
        # Convert to SNAME Velocity
        self.vitad = np.zeros(self.nitad.shape)
        self.vita = np.array([self.twist_pos[0],self.twist_pos[1],self.twist_pos[2], self.twist_rot[0],self.twist_rot[1],self.twist_rot[2]]).reshape(self.vitad.shape)
        self.nita_p = np.matmul(self.J,self.vita)
        # Get position error
        self.t = msg.header.stamp.to_sec()
        self.get_errors()
        self.force_callback()

    def get_errors(self):
        # Create the errors
        self.error_pos = self.nitad - self.nita
        vitad = np.empty_like(self.error_pos)
        for i in range(len(self.error_pos)):
            vitad[i]=self.error_pos[i]/self.dt_vel
        self.error_vel = self.vitad - self.nita_p

    def force_callback(self):
        # Control Law
        # PID control
        T_p = self.pid_pitch.controlate(self.error_pos[3,],self.t)
        T_r = self.pid_roll.controlate(self.error_pos[4,],self.t)
        T_y = self.pid_yaw.controlate(self.error_pos[5,],self.t)
        # To create the message
        force_msg = WrenchStamped()
        force_msg.header.stamp = rospy.Time.now()
        force_msg.header.frame_id = 'mur/control'
        force_msg.wrench.torque.x = T_p
        force_msg.wrench.torque.y = T_r
        force_msg.wrench.torque.z = T_y
        # To publish the message
        rospy.loginfo("Rot :=\n %s" %self.nita2)
        self.pub_cmd_force.publish(force_msg)


    def config_callback(self, config, level):
        # To put info into the matrix
        self.p_r = config['p_r']
        self.i_r = config['i_r']
        self.d_r = config['d_r']

        self.p_p = config['p_p']
        self.i_p = config['i_p']
        self.d_p = config['d_p']

        self.p_y = config['p_yaw']
        self.i_y = config['i_yaw']
        self.d_y = config['d_yaw']

        # PID Control
        self.pid_roll   = mur_PID.mur_PID(self.p_r, self.i_r, self.d_r, 6)
        self.pid_pitch  = mur_PID.mur_PID(self.p_p, self.i_p, self.d_p, 6)
        self.pid_yaw    = mur_PID.mur_PID(self.p_y, self.i_y, self.d_y, 6)
        # To refresh the desire points (To topics after, while like parameters)
        self.roll_d = config['roll_d']
        self.pitch_d = config['pitch_d']
        self.yaw_d = config['yaw_d']
        # To build the desire points vector
        self.nitad = np.array([0.0, 0.0, 0.0, self.pitch_d, self.roll_d, self.yaw_d])
        # To refresh the config value
        self.config = config
        # Return the config value
        return config


if __name__ == '__main__':
    rospy.init_node('mur_Attitude_control')
    try:
        node = MURAttitudeControlNode()
        rospy.Rate(20)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
