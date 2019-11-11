#!/usr/bin/python

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
        self.dt_vel = 2.0
        self.station_keeping = 2.0
        self.mur_mass = 9.6;
        self.mur_weight = self.mur_mass * 9.79

        # State vectors
        self.h_last = 0.0;
        self.nitad = np.zeros(shape=(6,1))
        self.error_pos = np.zeros(shape=(6,1))
        self.error_vel = np.zeros(shape=(6,1))

        # Desire values
        self.pos_z = -1.5
        self.roll_d = 0.0
        self.pitch_d = 0.0
        self.yaw_d = 0.0
        self.g_z = 23.0
        self.nitad = np.array([0.0, 0.0, self.pos_z, self.pitch_d, self.roll_d, self.yaw_d])

        # ROS param server
        self.config = {}

        # Control gains
        self.p_x = 0.03
        self.d_x = -0.05

        self.p_y = -0.06
        self.d_y = 0.06

        self.p_z = 12.0
        self.i_z = 0.05
        self.d_z = -40.0

        self.p_r = 10.0
        self.i_r = 0.25
        self.d_r = -12.0

        self.p_p = 10.0
        self.i_p = 0.2
        self.d_p = -12.0

        self.p_yaw = 10.0
        self.i_yaw = 0.2
        self.d_yaw = -12.0

        # PID Control
        self.pid_x      = mur_PID.mur_PID(self.p_x,0,self.d_x,0.5)
        self.pid_y      = mur_PID.mur_PID(self.p_y,0,self.d_y,0.5)
        self.pid_z      = mur_PID.mur_PID(self.p_z, self.i_z, self.d_z, 120)
        self.pid_pitch  = mur_PID.mur_PID(self.p_p, self.i_p, self.d_p, 6)
        self.pid_roll   = mur_PID.mur_PID(self.p_r, self.i_r, self.d_r, 6)
        self.pid_yaw    = mur_PID.mur_PID(self.p_yaw, self.i_yaw, self.d_yaw, 6)

        # ROS infrastructure
        self.srv_reconfigure = Server(MurAttitudeControlConfig, self.config_callback)
        self.sub_cmd_pose = rospy.Subscriber('/mur/pose_gt', Odometry, self.cmd_pose_callback)
        self.sub_cmd_desired = rospy.Subscriber('/mur/pose/desired', Pose, self.cmd_desired_callback)
        self.pub_cmd_force = rospy.Publisher('/mur/force_input', WrenchStamped, queue_size=2)
        #self.pub_pose_verification = rospy.Publisher('/mur/pose_verification', Odometry, queue_size=2)

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

    def cmd_desired_callback(self,msg):
        X_error = self.pose_pos[0,]-msg.position.x
        Y_error = self.pose_pos[1,]-msg.position.y
        self.R = mur_common.rot2(self.nita2[2,])
        [X_r, Y_r] = np.matmul(np.transpose(self.R),np.array([[X_error],[Y_error]]))
        self.roll_d = self.pid_x.controlate(self.error_pos[0,],-self.error_vel[0,],self.t)
        self.pitch_d = self.pid_y.controlate(self.error_pos[1,],-self.error_vel[1,],self.t)
        self.pos_z = msg.position.z
        # To build the desire points vector
        self.nitad = np.array([msg.position.x, msg.position.y, msg.position.z, self.pitch_d, self.roll_d, self.yaw_d])
        rospy.loginfo("nita desired :=\n %s" %self.nitad)


    def get_errors(self):
        # Create the errors
        self.error_pos = self.nitad - self.nita
        vitad = np.empty_like(self.error_pos)
        for i in range(len(self.error_pos)):
            vitad[i]=self.error_pos[i]/self.dt_vel
        self.error_vel = vitad - self.nita_p

    def force_callback(self):
        # Control Law
        # PID control
        Tz = self.pid_z.controlate(self.error_pos[2,],-self.error_vel[2,],self.t) + self.g_z
        T_p = self.pid_pitch.controlate(self.error_pos[3,],-self.error_vel[3,],self.t)
        T_r = self.pid_roll.controlate(self.error_pos[4,],-self.error_vel[4,],self.t)
        T_y = 0.0 #self.pid_yaw.controlate(self.error_pos[5,],-self.vita[5,],self.t)
        # To create the message
        force_msg = WrenchStamped()
        force_msg.header.stamp = rospy.Time.now()
        force_msg.header.frame_id = 'mur/base_link'
        force_msg.wrench.force.z = Tz
        force_msg.wrench.torque.x = T_p
        force_msg.wrench.torque.y = T_r
        force_msg.wrench.torque.z = T_y
        # To publish the message
        #rospy.loginfo("Rot :=\n %s" %self.error_pos)
        #rospy.loginfo("For :=\n %s" %np.array([Tz,T_p,T_r,T_y]))
        self.pub_cmd_force.publish(force_msg)
        '''
        # Odometry auxiliar
        pose_msg = Odometry()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'mur_base_link'
        pose_msg.pose.pose.position.x = self.error_pos[3,]
        pose_msg.pose.pose.position.y = self.error_pos[4,]
        pose_msg.pose.pose.position.z = self.error_pos[5,]
        self.pub_pose_verification.publish(pose_msg)
        '''


    def config_callback(self, config, level):
        # To put info into the matrix
        self.p_z = config['p_z']
        self.i_z = config['i_z']
        self.d_z = config['d_z']
        self.g_z = config['g_z']

        self.p_r = config['p_r']
        self.i_r = config['i_r']
        self.d_r = config['d_r']

        self.p_p = config['p_p']
        self.i_p = config['i_p']
        self.d_p = config['d_p']

        self.p_yaw = config['p_yaw']
        self.i_yaw = config['i_yaw']
        self.d_yaw = config['d_yaw']

        # PID Control
        self.pid_z      = mur_PID.mur_PID(self.p_z, self.i_z, self.d_z, 120)
        self.pid_roll   = mur_PID.mur_PID(self.p_r, self.i_r, self.d_r, 6)
        self.pid_pitch  = mur_PID.mur_PID(self.p_p, self.i_p, self.d_p, 6)
        self.pid_yaw    = mur_PID.mur_PID(self.p_yaw, self.i_yaw, self.d_yaw, 6)

        # To refresh the config value
        self.config = config
        # Return the config value
        return config


if __name__ == '__main__':
    rospy.init_node('mur_Attitude_control')
    try:
        node = MURAttitudeControlNode()
        rospy.Rate(50)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
