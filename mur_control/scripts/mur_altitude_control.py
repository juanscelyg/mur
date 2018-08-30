#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
from common import mur_common, mur_PID
from dynamic_reconfigure.server import Server
from mur_control.cfg import MurAltitudeControlConfig
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped,Vector3, Quaternion, Pose
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class MURAltitudeControlNode:
    def __init__(self):
        # Init constants
        self.dt_vel = 5
        self.station_keeping = 2
        self.mur_weight = 8.5 * 9.81

        # State vectors
        self.nitad = np.zeros(shape=(6,1))
        self.error_pos = np.zeros(shape=(6,1))
        self.error_vel = np.zeros(shape=(6,1))

        # Desire values
        self.pos_z = 0

        # ROS param server
        self.config = {}

        # Control gains
        self.p_z = 58.0
        self.i_z = 25.0
        self.d_z = 32.0

        # PID Control
        self.pid_z = mur_PID.mur_PID(self.p_z, self.i_z, self.d_z, 120)

        # ROS infrastructure
        self.srv_reconfigure = Server(MurAltitudeControlConfig, self.config_callback)
        self.sub_cmd_pose = rospy.Subscriber('/mur/pose_gt', Odometry, self.cmd_pose_callback)
        self.pub_cmd_force = rospy.Publisher('/control/wrench/force', WrenchStamped, queue_size=10)

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
        Tz = self.pid_z.controlate(self.error_pos[2,],self.t)
        # To create the message
        force_msg = WrenchStamped()
        force_msg.header.stamp = rospy.Time.now()
        force_msg.header.frame_id = 'mur/control'
        force_msg.wrench.force.z = Tz
        # To publish the message
        self.pub_cmd_force.publish(force_msg)


    def config_callback(self, config, level):
        # To put info into the matrix
        self.p_z = config['p_z']
        self.i_z = config['i_z']
        self.d_z = config['d_z']
        self.pid_z = mur_PID.mur_PID(self.p_z, self.i_z, self.d_z, 120)
        # To refresh the desire points (To topics after, while like parameters)
        self.pos_z = config['pos_z']
        # To build the desire points vector
        self.nitad = np.array([0.0, 0.0, self.pos_z, 0.0, 0.0, 0.0])
        # To refresh the config value
        self.config = config
        # Return the config value
        return config


if __name__ == '__main__':
    rospy.init_node('mur_altitude_control')
    try:
        node = MURAltitudeControlNode()
        rospy.Rate(5)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
