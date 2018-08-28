#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
from common import mur_common
from dynamic_reconfigure.server import Server
from mur_control.cfg import MurVelocityControlConfig
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped,Vector3, Quaternion, Pose
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class MURVelocityControlNode:
    def __init__(self):
        # Init constants
        self.dt_vel = 2
        self.station_keeping = 2

        # State vectors
        self.nitad = np.zeros(shape=(6,1))
        self.nita = np.zeros(shape=(6,1))
        self.error_pos = np.zeros(shape=(6,1))
        self.error_vel = np.zeros(shape=(6,1))

        # Desire values
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.angle = 0

        # ROS param server
        self.config = {}

        # Control gains
        self.p_xy = np.eye(2)
        self.d_xy = 0.0
        self.p_pr = 0.0
        self.d_pr = 0.0

        # ROS infrastructure
        self.srv_reconfigure = Server(MurVelocityControlConfig, self.config_callback)
        self.sub_cmd_pose = rospy.Subscriber('/mur/pose_gt', Odometry, self.cmd_pose_callback)
        self.pub_cmd_force = rospy.Publisher('/control/Wrench/velocity', WrenchStamped, queue_size=1)

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
        self.nita = np.array([[self.pose_pos[0]],[self.pose_pos[1]],[self.pose_pos[2]], [self.nita2[0]],[self.nita2[1]],[self.nita2[2]]])
        # Global rotation
        self.J = mur_common.convert_body_world(self.pose_rot)
        # Convert to SNAME Velocity
        self.vitad = np.zeros(self.nitad.shape)
        self.vita = np.array([[self.twist_pos[0]],[self.twist_pos[1]],[self.twist_pos[2]], [self.twist_rot[0]],[self.twist_rot[1]],[self.twist_rot[2]]])
        self.nita_p = np.matmul(self.J,self.vita)
        # Get position error
        self.get_errors()
        self.force_callback()

    def get_errors(self):
        # Create the errors
        self.error_pos = self.nitad - self.nita

        vitad = np.empty_like(self.error_pos)
        for i in range(len(self.error_pos)):
            vitad[i]=self.error_pos[i]/self.dt_vel
        self.error_vel = self.vitad - self.vita

    def force_callback(self):
        # Control Law
        pos_error = self.error_pos[0:2]
        # PD control
        position_rate_error = np.matmul(np.transpose(mur_common.rot2(self.nita[5,0])), pos_error) - np.multiply(self.d_xy, self.nita_p[0:2,])
        attitude_error = np.matmul(self.p_xy, position_rate_error) - self.nita[3:5,]
        attitude_rate_error = attitude_error - np.multiply(self.d_pr, self.nita_p[3:5,])
        force = np.dot(self.p_pr,attitude_rate_error)
        # To create the message
        force_msg = WrenchStamped()
        force_msg.header.stamp = rospy.Time.now()
        force_msg.header.frame_id = 'mur/control'
        force_msg.wrench.torque.x = force[0]
        force_msg.wrench.torque.y = force[1]
        # To publish the message
        self.pub_cmd_force.publish(force_msg)


    def config_callback(self, config, level):
        # Config has changed, reset PD controllers
        par_xy = config['p_xy']
        self.p_pr = config['p_pr']
        self.d_xy = config['d_xy']
        self.d_pr = config['d_pr']

        # To put info into the matrix
        np.put(self.p_xy, [0,1,2,3], [0, par_xy, -par_xy, 0])
        # To refresh the desire points (To topics after, while like parameters)
        self.pos_x = config['pos_x']
        self.pos_y = config['pos_y']
        # To build the desire points vector
        self.nitad = np.array([[self.pos_x], [self.pos_y], [0.0], [0.0], [0.0], [0.0]])
        # To refresh the config value
        self.config = config
        # Return the config value
        return config


if __name__ == '__main__':
    rospy.init_node('mur_velocity_control')
    try:
        node = MURVelocityControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')

