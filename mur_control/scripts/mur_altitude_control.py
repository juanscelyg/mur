#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
import message_filters
from common import mur_common, mur_PID
from dynamic_reconfigure.server import Server
from mur_control.cfg import MurAltitudeControlConfig
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped,Vector3, Quaternion, Pose
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import FluidPressure, Imu

class MURAltitudeControlNode:
    def __init__(self):
        # Init constants
        self.dt_vel = 5
        self.station_keeping = 2
        self.mur_weight = 8.5 * 9.81

        # State vectors
        self.h_last = 0;
        self.nitad = np.zeros(shape=(6,1))
        self.error_pos = np.zeros(shape=(6,1))
        self.error_vel = np.zeros(shape=(6,1))

        # Desire values
        self.pos_z = -1

        # ROS param server
        self.config = {}

        # Control gains
        self.p_z = 38.0
        self.i_z = 5.0
        self.d_z = 12.0

        # PID Control
        self.pid_z = mur_PID.mur_PID(self.p_z, self.i_z, self.d_z, 120)

        # ROS infrastructure
        self.srv_reconfigure = Server(MurAltitudeControlConfig, self.config_callback)
        self.sub_pose = message_filters.Subscriber('/mavros/local_position/pose', PoseStamped)
        self.sub_imu = message_filters.Subscriber('/mavros/imu/data', Imu)
        self.sub_pres = message_filters.Subscriber('/mavros/imu/diff_pressure', FluidPressure)
        self.ts = message_filters.TimeSynchronizer([self.sub_pose, self.sub_pres, self.sub_imu], 10)
        self.ts.registerCallback(self.cmd_control_callback)
        self.pub_cmd_force = rospy.Publisher('/control/force', WrenchStamped, queue_size=10)

    def cmd_control_callback(self, msg_pose, msg_pres, msg_imu):
        if not bool(self.config):
            return
        self.t = msg_pose.header.stamp.to_sec()
        # Get the position and velocities
        h = mur_common.pressure_to_meters(msg_pres.fluid_pressure);
        rospy.loginfo("H :=\n %s" %h)
        self.pose_pos = np.array([0,0,h])
        self.pose_rot = np.array([msg_pose.pose.orientation.x, msg_pose.pose.orientation.y, msg_pose.pose.orientation.z, msg_pose.pose.orientation.w])
        self.twist_pos = np.array([0,0,(h-self.h_last)/self.t])
        self.twist_rot = np.array([msg_imu.angular_velocity.x, msg_imu.angular_velocity.y, msg_imu.angular_velocity.z])
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
        self.h_last = h;
        self.get_errors()
        self.force_callback()

    def get_errors(self):
        # Create the errors
        self.error_pos = self.nitad - self.nita
        ### rospy.loginfo("Error :=\n %s" %self.error_pos)
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
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
