#!/usr/bin/python

import numpy as np
import math
import rospy
import logging
import sys
import tf
from common import mur_common, mur_PID
from threading import Timer
from dynamic_reconfigure.server import Server
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped,Vector3, Quaternion, Pose, PoseWithCovarianceStamped
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, StatusText
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import FluidPressure, Imu

class MURPIDAcc():
    def __init__(self):
        # Globals
        self.gravity = 9.81
        self.mytime = 0.4
        self.t_1 = 0.0

        # Angles
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.pitch_d = 0.0
        self.roll_d = 0.0
        self.yaw_d = 0.0

        # Angular velocities
        self.vita2 = np.zeros(shape=(3,1))
        self.vita2_1 = np.zeros(shape=(3,1))
        self.vitap2 = np.zeros(shape=(3,1))

        # Depth
        self.z = 0.0
        self.z_1 = 0.0
        self.z_d = 0.0
        self.vz = 0.0
        self.vpz = 0.0

        # Pose Desired
        self.pitch_e = 0.0
        self.roll_e = 0.0
        self.yaw_e = 0.0
        self.z_e = 0.0

        # ROS infrastructure
        self.sub_imu = rospy.Subscriber('/mavros/imu/data', Imu, self.get_imu)
        self.sub_depth = rospy.Subscriber('/mur/depth/pose', PoseWithCovarianceStamped, self.get_depth)
        self.sub_pose = rospy.Subscriber('/mur/cmd_pose', PoseStamped, self.get_pose)

        self.pub_force = rospy.Publisher('/mur/force_input', WrenchStamped, queue_size=1)
        self.pub_wrench = rospy.Publisher('/mur/wrench_input', WrenchStamped, queue_size=1)

    def get_imu(self, msg_imu):
        # Get Z acc
        self.vpz = msg_imu.linear_acceleration.z - self.gravity # Assumpting that the robot is stabilzed about RP = 0
        # Get the angles and angular velocities
        pose_rot = np.array([msg_imu.orientation.x, msg_imu.orientation.y, msg_imu.orientation.z, msg_imu.orientation.w])
        _, orientation = mur_common.convert_body_world(pose_rot)
        self.pitch=orientation[0]
        self.roll=orientation[1]
        self.yaw=orientation[2]
        self.vita2 = np.array([[msg_imu.angular_velocity.x], [msg_imu.angular_velocity.y], [msg_imu.angular_velocity.z]])


    def get_depth(self, msg_depth):
        self.z = msg_depth.pose.pose.position.z


    def get_pose(self, msg_pose):
        pose_rot = np.array([msg_pose.pose.orientation.x, msg_pose.pose.orientation.y, msg_pose.pose.orientation.z, msg_pose.pose.orientation.w])
        _, orientation = mur_common.convert_body_world(pose_rot)
        self.pitch_d = orientation[0]
        self.roll_d = orientation[1]
        self.yaw_d = orientation[2]
        self.pitch_e = self.pitch - self.pitch_d
        self.roll_e = self.roll - self.roll_d
        self.yaw_e = self.yaw - self.yaw_d
        if abs(self.pitch_e)>0.2:
            self.pitch_e = 0.2*self.pitch_e/abs(self.pitch_e)
        if abs(self.roll_e)>0.2:
            self.roll_e = 0.2*self.roll_e/abs(self.roll_e)
        if abs(self.yaw_e)>0.2:
            self.yaw_e = 0.2*self.yaw_e/abs(self.yaw_e)
        self.z_d = msg_pose.pose.position.z
        self.z_e = self.z - self.z_d
        if abs(self.z_e)>0.2:
            self.z_e = 0.2*self.z_e/abs(self.z_e)

    def force_pub(self, event):
        t_now = rospy.get_time()
        dt = t_now - self.t_1
        # Depth Control
        self.vz = (self.z - self.z_1)/dt
        #rospy.loginfo("dt:= %s", dt)
        #rospy.loginfo("Vz:= %s", self.vz)
        #rospy.loginfo("VPz:= %s", self.vpz)
        Fz = self.altitude_control(self.z_e, self.z_d, self.vz, self.vpz)
        force_msg = WrenchStamped()
        force_msg.header.stamp = rospy.Time.now()
        force_msg.header.frame_id = 'mur/base_link'
        force_msg.wrench.force.z = Fz
        self.pub_force.publish(force_msg)
        # Stabilization Control
        self.vitap2 = (self.vita2 - self.vita2_1)/dt
        #rospy.loginfo("v2:= %s", self.vitap2)
        Tx = self.attitude_control(self.pitch_e, self.pitch_d, self.vita2[0], self.vitap2[0], 1.9, 0.1492, 0.013, 1.0130, 3)
        Ty = self.attitude_control(self.roll_e, self.roll_d, self.vita2[1], self.vitap2[1], 0.9, 0.2222, 0.013, 1.6901, 3)
        Tz = self.attitude_control(self.yaw_e, self.yaw_d, self.vita2[2], self.vitap2[2], 0.08, 0.3209, 1.0, 0.0, 0.15)
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = rospy.Time.now()
        wrench_msg.header.frame_id = 'mur/base_link'
        wrench_msg.wrench.torque.x = Tx
        wrench_msg.wrench.torque.y = Ty+0.1
        wrench_msg.wrench.torque.z = 0.0
        self.pub_wrench.publish(wrench_msg)
        rospy.loginfo("T:= %s", np.array([[Fz],[Tx],[Ty],[Tz]]))
        # Loop variables
        self.t_1 = t_now
        self.z_1 = self.z
        self.vita2_1 = self.vita2

    def altitude_control(self, error, ref, v1, vp1):
        ## Variables
        K_qz=55.7484;
        m=10.7;
        ## Z (Z)
        wb=0.9;
        zita=1.0;
        km=m;
        wn = wb/(math.sqrt(1-2*zita**2+math.sqrt(4*zita**4-4*zita**2+2)))
        F_ref=ref;
        Kp=(m+km)*wn**2;
        Kd=2*zita*wn*(m+km)-K_qz;
        F_pid=Kp*error+Kd*v1;
        F_acc=km*vp1;
        F=F_ref-F_pid-F_acc;
        if F>0:
            F=0.0;
        if F<-80:
            F=-80.0
        return F

    def attitude_control(self, error, ref, v, vp, wb, km, r_gb, Kq, sat):
        zita=1.0
        wn = wb/(math.sqrt(1-2*zita**2+math.sqrt(4*zita**4-4*zita**2+2)))
        T_ref=ref*r_gb;
        Kp=(2*km)*wn**2-r_gb;
        Kd=2*zita*wn*(2*km)-Kq;
        T_pid=Kp*error+Kd*v;
        T_acc=km*vp;
        T=T_ref-T_pid-T_acc;
        #rospy.loginfo("T:= %s", T)
        if abs(T)>sat:
            T = sat*T/abs(T)
        return T


if __name__ == '__main__':
    np.set_printoptions(suppress=True)
    rospy.init_node('mur_pid_acc')
    try:
        node = MURPIDAcc()
        rospy.Timer(rospy.Duration(node.mytime), node.force_pub)
        rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
