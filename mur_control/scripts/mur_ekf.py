#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
import message_filters
from mur_control.msg import FloatStamped
from common import mur_common
from geometry_msgs.msg import WrenchStamped, PoseStamped, AccelStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Imu

class MURExtendedKalmanFilter():
    def __init__(self):
        # Init constants
        self.GRAVITY_VALUE = -9.81
        self.DIM_STATE = 15
        self.cov_acc = 0.06
        self.cov_ang = 0.0027
        self.cov_mag = 0.05
        self.cov_bar = 0.01
        self.cov_img = 0.01

        # EKF infraestructure
        self.X = np.zeros(shape=(self.DIM_STATE,1)) # x y z phi theta yaw dx dy dz dphi dtheta dyaw accx accy accz
        self.P = np.zeros(shape=(self.DIM_STATE,self.DIM_STATE))
        self.Q = np.zeros(shape=(self.DIM_STATE,self.DIM_STATE))
        self.F = np.eye(self.DIM_STATE)
        self.set_QMatrix()

        # Variables
        self.pose = np.zeros(shape=(6,1))
        self.twist = np.zeros(shape=(6,1))
        self.acc = np.zeros(shape=(3,1))
        self.dT = 0.01 # Variable
        self.last_time = 0.0 #init

        # ROS infraestucture
        self.pub_pose = rospy.Publisher('/mur/ekf', Odometry, queue_size=1)
        self.pub_acce = rospy.Publisher('/mur/Acc', AccelStamped, queue_size=1)
        self.sub_imu = message_filters.Subscriber('/mavros/imu/data', Imu)
        self.sub_pres = message_filters.Subscriber('/mavros/imu/diff_pressure', FluidPressure)
        self.tsync = message_filters.TimeSynchronizer([self.sub_imu, self.sub_pres], 10)
        self.tsync.registerCallback(self.cmd_ekf_callback)

    def set_QMatrix(self):
        self.Q[0,0] = 0.05  # x
        self.Q[1,1] = 0.05  # y
        self.Q[2,2] = 0.06  # z
        self.Q[3,3] = 0.03  # phi
        self.Q[4,4] = 0.03  # theta
        self.Q[5,5] = 0.06  # gamma
        self.Q[6,6] = 0.025  # dx
        self.Q[7,7] = 0.025  # dy
        self.Q[8,8] = 0.04 # dz
        self.Q[9,9] =  0.01 # dphi
        self.Q[10,10] = 0.01 # dtheta
        self.Q[11,11] = 0.02  # dgamma
        self.Q[12,12] = 0.01  # ddx
        self.Q[13,13] = 0.01 # ddy
        self.Q[14,14] = 0.015  # ddz

    def get_FMatrix(self):
        self.F[0,6] = self.dT
        self.F[1,7] = self.dT
        self.F[2,8] = self.dT
        self.F[3,9]  = self.dT
        self.F[4,10] = self.dT
        self.F[5,11] = self.dT
        self.F[6,12] = self.dT
        self.F[7,13] = self.dT
        self.F[8,14] = self.dT
        self.F[0,12] = 0.5*((self.dT)**2)
        self.F[1,13] = 0.5*((self.dT)**2)
        self.F[2,14] = 0.5*((self.dT)**2)

    def ekf_estimation(self, XEst, PEst, z):
        ### Predict
        self.get_FMatrix()
        XPred = np.matmul(self.F, XEst)
        # F = dF/dX
        PPred = self.F * PEst * np.transpose(self.F) + self.Q
        ### Update or Correction
        # Full dimensions
        #--- Provisional ---
        num_meas,_ = np.shape(z)
        # We have just 8 sensors: IMU(3 acc lin, 3 vel ang, 1 angle) Barometer (1 pos)
        zPred = np.zeros(shape=(num_meas,1))
        v = np.zeros(shape=(num_meas,1))
        H = np.zeros(shape=(num_meas, self.DIM_STATE))
        R = np.zeros(shape=(num_meas, num_meas))
        # Fill the matrixes
        for num_meas_index in range(num_meas):
            # Acc
            if num_meas_index == 0:
                H[num_meas_index,12] = 1.0
                zPred[num_meas_index,0] = XPred[12,0]
                R[num_meas_index,num_meas_index] = self.cov_acc
            if num_meas_index == 1:
                H[num_meas_index,13] = 1.0
                zPred[num_meas_index,0] = XPred[13,0]
                R[num_meas_index,num_meas_index] = self.cov_acc
            if num_meas_index == 2:
                H[num_meas_index,14] = 1.0
                zPred[num_meas_index,0] = XPred[14,0]
                R[num_meas_index,num_meas_index] = self.cov_acc
            # Ang Vel
            if num_meas_index == 3:
                H[num_meas_index,9] = 1.0
                zPred[num_meas_index,0] = XPred[9,0]
                R[num_meas_index,num_meas_index] = self.cov_ang
            if num_meas_index == 4:
                H[num_meas_index,10] = 1.0
                zPred[num_meas_index,0] = XPred[10,0]
                R[num_meas_index,num_meas_index] = self.cov_ang
            if num_meas_index == 5:
                H[num_meas_index,11] = 1.0
                zPred[num_meas_index,0] = XPred[11,0]
                R[num_meas_index,num_meas_index] = self.cov_ang
            # Yaw
            if num_meas_index == 6:
                H[num_meas_index,5] = 1.0
                zPred[num_meas_index,0] = XPred[5,0]
                R[num_meas_index,num_meas_index] = self.cov_mag
            # Barometer
            if num_meas_index == 7:
                H[num_meas_index,2] = 1.0
                zPred[num_meas_index,0] = XPred[3,0] + 0.38
                R[num_meas_index,num_meas_index] = self.cov_bar
            # Camera
            if num_meas_index == 8:
                H[num_meas_index,0] = 1.0
                zPred[num_meas_index,0] = XPred[0,0]
                R[num_meas_index,num_meas_index] = self.cov_img
            if num_meas_index == 9:
                H[num_meas_index,1] = 1.0
                zPred[num_meas_index,0] = XPred[1,0]
                R[num_meas_index,num_meas_index] = self.cov_img
            if num_meas_index == 10:
                H[num_meas_index,2] = 1.0
                zPred[num_meas_index,0] = XPred[2,0]
                R[num_meas_index,num_meas_index] = self.cov_img
            if num_meas_index == 11:
                H[num_meas_index,5] = 1.0
                zPred[num_meas_index,0] = XPred[5,0]
                R[num_meas_index,num_meas_index] = self.cov_img

        # Innovation
        v = z - zPred
        # Innovation covariance
        S1 = np.matmul(H, PPred)
        S = np.matmul(S1, np.transpose(H)) + R
        # Filter gain
        K1 = np.matmul(PPred, np.transpose(H))
        K = np.matmul(K1, np.linalg.inv(S))
        # return
        self.X = XPred + np.matmul(K, v)
        self.P =np.matmul((np.eye(self.DIM_STATE) - np.matmul(K , H)), PPred)

    def get_gravity_vector(self,g,e1,e2,e3,n):
        RB_I = np.array([[1-2*((e2**2)+(e3**2)), 2*(e1*e2+e3*n), 2*(e1*e3+e2*n)],[2*(e1*e2-e3*n), 1-2*((e1**2)+(e3**2)),2*(e2*e3+e1*n)],[2*(e1*e3+e2*n),2*(e2*e3+e1*n), 1-2*((e1**2)+(e2**2))]])
        g_v = np.array([[0],[0],[g]])
        GB_I = np.matmul(RB_I,g_v)
        return GB_I


    def cmd_ekf_callback(self, msg_pose, msg_pres):
        # Time refresh
        now = rospy.Time.now()
        time_now = now.secs + now.nsecs/1E9
        self.dT = time_now - self.last_time
        # Set rotation
        e1 = msg_pose.orientation.x
        e2 = msg_pose.orientation.y
        e3 = msg_pose.orientation.z
        n  = msg_pose.orientation.w
        quaternion = (e1,e2,e3,n)
        GB_I = self.get_gravity_vector(self.GRAVITY_VALUE,e1,e2,e3,n)
        rospy.loginfo("GB_I :=\n %s",GB_I)
        # Measurement vector
        num_meas = 8;
        z = np.zeros(shape=(num_meas,1))
        z[0,0] = msg_pose.linear_acceleration.y - GB_I[0,0]
        z[1,0] = msg_pose.linear_acceleration.x - GB_I[1,0]
        z[2,0] = -msg_pose.linear_acceleration.z - GB_I[2,0]
        z[3,0] = msg_pose.angular_velocity.x
        z[4,0] = msg_pose.angular_velocity.y
        z[5,0] = msg_pose.angular_velocity.z
        _,_,z[6,0] = euler_from_quaternion(quaternion) # yaw magnetometer
        z[7,0] = mur_common.pressure_to_meters(msg_pres.fluid_pressure) # barometer
        rospy.loginfo("z :=\n %s",z)

        # EKF
        self.ekf_estimation(self.X, self.P, z)
        # Publish
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'mur/base_link'
        odom_msg.pose.pose.position.x = self.X[0,0]
        odom_msg.pose.pose.position.y = self.X[1,0]
        odom_msg.pose.pose.position.z = self.X[2,0]
        qx,qy,qz,qw = quaternion_from_euler(self.X[3,0],self.X[4,0],self.X[5,0])
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = self.X[6,0]
        odom_msg.twist.twist.linear.y = self.X[7,0]
        odom_msg.twist.twist.linear.z = self.X[8,0]
        odom_msg.twist.twist.angular.x = self.X[9,0]
        odom_msg.twist.twist.angular.y = self.X[10,0]
        odom_msg.twist.twist.angular.z = self.X[11,0]
        self.pub_pose.publish(odom_msg)
        acce_msg = AccelStamped()
        acce_msg.header.stamp = rospy.Time.now()
        acce_msg.header.frame_id = 'mur/ekf'
        acce_msg.accel.linear.x = z[0,0]
        acce_msg.accel.linear.y = z[1,0]
        acce_msg.accel.linear.z = z[2,0]
        self.pub_acce.publish(acce_msg)
        rospy.loginfo("X :=\n %s",self.X)
        # Next iteration
        self.last_time = time_now

if __name__ == '__main__':
    rospy.init_node('mur_extended_kalman_filter')
    try:
        node = MURExtendedKalmanFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
