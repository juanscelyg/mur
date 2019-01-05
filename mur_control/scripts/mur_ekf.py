#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
import message_filters
from mur_control.msg import FloatStamped, BoolStamped
from common import mur_common
from geometry_msgs.msg import WrenchStamped, PoseStamped, AccelStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Imu
from std_msgs.msg import Bool

class MURExtendedKalmanFilter():
    def __init__(self):
        # Init constants
        self.GRAVITY_VALUE = -9.81
        self.DIM_STATE = 15
        self.cov_pos = 0.05
        self.cov_vel = 0.025
        self.cov_acc = 0.0006
        self.cov_ori = 0.006108
        self.cov_ang = 0.0027
        self.cov_mag = 0.06108
        self.cov_bar = 0.002
        self.cov_img = 0.01

        # EKF infraestructure
        self.X = np.zeros(shape=(self.DIM_STATE,1)) # x y z phi theta yaw dx dy dz dphi dtheta dyaw accx accy accz
        self.P = np.zeros(shape=(self.DIM_STATE,self.DIM_STATE))
        self.Q = np.zeros(shape=(self.DIM_STATE,self.DIM_STATE))
        self.F = np.eye(self.DIM_STATE)
        self.set_QMatrix()
        self.set_PMatrix()
        self.pres_flag = False
        self.pose_flag = False
        self.flag_flag = False

        # Variables
        self.pose = np.zeros(shape=(6,1))
        self.twist = np.zeros(shape=(6,1))
        self.acc = np.zeros(shape=(3,1))
        self.dT = 0.1 # Variable
        self.last_time = 0.0 #init

        # ROS infraestucture
        self.pub_pose = rospy.Publisher('/mur/ekf', Odometry, queue_size=1)
        self.pub_acce = rospy.Publisher('/mur/Acc', AccelStamped, queue_size=1)
        self.sub_imu = rospy.Subscriber('/mavros/imu/data', Imu, self.call_imu)
        self.sub_pres = rospy.Subscriber('/mavros/imu/diff_pressure', FluidPressure, self.call_pres)
        self.sub_pose = rospy.Subscriber('/mur/aruco_pose', PoseStamped, self.call_pose)
        self.sub_flag = rospy.Subscriber('/mur/aruco_flag', BoolStamped, self.call_flag)

    def call_imu(self, msg_imu):
        self.msg_imu = msg_imu
        if self.pres_flag and self.pose_flag and self.flag_flag:
            self.cmd_ekf_callback(self.msg_imu, self.msg_pres, self.msg_pose, self.msg_flag)
        else:
            rospy.loginfo("Unsycronized")

    def call_pres(self, msg_pres):
        self.pres_flag = True
        self.msg_pres = msg_pres

    def call_pose(self, msg_pose):
        self.pose_flag = True
        self.msg_pose = msg_pose

    def call_flag(self, msg_flag):
        self.flag_flag = True
        self.msg_flag = msg_flag

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

    def set_PMatrix(self):
        self.P[0,0] = 0.05  # x
        self.P[1,1] = 0.05  # y
        self.P[2,2] = 0.06  # z
        self.P[3,3] = 0.03  # phi
        self.P[4,4] = 0.03  # theta
        self.P[5,5] = 0.06  # gamma
        self.P[6,6] = 0.025  # dx
        self.P[7,7] = 0.025  # dy
        self.P[8,8] = 0.04 # dz
        self.P[9,9] =  0.01 # dphi
        self.P[10,10] = 0.01 # dtheta
        self.P[11,11] = 0.02  # dgamma
        self.P[12,12] = 0.01  # ddx
        self.P[13,13] = 0.01 # ddy
        self.P[14,14] = 0.015  # ddz

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
        # Full dimensions
        num_meas,_ = np.shape(z)
        ### Predict
        if num_meas is not 14:
            self.get_FMatrix()
            XPred = np.matmul(self.F, XEst)
            # F = dF/dX
            PPred = self.F * PEst * np.transpose(self.F) + self.Q
        else:
            XPred = XEst
            PPred = PEst
            rospy.loginfo("Not Prediction")
        ### Update or Correction
        # We have just 8 sensors: IMU(3 acc lin, 3 vel ang, 1 angle) Barometer (1 pos)
        # Using aruco we have 4 measurements more
        zPred = np.zeros(shape=(num_meas,1))
        v = np.zeros(shape=(num_meas,1))
        H = np.zeros(shape=(num_meas, self.DIM_STATE))
        R = np.zeros(shape=(num_meas, num_meas))
        # Fill the matrixes
        for num_meas_index in range(num_meas):
            # Angles
            if num_meas_index == 0:
                H[num_meas_index,3] = 1.0
                zPred[num_meas_index,0] = XPred[3,0]
                R[num_meas_index,num_meas_index] = self.cov_ori
            if num_meas_index == 1:
                H[num_meas_index,4] = 1.0
                zPred[num_meas_index,0] = XPred[4,0]
                R[num_meas_index,num_meas_index] = self.cov_ori
            if num_meas_index == 2:
                H[num_meas_index,5] = 1.0
                zPred[num_meas_index,0] = XPred[5,0]
                R[num_meas_index,num_meas_index] = self.cov_ori
            # Acc
            if num_meas_index == 3:
                H[num_meas_index,12] = 1.0
                zPred[num_meas_index,0] = XPred[12,0]
                R[num_meas_index,num_meas_index] = self.cov_acc
            if num_meas_index == 4:
                H[num_meas_index,13] = 1.0
                zPred[num_meas_index,0] = XPred[13,0]
                R[num_meas_index,num_meas_index] = self.cov_acc
            if num_meas_index == 5:
                H[num_meas_index,14] = 1.0
                zPred[num_meas_index,0] = XPred[14,0]
                R[num_meas_index,num_meas_index] = self.cov_acc
            # Ang Vel
            if num_meas_index == 6:
                H[num_meas_index,9] = 1.0
                zPred[num_meas_index,0] = XPred[9,0]
                R[num_meas_index,num_meas_index] = self.cov_ang
            if num_meas_index == 7:
                H[num_meas_index,10] = 1.0
                zPred[num_meas_index,0] = XPred[10,0]
                R[num_meas_index,num_meas_index] = self.cov_ang
            if num_meas_index == 8:
                H[num_meas_index,11] = 1.0
                zPred[num_meas_index,0] = XPred[11,0]
                R[num_meas_index,num_meas_index] = self.cov_ang
            # Barometer
            if num_meas_index == 9:
                H[num_meas_index,2] = 1.0
                zPred[num_meas_index,0] = XPred[2,0]
                R[num_meas_index,num_meas_index] = self.cov_bar
            # Camera
            if num_meas_index == 10:
                H[num_meas_index,0] = 1.0
                zPred[num_meas_index,0] = XPred[0,0]
                R[num_meas_index,num_meas_index] = self.cov_img
            if num_meas_index == 11:
                H[num_meas_index,1] = 1.0
                zPred[num_meas_index,0] = XPred[1,0]
                R[num_meas_index,num_meas_index] = self.cov_img
            if num_meas_index == 12:
                H[num_meas_index,2] = 1.0
                zPred[num_meas_index,0] = XPred[2,0]
                R[num_meas_index,num_meas_index] = self.cov_img
            if num_meas_index == 13:
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

    def cmd_ekf_callback(self, msg_imu, msg_pres,msg_pose,msg_flag):
        # Time refresh
        now = rospy.Time.now()
        time_now = now.secs + now.nsecs/1E9
        self.dT = time_now - self.last_time
        # Set rotation
        e1 = msg_imu.orientation.x
        e2 = msg_imu.orientation.y
        e3 = msg_imu.orientation.z
        n  = msg_imu.orientation.w
        quaternion = (e1,e2,e3,n)
        GB_I = self.get_gravity_vector(self.GRAVITY_VALUE,e1,e2,e3,n)
        # Measurement vector
        # z = [phi,theta,yaw,accx,accy,accz,ang_x,ang_y,ang_z,Z, x,y,z,ang_z2]
        mflag = msg_flag.data
        #mflag = True
        if mflag == True:
            num_meas = 14
            z = np.zeros(shape=(num_meas,1))
            z[10,0] = msg_pose.pose.position.x
            z[11,0] = msg_pose.pose.position.y
            z[12,0] = msg_pose.pose.position.z
            e1 = msg_pose.pose.orientation.x
            e2 = msg_pose.pose.orientation.y
            e3 = msg_pose.pose.orientation.z
            n  = msg_pose.pose.orientation.w
            q_aruco = (e1,e2,e3,n)
            _,_,z[13,0] = euler_from_quaternion(q_aruco)
        else:
            num_meas = 10
            z = np.zeros(shape=(num_meas,1))
        z[0,0],z[1,0],z[2,0] = euler_from_quaternion(quaternion) # yaw magnetometer
        z[3,0] = msg_imu.linear_acceleration.y - GB_I[0,0]
        z[4,0] = -msg_imu.linear_acceleration.x - GB_I[1,0]
        z[5,0] = -msg_imu.linear_acceleration.z - GB_I[2,0]
        z[6,0] = msg_imu.angular_velocity.y
        z[7,0] = msg_imu.angular_velocity.x
        z[8,0] = -msg_imu.angular_velocity.z
        z[9,0] = mur_common.pressure_to_meters(msg_pres.fluid_pressure) # barometer
        #rospy.loginfo("z :=\n %s",z)
        # EKF
        self.ekf_estimation(self.X, self.P, z)
        # Publish
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'world'
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
        acce_msg.accel.linear.x = z[3,0]
        acce_msg.accel.linear.y = z[4,0]
        acce_msg.accel.linear.z = z[5,0]
        self.pub_acce.publish(acce_msg)
        #rospy.loginfo("X :=\n %s",self.X)
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
