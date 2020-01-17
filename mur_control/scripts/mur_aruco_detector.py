#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
import tf2_ros
import cv2
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from common import mur_common
from mur_control.msg import BoolStamped
from geometry_msgs.msg import WrenchStamped, PoseStamped, AccelStamped, TransformStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, compose_matrix, decompose_matrix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Imu, Image
from std_msgs.msg import Bool

class MURArucoDetector():
    def __init__(self):
        # Aruco markers
        self.aruco_dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.markerLength = 0.15 # in meters

        # Camera
        camera_matrix = rospy.get_param('/mur_aruco_detector/camera_matrix/data')
        self.camera_matrix = np.array([[camera_matrix[0], camera_matrix[1], camera_matrix[2]],
        [camera_matrix[3], camera_matrix[4], camera_matrix[5]],
        [0, 0, 1]], dtype = "double")
        dist_coeffs = rospy.get_param('/mur_aruco_detector/distortion_coefficients/data')
        self.dist_coeffs = np.array([dist_coeffs[0],dist_coeffs[1],dist_coeffs[2],dist_coeffs[3],dist_coeffs[4]])

        # ROS infraestucture
        self.image_pub = rospy.Publisher("/mur/image_detector",Image, queue_size=1)
        self.pose_pub = rospy.Publisher("/mur/aruco_pose", PoseWithCovarianceStamped, queue_size=1)
        self.aruco_pub = rospy.Publisher("/mur/aruco_flag", BoolStamped, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/mur/mur/camera1/camera_image",Image,self.callback)
        self.listener = tf.TransformListener()


    def get_camera_pose_robot(self):
        try:
            (trans,rot) = self.listener.lookupTransform('camera_link_optical','base_link', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            trans = np.array([-0.055, 0.160, 0])
            rot= np.array([0.7068252, 0, 0, 0.7073883])
        rot = euler_from_quaternion(rot)
        # rospy.loginfo("trans :=\n %s", trans)
        RC_B = compose_matrix(None, None, rot, trans, None)
        return RC_B

    def get_aruco_pose(self, id):
        data_url="/mur_aruco_detector/marker_" + str(int(id)) + "/data"
        try:
            aruco_pose = rospy.get_param(data_url)
        except KeyError as e:
            rospy.loginfo("Parameter := %s. Not Found" %e)
            aruco_pose = {0.0,0.0,-1.75,0.0,0.0,0.0}
        tvec = np.array([aruco_pose[0], aruco_pose[1], aruco_pose[2]])
        rvec = np.array([aruco_pose[3], aruco_pose[4], aruco_pose[5]])
        return  tvec, rvec

    def pose_callback(self,tvec,rvec,aruco_flag):
        msg_flag = BoolStamped()
        msg_flag.header.stamp = rospy.Time.now()
        msg_flag.header.frame_id = 'odom'
        msg_flag.data = aruco_flag
        msg_pose = PoseWithCovarianceStamped()
        msg_pose.header.stamp = rospy.Time.now()
        msg_pose.header.frame_id = 'odom'
        msg_pose.pose.pose.position.x = tvec[0]
        msg_pose.pose.pose.position.y = tvec[1]
        msg_pose.pose.pose.position.z = tvec[2]
        q = quaternion_from_euler(rvec[0], rvec[1], rvec[2])
        msg_pose.pose.pose.orientation.x = q[0]
        msg_pose.pose.pose.orientation.y = q[1]
        msg_pose.pose.pose.orientation.z = q[2]
        msg_pose.pose.pose.orientation.w = q[3]
        self.pose_pub.publish(msg_pose)
        self.aruco_pub.publish(msg_flag)


    def callback(self, msg_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg_image, "bgr8")
        except CvBridgeError as e:
            print(e)
        corners, ids, ripoints = cv2.aruco.detectMarkers(cv_image,self.aruco_dic)
        aruco_flag = False
        if ids is not None:
            aruco_flag = True
            rvecs = np.zeros(shape=(len(ids),3))
            tvecs = np.zeros(shape=(len(ids),3))
            for i in range(len(ids)):
                rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.markerLength, self.camera_matrix, self.dist_coeffs)
                t_vec = np.array([tvec[0][0][0],tvec[0][0][1],tvec[0][0][2]])
                #rospy.loginfo("t_vec :=\n %s", t_vec)
                r_vec = np.array([rvec[0][0][0],rvec[0][0][1],rvec[0][0][2]])
                #rospy.loginfo("r_vec :=\n %s", r_vec)
                cv2.aruco.drawDetectedMarkers(cv_image,corners, ids)
                cv_image = cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.2)
                br = tf2_ros.TransformBroadcaster()
                t = TransformStamped()
                t.header.frame_id = "camera_link_optical"
                t.header.stamp = rospy.Time.now()
                t.child_frame_id = "Marker_"+str(int(ids[i]))
                t.transform.translation.x = t_vec[0]
                t.transform.translation.y = t_vec[1]
                t.transform.translation.z = t_vec[2]
                q = quaternion_from_euler(r_vec[0], r_vec[1], r_vec[2])
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                br.sendTransform(t)
                rvecs[i,:] = rvec
                tvecs[i,:] = tvec
                ## Get global position about the origin
                (trans, rot) = self.get_aruco_pose(ids[i])
                RO_A = compose_matrix(None, None, rot, trans, None) # Aruco Pose
                #rospy.loginfo("RO_A :=\n %s", RO_A)
                RC_A = compose_matrix(None, None, r_vec, t_vec, None) # Pose from Aruco to Camera Optical link
                RA_C = np.linalg.inv(RC_A)
                #rospy.loginfo("RC_A :=\n %s", RC_A)
                RC_B = self.get_camera_pose_robot() # Camera pose from Robot base link
                #rospy.loginfo("RC_B :=\n %s", RC_B)
                RA_B = np.matmul(RA_C,RC_B)
                #rospy.loginfo("RA_B :=\n %s", RA_B)
                RO_B = np.matmul(RO_A,RA_B)
                #rospy.loginfo("RO_B :=\n %s", RO_B)
                (_,_,rot,trans,_) = decompose_matrix(RO_B)
            self.pose_callback(trans,rot,aruco_flag)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('mur_aruco_detector')
    try:
        node = MURArucoDetector()
        rate = rospy.Rate(50)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
