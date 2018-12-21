#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
import cv2
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from common import mur_common
from geometry_msgs.msg import WrenchStamped, PoseStamped, AccelStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Imu, Image

class MURArucoDetector():
    def __init__(self):
        # Aruco markers
        self.aruco_dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.markerLength = 0.2 # in meters

        # Camera
        self.calibrationFile = "calibrationFileName.xml"
        self.calibrationParams = cv2.FileStorage(self.calibrationFile, cv2.FILE_STORAGE_READ)
        self.camera_matrix = self.calibrationParams.getNode("cameraMatrix").mat()
        self.dist_coeffs = self.calibrationParams.getNode("distCoeffs").mat()

        # ROS infraestucture
        self.image_pub = rospy.Publisher("/mur/image_detector",Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/mur/mur/camera/camera_image",Image,self.callback)


    def cameraPoseFromHomography(self, H):
        H1 = H[:, 0]
        H2 = H[:, 1]
        H3 = np.cross(H1, H2)

        norm1 = np.linalg.norm(H1)
        norm2 = np.linalg.norm(H2)
        tnorm = (norm1 + norm2) / 2.0;

        T = H[:, 2] / tnorm
        return np.mat([H1, H2, H3, T])

    def callback(self, msg_image):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(msg_image, "bgr8")
        except CvBridgeError as e:
          print(e)
        camera_matrix = np.array([[1.2519588293098975E+03, 0, 6.6684948780852471E+02],
            [0, 1.2519588293098975E+03, 3.6298123112613683E+02],
            [0, 0, 1]], dtype = "double")
        corners, ids, ripoints = cv2.aruco.detectMarkers(cv_image,self.aruco_dic)
        rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[0], self.markerLength, camera_matrix, self.dist_coeffs)
        if ids is not None:
        #if len(ids) > 0:
            #rospy.loginfo("Size ids:= \n%s",len(ids))
            #rospy.loginfo("Size tvec:= \n%s",tvec)
            pts_dst = np.array([[corners[0][0][0][0], corners[0][0][0][1]], [corners[0][0][1][0], corners[0][0][1][1]], [corners[0][0][2][0], corners[0][0][2][1]], [corners[0][0][3][0], corners[0][0][3][1]]])
            pts_src = pts_dst
            h, status = cv2.findHomography(pts_src, pts_dst)
            cv2.aruco.drawDetectedMarkers(cv_image,corners, ids)
            cv_image = cv2.aruco.drawAxis(cv_image, camera_matrix, self.dist_coeffs, rvec, tvec, 1)
            cameraPose = self.cameraPoseFromHomography(h)
            rospy.loginfo("Pose:= \n%s",h)
        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
          print(e)


if __name__ == '__main__':
    rospy.init_node('mur_aruco_detector')
    try:
        node = MURArucoDetector()
        rate = rospy.Rate(20)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
