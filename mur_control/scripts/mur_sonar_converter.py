#!/usr/bin/env python

import numpy as np
import math
import rospy
import logging
import sys
import tf
import tf2_ros
import message_filters
from common import mur_common
from mur_control.cfg import MurControlMixerConfig
from mur_control.msg import FloatStamped
from geometry_msgs.msg import WrenchStamped, PoseStamped, TransformStamped, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud, LaserScan
from tritech_micron.msg import TritechMicronConfig

class MURSonarConverterNode():
    def __init__(self):
        # Init constants
        self.x_1 = 0.0
        self.y_1 = 0.0
        self.angle_increment = 0.0
        self.range_max = 5.0
        self.range_min = 0.0
        self.heading = 0.0
        self.angle_increment = math.pi/100.0
        self.range = np.zeros(shape=(int(2.0*math.pi/self.angle_increment),1))
        self.min_distance = 0.3 # 30 cm
        self.max_distance = 4.5 # 30 cm
        self.umbral = 0.15 # 15 cm

        # ROS infraestucture
        self.pub_laser = rospy.Publisher('/sonar', LaserScan, queue_size=1)
        self.sub_cloud = rospy.Subscriber('/tritech_micron/scan', PointCloud, self.get_cloud)
        self.sub_config = rospy.Subscriber('/tritech_micron/config', TritechMicronConfig, self.get_config)
        self.sub_heading = rospy.Subscriber('/tritech_micron/heading', PoseStamped, self.get_heading)

    def get_config(self, msg_config):
        if msg_config.step != self.angle_increment:
            self.angle_increment = msg_config.step
            self.range_max = msg_config.range
            self.range = np.zeros(shape=(int(2.0*math.pi/self.angle_increment),1))

    def get_heading(self, msg_heading):
        pose_heading = np.array([msg_heading.pose.orientation.x,msg_heading.pose.orientation.y,msg_heading.pose.orientation.z,msg_heading.pose.orientation.w])
        _,_,self.heading = euler_from_quaternion(pose_heading)
        self.header = msg_heading.header

    def get_cloud(self, msg_cloud):
        values_array = np.array([msg_cloud.channels[0].values])
        max_val = np.amax(values_array);
        result = np.where(values_array == max_val)
        if len(result)>1:
            index = int(result[1][0])
        else:
            index=int(result[1])
        x_val =  msg_cloud.points[index].x
        y_val =  msg_cloud.points[index].y
        if abs(x_val-self.x_1)<self.umbral and abs(y_val-self.y_1)>self.umbral:
            x_val=self.x_1
        elif abs(y_val-self.y_1)<self.umbral and abs(x_val-self.x_1)>self.umbral:
            y_val=self.y_1
        index_range=int(self.heading/self.angle_increment)
        value_range=np.linalg.norm(np.array([x_val, y_val]))
        if value_range<self.min_distance or value_range>self.max_distance:
            value_range = self.range[index_range-1]
        if self.range[index_range-1]==0 and self.range[index_range+1]==0:
            self.range[index_range] = value_range
        elif self.range[index_range-1]==0:
            self.range[index_range] = (value_range+self.range[index_range+1])/2.0
        elif self.range[index_range+1]==0:
            self.range[index_range] = (self.range[index_range-1]+value_range)/2.0
        else:
            self.range[index_range] = (self.range[index_range-1]+value_range+self.range[index_range+1])/3.0
        #rospy.loginfo("X val:= %s Y val:= %s Angle:= %s," %(x_val,y_val,self.heading))
        msg_laser = LaserScan()
        msg_laser.header = self.header
        msg_laser.angle_min = 0.0
        msg_laser.angle_max = 2.0*math.pi
        msg_laser.angle_increment = self.angle_increment
        msg_laser.range_min = self.range_min
        msg_laser.range_max = self.range_max
        msg_laser.ranges = self.range
        self.x_1 = x_val
        self.y_1 = y_val
        self.pub_laser.publish(msg_laser)

if __name__ == '__main__':
    rospy.init_node('mur_sonar_converter')
    try:
        node = MURSonarConverterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
