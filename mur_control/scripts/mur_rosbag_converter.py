#!/usr/bin/env python

import numpy as np
import rospy
import tf
from geometry_msgs.msg import WrenchStamped, WrenchStamped
from mavros_msgs.msg import Mavlink

class MURRosbagConverter():
    def __init__(self):
        self.input_msg = WrenchStamped()
        self.input_msg.wrench.force.x = 0.0
        self.input_msg.wrench.force.y = 0.0
        self.input_msg.wrench.force.z = 0.0
        self.input_msg.wrench.torque.x = 0.0
        self.input_msg.wrench.torque.y = 0.0
        self.input_msg.wrench.torque.z = 0.0
        # ROS infraestucture
        self.final_pub = rospy.Publisher("/mur/input",WrenchStamped, queue_size=1)
        self.force_sub = rospy.Subscriber("/mur/force_input",WrenchStamped,self.force_callback)
        self.wrench_sub = rospy.Subscriber("/mur/wrench_input",WrenchStamped,self.wrench_callback)
        self.wrench_sub = rospy.Subscriber("/mavlink/from",Mavlink,self.mavlink_callback)

    def force_callback(self,msg_force):
        self.input_msg.wrench.force.x = msg_force.wrench.force.x
        self.input_msg.wrench.force.y = msg_force.wrench.force.y
        self.input_msg.wrench.force.z = msg_force.wrench.force.z

    def wrench_callback(self,msg_force):
        self.input_msg.wrench.torque.x = msg_force.wrench.torque.x
        self.input_msg.wrench.torque.y = msg_force.wrench.torque.y
        self.input_msg.wrench.torque.z = msg_force.wrench.torque.z

    def mavlink_callback(self,msg_mavlink):
        self.input_msg.header.stamp = msg_mavlink.header.stamp
        self.input_msg.header.frame_id = "world"
        self.final_pub.publish(self.input_msg)
        rospy.loginfo("Republishing message.")
        
if __name__ == '__main__':
    rospy.init_node('mur_rosbag_converter')
    try:
        node = MURRosbagConverter()
        rate = rospy.Rate(5)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')