#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
from simtools import mur_simtools
from dynamic_reconfigure.server import Server
from mur_control.cfg import MurControlMixerConfig
from rospy.numpy_msg import numpy_msg
from mur_control.msg import FloatStamped
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped,Vector3, Quaternion, Pose
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from mavros_msgs.srv import CommandBool, CommandBoolResponse

class MURSimToIntoNode:
    def __init__(self):
        # Desire parameters
        self.saturation = 30.0

        # ROS parameter server
        self.config = {}

        # ROS infrastructure
        self.arming_srv = rospy.Service('/mavros/cmd/arming', CommandBool, self.arming_req)
        self.sub_cmd_pose = rospy.Subscriber('/mavros/rc/override', OverrideRCIn, self.cmd_actuators)
        self.pub_thruster_0 = rospy.Publisher('/mur/thrusters/0/input', FloatStamped, queue_size=1)
        self.pub_thruster_1 = rospy.Publisher('/mur/thrusters/1/input', FloatStamped, queue_size=1)
        self.pub_thruster_2 = rospy.Publisher('/mur/thrusters/2/input', FloatStamped, queue_size=1)
        self.pub_thruster_3 = rospy.Publisher('/mur/thrusters/3/input', FloatStamped, queue_size=1)

    def arming_req(self,req):
        return CommandBoolResponse(req.value,0)

    def vel_normalize(self, msg):
        force_actuators = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        force_size = len(force_actuators)
        for i in range (force_size):
            force_actuators[i] = mur_simtools.pwm_to_push(msg.channels[i])
        #rospy.loginfo("Force Actuators:=\n %s" %force_actuators)
        return force_actuators

    def cmd_actuators(self, msg):
        # Get the values and velocities
        force_actuators = self.vel_normalize(msg)
        self.force_callback(force_actuators)

    def force_callback(self, force_actuators):
        msg_thruster_0 = FloatStamped()
        msg_thruster_1 = FloatStamped()
        msg_thruster_2 = FloatStamped()
        msg_thruster_3 = FloatStamped()
        msg_thruster_0.header.stamp = rospy.Time.now()
        msg_thruster_0.header.frame_id = 'mur/thrusters/0'
        msg_thruster_1.header.stamp = rospy.Time.now()
        msg_thruster_1.header.frame_id = 'mur/thrusters/1'
        msg_thruster_2.header.stamp = rospy.Time.now()
        msg_thruster_2.header.frame_id = 'mur/thrusters/2'
        msg_thruster_3.header.stamp = rospy.Time.now()
        msg_thruster_3.header.frame_id = 'mur/thrusters/3'
        msg_thruster_0.data = force_actuators[0]
        msg_thruster_1.data = force_actuators[1]
        msg_thruster_2.data = force_actuators[2]
        msg_thruster_3.data = force_actuators[3]
        rospy.logdebug("Force Actuators:=\n %s" %force_actuators)
        self.pub_thruster_0.publish(msg_thruster_0)
        self.pub_thruster_1.publish(msg_thruster_1)
        self.pub_thruster_2.publish(msg_thruster_2)
        self.pub_thruster_3.publish(msg_thruster_3)

if __name__ == '__main__':
    rospy.init_node('mur_sim_to_into_node')
    try:
        node = MURSimToIntoNode()
        rate = rospy.Rate(50)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
