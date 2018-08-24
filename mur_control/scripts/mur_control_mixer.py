#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
import message_filters
from mur_control.msg import FloatStamped
from geometry_msgs.msg import WrenchStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry

# Constants
NUM_THRUSTERS = 4
ANGLE = 0
X_BAR = 0.197
Y_BAR = 0.18022
# Init the desired dt
RATE = 10
# Matrix allocator
T = np.matrix([[0, 0, 0, 0], [np.sin(np.deg2rad(ANGLE)), np.sin(np.deg2rad(-ANGLE)), np.sin(np.deg2rad(-ANGLE)), np.sin(np.deg2rad(ANGLE))], [1,1,1,1], [np.cos(np.deg2rad(ANGLE))*Y_BAR, -np.cos(np.deg2rad(-ANGLE))*Y_BAR, -np.cos(np.deg2rad(-ANGLE))*Y_BAR, np.cos(np.deg2rad(ANGLE))*Y_BAR], [-X_BAR, -X_BAR, -X_BAR, -X_BAR], [-np.sin(np.deg2rad(ANGLE))*X_BAR, np.sin(np.deg2rad(-ANGLE))*X_BAR, -np.sin(np.deg2rad(-ANGLE))*X_BAR, np.sin(np.deg2rad(ANGLE))*X_BAR]])
# Topics to Publish the info
thruster_0_pub = rospy.Publisher('/mur/thrusters/0/input', FloatStamped, queue_size=1)
thruster_1_pub = rospy.Publisher('/mur/thrusters/1/input', FloatStamped, queue_size=1)
thruster_2_pub = rospy.Publisher('/mur/thrusters/2/input', FloatStamped, queue_size=1)
thruster_3_pub = rospy.Publisher('/mur/thrusters/3/input', FloatStamped, queue_size=1)
if rospy.get_param('/mur/mur_control_mixer/saturation'):
    thruster_saturation = rospy.get_param('/mur/mur_control_mixer/saturation')
else:
    thruster_saturation = 30.0

def saturator_thruster(force):
    global thruster_saturation
    thrusters = np.empty_like(force)

    for i in range(len(force)):
        if force[i]<-thruster_saturation:
            thrusters[i]=-thruster_saturation
            rospy.logwarn("The motor %s was min saturated %s",i,force[i])
        elif force[i]>thruster_saturation:
            thrusters[i]=thruster_saturation
            rospy.logwarn("The motor %s was max saturated %s",i,force[i])
        else:
            thrusters[i]=force[i]
    return thrusters

def convert_to_global_frame(pose_rot):
    nita2_t = euler_from_quaternion(pose_rot)
    nita2 = np.array([nita2_t[0],nita2_t[1],nita2_t[2]])
    r = nita2[0]
    p = nita2[1]
    y = nita2[2]
    sr = np.sin(nita2[0])
    sp = np.sin(nita2[1])
    sy = np.sin(nita2[2])
    cr = np.cos(nita2[0])
    cp = np.cos(nita2[1])
    cy = np.cos(nita2[2])
    tp = np.tan(nita2[1])
    J = np.array([[cy*cp, -sy*cr+cy*sp*sr, sy*sr+cy*cr*sp, 0, 0, 0],[sy*cp, cy*cr+sr*sp*sy, -cy*sr+sp*sy*cr, 0, 0, 0],[-sp, cp*sr, cp*cr, 0, 0, 0],[0, 0, 0, 1, sr*tp, cr*tp],[0, 0, 0, 0, cr, -sr],[0, 0, 0, 0, sr/cp, cr/cp]])
    return J


def callback(posegt, setforces):
    global T
    pose_rot = np.array([posegt.pose.pose.orientation.x,posegt.pose.pose.orientation.y,posegt.pose.pose.orientation.z,posegt.pose.pose.orientation.w])
    J = convert_to_global_frame(pose_rot)
    force = np.array([setforces.wrench.force.x, setforces.wrench.force.y, setforces.wrench.force.z])
    torque = np.array([setforces.wrench.torque.x, setforces.wrench.torque.y, setforces.wrench.torque.z])
    tau = np.array([force[0],force[1],force[2],torque[0],torque[1],torque[2]]).reshape(6,1)
    A = np.linalg.inv(np.transpose(J))
    Tt = np.matmul(A,T)
    thrusters = np.matmul(np.transpose(Tt),tau)
    thrusters = saturator_thruster(thrusters)
    rospy.loginfo("thrusters := \n%s" %thrusters)
    thruster_0_msg = FloatStamped()
    thruster_1_msg = FloatStamped()
    thruster_2_msg = FloatStamped()
    thruster_3_msg = FloatStamped()
    thruster_0_msg.header.stamp = rospy.Time.now()
    thruster_0_msg.header.frame_id = 'mur/thrusters/0'
    thruster_1_msg.header.stamp = rospy.Time.now()
    thruster_1_msg.header.frame_id = 'mur/thrusters/1'
    thruster_2_msg.header.stamp = rospy.Time.now()
    thruster_2_msg.header.frame_id = 'mur/thrusters/2'
    thruster_3_msg.header.stamp = rospy.Time.now()
    thruster_3_msg.header.frame_id = 'mur/thrusters/3'
    thruster_0_msg.data = thrusters[0]
    thruster_1_msg.data = thrusters[1]
    thruster_2_msg.data = thrusters[2]
    thruster_3_msg.data = thrusters[3]
    thruster_0_pub.publish(thruster_0_msg)
    thruster_1_pub.publish(thruster_1_msg)
    thruster_2_pub.publish(thruster_2_msg)
    thruster_3_pub.publish(thruster_3_msg)

def listener():
    posegt_sub = message_filters.Subscriber('/mur/pose_gt', Odometry)
    setforces_sub = message_filters.Subscriber('/control/Wrench', WrenchStamped)

    ts = message_filters.TimeSynchronizer([posegt_sub, setforces_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('mur_control_mixer')
        rospy.Rate(RATE) # 10 Hz

        listener()

    except rospy.ROSInterruptException:  pass
