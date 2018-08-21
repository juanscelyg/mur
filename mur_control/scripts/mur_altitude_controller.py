#!/usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped,Vector3, Quaternion, Pose
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply, quaternion_matrix, quaternion_conjugate, quaternion_inverse

# Init the desired dt
RATE = 10
# Let's initialize the controller gain matrices Kp, Kd and Ki
Kp = np.zeros(shape=(6, 6))
Kd = np.zeros(shape=(6, 6))
Ki = np.zeros(shape=(6, 6))
# Initialize the integrator component
integrator_comp = np.zeros(shape=(6,))
# Initialize variable that will store the vehicle pose error
error_pose = np.zeros(shape=(6,))
# Init the force
force = np.zeros(shape=(6,))
force_pub = rospy.Publisher('/control/Wrench', WrenchStamped, queue_size=1)

'''
# Validation of the parameters
if rospy.get_param('~x'):
    pos_x = rospy.get_param('~x')
else:
    pos_x = 0.0

if rospy.get_param('~y'):
    pos_y = rospy.get_param('~y')
else:
    pos_y = 0.0

if rospy.get_param('~z'):
    pos_z = rospy.get_param('~z')
else:
    pos_z = -20.0

if rospy.get_param('~phi'):
    pos_phi = rospy.get_param('~phi')
else:
    pos_phi = 0.0

if rospy.get_param('~theta'):
    pos_theta = rospy.get_param('~theta')
else:
    pos_theta = 0.0

if rospy.get_param('~psi'):
    pos_psi = rospy.get_param('~psi')
else:
    pos_psi = 0.0

nitad=np.array([pos_x, pos_y, pos_z, pos_phi, pos_theta, pos_psi])

# The control constants
if rospy.get_param('~Kp'):
    Kp_diag = rospy.get_param('~Kp')
    if len(Kp_diag) == 6:
        Kp = np.diag(Kp_diag)
    else:
        # If the vector provided has the wrong dimension, raise an exception
        raise rospy.ROSException('For the Kp diagonal matrix, 6 coefficients are needed')

# Do the same for the other two matrices
if rospy.get_param('~Kd'):
    diag = rospy.get_param('~Kd')
    if len(diag) == 6:
        Kd = np.diag(diag)
        print 'Kd=\n', Kd
    else:
        # If the vector provided has the wrong dimension, raise an exception
        raise rospy.ROSException('For the Kd diagonal matrix, 6 coefficients are needed')

if rospy.get_param('~Ki'):
    diag = rospy.get_param('~Ki')
    if len(diag) == 6:
        Ki = np.diag(diag)
        print 'Ki=\n', Ki
    else:
        # If the vector provided has the wrong dimension, raise an exception
        raise rospy.ROSException('For the Ki diagonal matrix, 6 coefficients are needed')
'''
pos_x=0.0
pos_y=0.0
pos_z=-20.0
pos_phi=0.0
pos_theta=0.0
pos_psi=0.0
Kp=np.diag(np.array([10,10,10,10,10,10]))

def get_posegt(msg):
    global error_pose

    pose_pos = msg.pose.pose.position
    pose_rot = msg.pose.pose.orientation
    nita2 = euler_from_quaternion(pose_rot)
    error_pose = nitad - np.array([pose_pos, nita2])
    set_force()

def set_force():
    global error_pose
    global Kp

    force=Kp*error_pose

    force_msg = WrenchStamped()
    force_msg.header.stamp = rospy.Time.now()
    force_msg.header.frame_id = 'mur/control'
    force_msg.wrench.force.x = force[0]
    force_msg.wrench.force.y = force[1]
    force_msg.wrench.force.z = force[2]

    force_msg.wrench.torque.x = force[3]
    force_msg.wrench.torque.y = force[4]
    force_msg.wrench.torque.z = force[5]

    force_pub.publish(force_msg)


def listener():
    rospy.Subscriber('/mur/pose_get', Odometry, get_posegt)

    rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('mur_altitude_controller')
        rospy.Rate(RATE) # 10hz

        listener()

    except rospy.ROSInterruptException: pass

