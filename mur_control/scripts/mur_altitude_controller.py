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
DT_VEL = 2
STATION_KEEPING = 2
# Let's initialize the controller gain matrices Kp, Kd and Ki
Kp = np.zeros(shape=(6, 6))
Kd = np.zeros(shape=(6, 6))
Ki = np.zeros(shape=(6, 6))
# Initialize the integrator component
integrator_comp = np.zeros(shape=(6,))
# Initialize variable that will store the vehicle pose error
error_pose = np.zeros(shape=(6,))
error_vel = np.zeros(shape=(6,))
# Init the force
force = np.zeros(shape=(6,))
force_pub = rospy.Publisher('/control/Wrench', WrenchStamped, queue_size=1)

# Validation of the parameters
if rospy.get_param('/mur/mur_altitude_controller/x'):
    pos_x = rospy.get_param('/mur/mur_altitude_controller/x')
else:
    pos_x = 0.0

if rospy.get_param('/mur/mur_altitude_controller/y'):
    pos_y = rospy.get_param('/mur/mur_altitude_controller/y')
else:
    pos_y = 0.0

if rospy.get_param('/mur/mur_altitude_controller/z'):
    pos_z = rospy.get_param('/mur/mur_altitude_controller/z')
else:
    pos_z = -20.0

if rospy.get_param('/mur/mur_altitude_controller/phi'):
    pos_phi = rospy.get_param('/mur/mur_altitude_controller/phi')
else:
    pos_phi = 0.0

if rospy.get_param('/mur/mur_altitude_controller/theta'):
    pos_theta = rospy.get_param('/mur/mur_altitude_controller/theta')
else:
    pos_theta = 0.0

if rospy.get_param('/mur/mur_altitude_controller/psi'):
    pos_psi = rospy.get_param('/mur/mur_altitude_controller/psi')
else:
    pos_psi = 0.0

nitad = np.array([pos_x, pos_y, pos_z, pos_phi, pos_theta, pos_psi])

# The control constants
if rospy.get_param('/mur/mur_altitude_controller/Kp'):
    Kp_diag = rospy.get_param('/mur/mur_altitude_controller/Kp')
    if len(Kp_diag) == 6:
        Kp = np.diag(Kp_diag)
    else:
        # If the vector provided has the wrong dimension, raise an exception
        raise rospy.ROSException('For the Kp diagonal matrix, 6 coefficients are needed')

# Do the same for the other two matrices
if rospy.get_param('/mur/mur_altitude_controller/Kd'):
    diag = rospy.get_param('/mur/mur_altitude_controller/Kd')
    if len(diag) == 6:
        Kd = np.diag(diag)
        print 'Kd=\n', Kd
    else:
        # If the vector provided has the wrong dimension, raise an exception
        raise rospy.ROSException('For the Kd diagonal matrix, 6 coefficients are needed')

if rospy.get_param('/mur/mur_altitude_controller/Ki'):
    diag = rospy.get_param('/mur/mur_altitude_controller/Ki')
    if len(diag) == 6:
        Ki = np.diag(diag)
        print 'Ki=\n', Ki
    else:
        # If the vector provided has the wrong dimension, raise an exception
        raise rospy.ROSException('For the Ki diagonal matrix, 6 coefficients are needed')

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

def get_posegt(msg):
    global error_pose
    global error_vel
    global nitad
    # Get the position and velocities
    pose_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
    pose_rot = np.array([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    twist_pos = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
    twist_rot = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
    # Convert to global frame
    nita2_t = euler_from_quaternion(pose_rot)
    nita2 = np.array([nita2_t[0],nita2_t[1],nita2_t[2]])
    nita = np.array([pose_pos[0],pose_pos[1],pose_pos[2], nita2[0],nita2[1],nita2[2]]).reshape(nitad.shape)
    J = convert_to_global_frame(pose_rot)
    vitad = np.zeros(nitad.shape)
    vita_t = np.array([twist_pos[0],twist_pos[1],twist_pos[2], twist_rot[0],twist_rot[1],twist_rot[2]]).reshape(vitad.shape)
    vita = np.matmul(J,vita_t)
    # Create the errors
    error_pose = nitad - nita
    for i in range(len(error_pose)):
        vitad[i]=error_pose[i]/DT_VEL
    error_vel = vitad - vita
    set_force()

def set_force():
    global error_pose
    global error_vel
    global Kp, Kd
    global force_pub

    force=np.dot(Kp,error_pose)+np.dot(Kd,error_vel)

    rospy.loginfo("Force := \n%s" %force)
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
    rospy.Subscriber('/mur/pose_gt', Odometry, get_posegt)
    rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('mur_altitude_controller')
        rospy.Rate(RATE) # 10hz

        listener()

    except rospy.ROSInterruptException: pass

