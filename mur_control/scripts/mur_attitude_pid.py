#!/usr/bin/python

import numpy as np
import rospy
import logging
import sys
import tf
from common import mur_common, mur_PID
from threading import Timer
from dynamic_reconfigure.server import Server
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped,Vector3, Quaternion, Pose
from std_msgs.msg import Time
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, StatusText
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        print "Repeated Timer Stop"
        self.is_running = False


class MURAttitudePIDNode(RepeatedTimer):
    def __init__(self):
        #Timer values
        self.dt = 0.01
        RepeatedTimer.__init__(self,self.dt, self.force_pub)
        self.t_1 = 0.0
        self.t = 0.0

        # Pose values
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        #Vel values
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.z_vel = 0.0
        self.x_avel = 0.0
        self.y_avel = 0.0
        self.z_avel = 0.0

        # Desired Pose values
        self.x_d = 0.0
        self.y_d = 0.0
        self.z_d = -1.0
        self.roll_d = 0.0
        self.pitch_d = 0.0
        self.yaw_d = 0.0

        # Control gains
        self.p_x = 0.03
        self.d_x = -0.09

        self.p_y = -0.06
        self.d_y = 0.09

        self.p_z = 22.0
        self.i_z = 0.02
        self.d_z = -45.0

        self.p_r = 10.0
        self.i_r = 0.25
        self.d_r = -12.0

        self.p_p = 10.0
        self.i_p = 0.2
        self.d_p = -12.0

        self.p_yaw = 10.0
        self.i_yaw = 0.2
        self.d_yaw = -12.0

        # PID Control
        self.pid_x      = mur_PID.mur_PID(self.p_x,0.0,self.d_x,0.5)
        self.pid_y      = mur_PID.mur_PID(self.p_y,0.0,self.d_y,0.5)
        self.pid_z      = mur_PID.mur_PID(self.p_z, self.i_z, self.d_z, 120.0)
        self.pid_pitch  = mur_PID.mur_PID(self.p_p, self.i_p, self.d_p, 6.0)
        self.pid_roll   = mur_PID.mur_PID(self.p_r, self.i_r, self.d_r, 6.0)
        self.pid_yaw    = mur_PID.mur_PID(self.p_yaw, self.i_yaw, self.d_yaw, 6.0)

        # ROS infrastructure
        self.sub_cmd_pose = rospy.Subscriber('/mur/odom_filtered', Odometry, self.cmd_pose_callback)
        self.pub_cmd_force = rospy.Publisher('/mur/force_input', WrenchStamped, queue_size=1)

    def cmd_pose_callback(self, msg):
        # Get Pose
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        pose_rot = np.array([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.t = msg.header.stamp.to_sec()
        self.pitch,self.roll,self.yaw= euler_from_quaternion(pose_rot)
        #rospy.loginfo("Angles:= \n %s \n %s \n %s," %(self.roll,self.pitch,self.yaw))

    def force_pub(self):
        # Control
        error_x = self.x_d - self.x
        error_y = self.y_d - self.y
        rot_mat=mur_common.rot2(self.yaw)
        rot2d_mat = np.transpose(rot_mat)
        x_rel, y_rel = np.matmul(rot2d_mat,np.array([[error_x],[error_y]]))
        #rospy.loginfo("Pose Relative:= %s" %np.array([x_rel, y_rel]))
        self.roll_d = self.pid_x.controlate(x_rel, -self.x_vel, self.dt)
        self.pitch_d = self.pid_y.controlate(y_rel, -self.y_vel, self.dt)
        #rospy.loginfo("Angle Desired:= %s" %np.array([self.roll_d, self.pitch_d]))
        F_z = self.pid_z.controlate((self.z_d - self.z),-self.z_vel,self.dt) + 22.00
        T_p = self.pid_pitch.controlate((self.pitch_d - self.pitch),-self.x_avel,self.dt)
        T_r = self.pid_roll.controlate((self.roll_d - self.roll),-self.y_avel,self.dt)
        T_y = 0.0 #self.pid_yaw.controlate(self.error_pos[5,],-self.vita[5,],self.dt)
        #rospy.loginfo("Forces:= %s" %np.array([[F_z],[T_p],[T_r],[T_y]]))
        force_msg = WrenchStamped()
        force_msg.header.stamp = rospy.Time.now()
        force_msg.header.frame_id = 'mur/base_link'
        force_msg.wrench.force.z = F_z
        force_msg.wrench.torque.x = T_p
        force_msg.wrench.torque.y = T_r
        force_msg.wrench.torque.z = T_y
        self.pub_cmd_force.publish(force_msg)


if __name__ == '__main__':
    rospy.init_node('mur_attitude_pid')
    try:
        node = MURAttitudePIDNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    if rospy.is_shutdown():
        node.stop()
    print('exiting')
