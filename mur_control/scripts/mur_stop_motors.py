#! /usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
from mavros_msgs.msg import OverrideRCIn

class MURStopMotors():
    def __init__(self):

        # ROS infrastructure
        self.pub_actuators = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=5)

        # Callback
        self.epochs = 5
        for i in range(self.epochs):
            self.shutdown_motors()
        rospy.logwarn("The motors have been stopped by themselves.")

    def shutdown_motors(self):
        msg_actuators = OverrideRCIn()
        msg_actuators.channels = np.array([1500,1500,1500,1500,0,0,0,0])
        rospy.loginfo("Motors Values:= %s", msg_actuators.channels)
        self.pub_actuators.publish(msg_actuators)


if __name__ == '__main__':
    rospy.init_node('mur_stop_motors')
    try:
        node = MURStopMotors()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
