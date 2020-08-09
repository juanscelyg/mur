#! /usr/bin/env python

import numpy as np
import rospy
import logging
import sys
import tf
from mavros_msgs.msg import OverrideRCIn

def MURStopMotors():
    pub_actuators = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=5)
    rospy.init_node('mur_stop_motors', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    epochs = 5
    for i in range(epochs):
        msg_actuators = OverrideRCIn()
        msg_actuators.channels = np.array([1500,1500,1500,1500,0,0,0,0])
        rospy.loginfo("Motors Values:= %s", msg_actuators.channels)
        pub_actuators.publish(msg_actuators)
        rate.sleep()
    rospy.logwarn("The motors have been stopped by themselves.")

if __name__ == '__main__':
    try:
        MURStopMotors()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
