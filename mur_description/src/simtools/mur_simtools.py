#!/usr/bin/env python

import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def pwm_to_push(pwm_value):
    pwm_value = int(pwm_value);
    if pwm_value > 1525 and pwm_value < 1900:
        params=np.array([0.0001051242,-0.263921,157.5376]);
    elif pwm_value > 1100 and pwm_value < 1475:
        params=np.array([-0.0001071748,0.3581287,-294.777]);
    else:
        pwm_value = 1500;
        params=np.array([0,0,0]);
    push = (params[0]*pwm_value**2)+(params[1]*pwm_value)+params[2]
    return push

def push_to_vel(force_value):
    if push_value > 0 and force_value < 35.0:
        params=np.array([-0.1865,13.9968,38.83]);
    elif push_value > -30.0 and force_value < 0:
        params=np.array([0.3108,17.4771,-44.55]);
    else:
        push_value = 0.0;
        params=np.array([0,0,0]);
    vel = (params[0]*force_value**2)+(params[1]*force_value)+params[2]
    return vel

def pwm_to_vel(pwm_value):
    pwm_value = int(pwm_value);
    if pwm_value > 1525 and pwm_value < 1900:
        params=np.array([-0.00092,3.9499,-3843.69]);
    elif pwm_value > 1100 and pwm_value < 1475:
        params=np.array([0.00121,-2.3882,857.9378]);
    else:
        pwm_value = 1500;
        params=np.array([0,0,0]);
    vel = (params[0]*pwm_value**2)+(params[1]*pwm_value)+params[2]
    return vel
