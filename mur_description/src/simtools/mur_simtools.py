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
