#!/usr/bin/env python

import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def convert_body_world(pose_rot):
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

def rot2(ang):
        ct = np.cos(ang);
        st = np.sin(ang);
        R=np.matrix([[ct,-st],[st,ct]])
        return R
