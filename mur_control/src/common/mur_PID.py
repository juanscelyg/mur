#!/usr/bin/env python

import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class mur_PID():
    def __init__(self, p, i, d, sat):
        self.p = p
        self.i = i
        self.d = d
        self.sat = sat

        self.integral = 0
        self.e_1 = 0
        self.t_1 = -1.0

    def controlate(self, e, t):
        de = 0.0
        dt = t - self.t_1
        if self.t_1 > 0.0 and dt > 0.0:
            de = (e - self.e_1)/dt
            self.integral += 0.5*(e + self.e_1)*dt

        u = self.p*e + self.d*de + self.i*self.integral

        self.e_1 = e
        self.t_1 = t

        if (np.linalg.norm(u) > self.sat):
            # controller is in saturation: limit outpt, reset integral
            u = self.sat*u/np.linalg.norm(u)
            self.integral = 0.0

        return u
