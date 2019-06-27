# -*- coding: utf-8 -*-
"""
Created on Thu Jun 27 11:48:44 2019

@author: Baris ALHAN
"""

import numpy as np

class PID:
    
    def __init__(self, kf=1, kn=1, ki=5):

        self.kf = kf
        self.kn = kn
        self.ki = ki
        self.sumError = 0

    def update(self, near_error, far_error, dt):
        
        self.sumError += near_error * dt

        u = np.multiply(self.kf, far_error) + np.multiply(self.kn, near_error) + np.multiply(self.ki, self.sumError)
        if len(u[u < -30 * np.pi / 180]) > 0:
            u[u < -30 * np.pi / 180] = -30 * np.pi / 180
        elif len(u[u > 30 * np.pi / 180]) > 0:
            u[u > 30 * np.pi / 180] = 30 * np.pi / 180
        return u