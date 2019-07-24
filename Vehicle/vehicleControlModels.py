# -*- coding: utf-8 -*-
"""
Created on Thu Jun 27 11:49:15 2019

@author: Baris ALHAN
"""

import numpy as np


class vehicleControlModels():

    class dynModel():

        def __init__(self):
            self.x = 0  # z[0]
            self.y = 0  # z[1]
            self.psi = 0  # z[2]
            self.v = 20  # z[3]
            self.d_f = 0  # u[0]
            self.a = 0  # u[1]
            self.L_r, self.L_f = 1.0006, 1.534
            self.dt = 0.01

        def update(self, z, u):
            # compute slip angle
            self.x = z[0]
            self.y = z[1]
            self.psi = z[2]
            self.v = z[3]
            self.d_f = u
            self.a = 0

            beta = np.arctan(
                np.multiply(self.L_r / (self.L_r + self.L_f), np.tan(self.d_f)))

            # compute next state
            x_next = self.x + self.dt * (
                np.multiply(self.v, np.cos(np.add(self.psi, beta))))
            y_next = self.y + self.dt * (
                np.multiply(self.v, np.sin(np.add(self.psi, beta))))
            psi_next = self.psi + self.dt * self.v / self.L_f * np.sin(beta)
            v_next = self.v + self.dt * self.a

            return [x_next, y_next, psi_next, v_next]

    class PID:

        def __init__(self, kf=1, kn=1, ki=5):

            self.kf = kf
            self.kn = kn
            self.ki = ki
            self.sumError = 0

        def update(self, near_error, far_error, dt):

            self.sumError += near_error * dt

            u = np.multiply(self.kf, far_error) + np.multiply(
                self.kn, near_error) + np.multiply(self.ki, self.sumError)
            if len(u[u < -30 * np.pi / 180]) > 0:
                u[u < -30 * np.pi / 180] = -30 * np.pi / 180
            elif len(u[u > 30 * np.pi / 180]) > 0:
                u[u > 30 * np.pi / 180] = 30 * np.pi / 180
            return u
