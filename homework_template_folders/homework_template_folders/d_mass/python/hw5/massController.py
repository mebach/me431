import numpy as np
import massParamHW5 as P
import sys
sys.path.append('..')  # add parent directory
import massParam as P0

class massController:
    def __init__(self):

        # instantiate the PD object
        self.kp = P.kp
        self.kd = P.kd
        self.limit = P0.F_max

    def update(self, z_r, x):
        z = x.item(0)
        zdot = x.item(1)

        # feedback linearized force
        F_fl = P0.k * z_r

        # equilibrium force around z_e = 0
        z_e = 0.0
        F_e = 0.0

        # compute the linearized torque using PD control
        F_tilde = self.kp * (z_r - z) - self.kd * zdot

        # compute total force
        F = F_fl + F_tilde

        # always saturate to protect the hardware
        F = self.saturate(F)

        return F

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u


