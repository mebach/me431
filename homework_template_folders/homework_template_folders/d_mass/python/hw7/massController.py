import numpy as np
import massParamHW7 as P
import sys
sys.path.append('..')  # add parent directory
import massParam as P0
from PIDControl import PIDControl

class massController:
    def __init__(self):

        # instantiate the PD object
        self.zCtrl = PIDControl(P.kp, P.ki, P.kd, P0.F_max, P.beta, P.Ts)

        self.limit = P0.F_max
    def update(self, z_r, y):
        z = y.item(0)
        # zdot = x.item(1)

        # feedback linearized force
        F_fl = P0.k * z_r

        # equilibrium force around z_e = 0
        z_e = 0.0
        F_e = 0.0

        # compute the linearized torque using PD control
        F_tilde = self.zCtrl.PID(z_r, z, False)

        # compute total force
        F = F_fl + F_tilde

        # always saturate to protect the hardware
        F = self.saturate(F)

        return F

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u


