import numpy as np
import VTOLParamHW5 as P


class VTOLController:
    def __init__(self):
        self.kp_h = P.kp_h
        self.kd_h = P.kd_h
        self.kp_th = P.kp_th
        self.kd_th = P.kd_th
        self.kp_z = P.kp_z
        self.kd_z = P.kd_z

    def update(self, reference, state):
        z_r = reference.item(0)
        h_r = reference.item(1)

        z = state.item(0)
        h = state.item(1)
        theta = state.item(2)
        zdot = state.item(3)
        hdot = state.item(4)
        thetadot = state.item(5)

        F_tilde = self.kp_h * (h_r - h) - self.kd_h * hdot
        F = F_tilde + P.Fe

        theta_r = self.kp_z * (z_r - z) - self.kd_z * zdot
        tau = self.kp_th * (theta_r - theta) - self.kd_th * thetadot

        return np.array([[F], [tau]])
