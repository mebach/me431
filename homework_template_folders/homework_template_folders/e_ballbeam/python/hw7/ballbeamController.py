import numpy as np
import ballbeamParamHW7 as P7
import sys
sys.path.append('..')
import ballbeamParam as P
from PIDControl import PIDControl

class ballbeamController:
    def __init__(self):
        # Instantiate the SS_ctrl object
        self.thetaCtrl = PIDControl(P7.kp_th, P7.ki_z, P7.kd_th, P7.F_max, P7.sigma, P7.Ts)
        self.zCtrl = PIDControl(P7.kp_z, 0.0, P7.kd_z, 1e3, P7.sigma, P7.Ts)
        # self.filter = zeroCancelingFilter()

    def update(self, z_r, y):
        z = y.item(0)
        theta = y.item(1)

        theta_r = self.zCtrl.PID(z_r, z, flag=False)

        # z_r = self.filter.update(z_r)
        F_fl = P.g * (P.m1 * z / P.length + P.m2 / 2.0)

        F_tilde = self.thetaCtrl.PD(theta_r, theta, flag=False)

        F = F_tilde + F_fl
        return F

class zeroCancelingFilter:
    def __init__(self):
        self.a = -3.0/(2.0*P.ell*P7.DC_gain)
        self.b = np.sqrt(3.0*P.g/(2.0*P.ell))
        self.state = 0.0

    def update(self, input):
        # integrate using RK1
        self.state = self.state \
                     + P.Ts * (-self.b*self.state + self.a*input)
        return self.state
