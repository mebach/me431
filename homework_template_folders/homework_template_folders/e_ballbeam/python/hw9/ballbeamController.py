import numpy as np
import ballbeamParamHW9 as P
import sys
sys.path.append('..')
import ballbeamParam as P0

class ballbeamController:
    def __init__(self):

        self.error_d1 = 0.0
        self.K = P.K  # state feedback gain
        x = np.array([[P0.z0], [P0.theta0],[P0.zdot0], [P0.thetadot0]])
        self.ki = P.ki  # Input gain
        self.integrator = self.K @ x / (-self.ki)
        self.limit = P.F_max  # Maxiumum force
        self.Ts = P.Ts  # sample rate of controller

    def update(self, z_r, x):
        z = x.item(0)
        error = z_r - z
        self.integrateError(error)

        Fe = P.m1*P.g*(P.ze/P.length) + P.m2*P.g/2.0

        # Compute the state feedback controller
        F_unsat = -self.K @ x - self.ki * self.integrator
        F_sat = self.saturate(F_unsat + Fe)
        return F_sat.item(0)

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_d1)

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

