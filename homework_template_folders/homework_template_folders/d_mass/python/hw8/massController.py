import numpy as np
import massParamHW8 as P

class massController:

    def __init__(self):
        self.K = P.K
        self.kr = P.kr
        self.limit = P.F_max
        self.Ts = P.Ts

    def update(self, z_r, x):
        z = x.item(0)
        # zdot = x.item(1)

        # compute feedback linearizing force F_fl
        F_fl = P.k * z

        # compute the state feedback controller
        F_tilde = -self.K @ x + self.kr * z_r

        # compute the total torque
        F = self.saturate(F_tilde[0, 0] + F_fl)

        return F

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit * np.sign(u)
        return u

