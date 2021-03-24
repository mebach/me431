import numpy as np
import massParamHW9 as P

class massController:

    def __init__(self):
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0  # error signal delayed by 1 sample
        self.K = P.K
        self.ki = P.ki
        self.limit = P.F_max
        self.Ts = P.Ts

    def update(self, z_r, x):
        z = x.item(0)

        # integrate error
        error = z_r - z
        self.integrateError(error)

        # compute feedback linearizing force F_fl
        F_fl = P.k * z

        # compute the state feedback controller
        F_tilde = -self.K @ x - self.ki*self.integrator

        # compute the total force
        F_sat = self.saturate(F_tilde[0, 0] + F_fl)

        if self.ki != 0.0:
            self.integrator = self.integrator + (1.0 / self.ki) * (F_sat - (F_tilde[0, 0] + F_fl))

        return F_sat

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit * np.sign(u)
        return u

