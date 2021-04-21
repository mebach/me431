import massParam as P
import loopshape_mass as L
import numpy as np
from discreteFilter import discreteFilter

class controllerLoop:
    def __init__(self):
        self.A_C = L.Css.A
        self.B_C = L.Css.B
        self.C_C = L.Css.C
        self.D_C = L.Css.D
        self.A_F = L.Fss.A
        self.B_F = L.Fss.B
        self.C_F = L.Fss.C
        self.D_F = L.Fss.D
        n = self.A_C.shape[0]
        self.x_C = np.zeros((n, 1))
        n = self.A_F.shape[0]
        self.x_F = np.zeros((n, 1))
        self.limit = P.F_max
        self.F_e = P.F_e
        self.Ts = P.Ts

        self.prefilter = discreteFilter(L.F.num, L.F.den, P.Ts)
        self.control = discreteFilter(L.C.num, L.C.den, P.Ts)

    def update(self, z_r, y):
        z = y.item(0)

        # prefilter
        z_r_filtered = self.prefilter.update(z_r)

        # error signal
        error = z_r_filtered - z

        # find the control
        F_tilde = self.control.update(error)

        # compute total force
        F = self.saturate(F_tilde + self.F_e)

        return F.item(0)

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u