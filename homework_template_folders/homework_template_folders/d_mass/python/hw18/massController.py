import sys
sys.path.append('..')  # add parent directory
import massParam as P
import loopshape_mass as L
import numpy as np
from discreteFilter import discreteFilter


class massController:
    def __init__(self):
        self.prefilter = discreteFilter(L.F.num, L.F.den, P.Ts)
        self.control = discreteFilter(L.C.num, L.C.den, P.Ts)
        self.limit = P.F_max  # Maximum force
        self.Ts = P.Ts  # sample rate of controller


    def update(self, z_r, y_m):
        z = y_m.item(0)

        # prefilter
        z_r_filtered = self.prefilter.update(z_r)

        # error signal for longitudinal loop
        error = z_r_filtered - z

        # longitudinal controller
        F = self.control.update(error)

        # if we use the saturation, it significantly affects performance. We could
        # therefore shift w_co further to the left (to reduce the saturation), but
        # for illustration purposes, we leave it as is. Try returning F.item(0)
        # directly to see the difference.
        force = self.saturate(F.item(0))

        return force

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

