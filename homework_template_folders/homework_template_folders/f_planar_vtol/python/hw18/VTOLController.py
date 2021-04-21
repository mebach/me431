import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P
import loopshape_VTOL_lon as L_lon
import loopshape_VTOL_lat_in as L_lat_in
import loopshape_VTOL_lat_out as L_lat_out
import numpy as np
from discreteFilter import discreteFilter

class VTOLController:
    def __init__(self):
        self.control_lon = discreteFilter(L_lon.C.num, L_lon.C.den, P.Ts)
        self.prefilter_lon = discreteFilter(L_lon.F.num, L_lon.F.den, P.Ts)
        self.control_lat_in = discreteFilter(L_lat_in.C.num, L_lat_in.C.den, P.Ts)
        self.control_lat_out = discreteFilter(L_lat_out.C.num, L_lat_out.C.den, P.Ts)
        self.prefilter_lat_out = discreteFilter(L_lat_out.F.num, L_lat_out.F.den, P.Ts)
        self.limit = P.f_max  # Maximum force

    def update(self, r, y):
        z_r = r.item(0)
        h_r = r.item(1)
        z = y.item(0)
        h = y.item(1)
        theta = y.item(2)

        # -----Longitudinal Control ------
        # prefilter
        h_r_filtered = self.prefilter_lon.update(h_r)

        # error signal for longitudinal loop
        error_lon = h_r_filtered - h

        # longitudinal controller
        F_tilde = self.control_lon.update(error_lon)

        # total force
        F = P.Fe + F_tilde

        # -----Lateral Control ------
        # +++++Lateral Outer Loop +++++
        z_r_filtered = self.prefilter_lat_out.update(z_r)

        # error signal for outer loop
        error_lat_out = z_r_filtered - z
        # Outer loop control
        theta_r = self.control_lat_out.update(error_lat_out)

        # +++++Lateral Inner Loop +++++
        # error signal for inner loop
        error_lat_in = theta_r - theta
        # Inner loop control
        tau = self.control_lat_in.update(error_lat_in)

        return np.array([[F], [tau]])


    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

