import numpy as np
import VTOLParamHW9 as P

class VTOLController:
    def __init__(self):
        self.integrator_long = 0.0
        self.error_d1_long = 0.0
        self.K_long = P.K_long
        self.ki_long = P.ki_long

        self.integrator_lat = 0.0
        self.error_d1_lat = 0.0
        self.K_lat = P.K_lat
        self.ki_lat = P.ki_lat

        self.limit = P.f_max
        self.Ts = P.Ts

    def update(self, r, x):
        # pull out the references from r
        z_r = r.item(0)
        h_r = r.item(1)

        # pull out the states from x
        z = x.item(0)
        h = x.item(1)
        theta = x.item(2)
        zdot = x.item(3)
        hdot = x.item(4)
        thetadot = x.item(5)

        # split the states into two separate state vectors x_long and x_lat
        state_long = np.array([[h],
                               [hdot]])
        state_lat = np.array([[z],
                              [theta],
                              [zdot],
                              [thetadot]])

        # integrate error
        error_long = h_r - h
        self.integrateError_long(error_long)

        error_lat = z_r - z
        self.integrateError_lat(error_lat)

        F_tilde = -self.K_long @ state_long - self.ki_long * self.integrator_long
        F = F_tilde + P.Fe

        tau = -self.K_lat @ state_lat - self.ki_lat * self.integrator_lat

        return np.array([[F], [tau]])

    def integrateError_long(self, error_long):
        self.integrator_long = self.integrator_long + (self.Ts/2.0) * (error_long + self.error_d1_long)
        self.error_d1_long = error_long

    def integrateError_lat(self, error_lat):
        self.integrator_lat = self.integrator_lat + (self.Ts/2.0) * (error_lat + self.error_d1_lat)
        self.error_d1_lat = error_lat

