import numpy as np
import VTOLParamHW8 as P

class VTOLController:
    def __init__(self):
        self.K_long = P.K_long
        self.kr_long = P.kr_long
        self.K_lat = P.K_lat
        self.kr_lat = P.kr_lat
        self.limit = P.f_max
        self.Ts = P.Ts

    def update(self, r, x):
        z_r = r.item(0)
        h_r = r.item(1)

        z = x.item(0)
        h = x.item(1)
        theta = x.item(2)
        zdot = x.item(3)
        hdot = x.item(4)
        thetadot = x.item(5)

        state_long = np.array([[h],
                               [hdot]])
        state_lat = np.array([[z],
                              [theta],
                              [zdot],
                              [thetadot]])

        F_tilde = -self.K_long @ state_long + self.kr_long * h_r
        F = F_tilde + P.Fe

        tau =  -self.K_lat @ state_lat + self.kr_lat * z_r

        return np.array([[F], [tau]])

