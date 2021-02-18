import numpy as np
import ballbeamParamHW5 as P


class ballbeamController:
    def __init__(self):
        self.kp_th = P.kp_th
        self.kd_th = P.kd_th
        self.kp_z = P.kp_z
        self.kd_z = P.kd_z
        # self.filter = zeroCancelingFilter()

    def update(self, z_r, state):
        z = state.item(0)
        theta = state.item(1)
        zdot = state.item(2)
        thetadot = state.item(3)

        # the reference z comes from the outer loop PD control
        theta_r = self.kp_z * (z_r - z) - self.kd_z * zdot

        # low pass filter the outer loop to cancel left-half plane zero and DC-gain
        # theta_r = self.filter.update(tmp)

        # the force applied to the beam from the inner loop control
        F = self.kp_th * (theta_r - theta) - self.kd_th * thetadot

        return F

# class zeroCancelingFilter:
#     def __init__(self):
#         self.a = -3.0 / (2.0 * P.ell * P.DC_gain)
#         self.b = np.sqrt(3.0 * P.g / (2.0 * P.ell))
#         self.state = 0.0
#
#     def update(self, input):
#         # integrate using RK1
#         self.state = self.state \
#                      + P.Ts * (-self.b * self.state + self.a * input)
#         return self.state

