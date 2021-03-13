import VTOLParamHW7 as P
from PIDControl import PIDControl
import numpy as np

class VTOLController:
    def __init__(self):
        # Instantiates the SS_ctrl object
        self.hCtrl = PIDControl(P.kp_h, P.ki_h, P.kd_h,
                                  np.pi, P.beta, P.Ts)
        self.thetaCtrl = PIDControl(P.kp_th, 0.0, P.kd_th,
                                    P.f_max, P.beta, P.Ts)
        self.zCtrl = PIDControl(P.kp_z, P.ki_z, P.kd_z, 10.0, P.beta, P.Ts)

    def update(self, r, y):
        z = y.item(0)
        h = y.item(1)
        theta = y.item(2)

        z_r = r.item(0)
        h_r = r.item(1)

        F_tilde = self.hCtrl.PID(h_r, h, flag=False)
        F = F_tilde + P.Fe

        theta_r = self.zCtrl.PID(z_r, z, flag=False)
        tau = self.thetaCtrl.PID(theta_r, theta, flag=False)


        return np.array([[F], [tau]])








