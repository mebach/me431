import numpy as np
import VTOLParamHW10 as P

class VTOLController:
    def __init__(self):
        self.x_hat_long = np.array([[0.0],[0.0]])
        self.F_d1_long = 0.0
        self.integrator_long = 0.0
        self.error_d1_long = 0.0
        self.K_long = P.K_long
        self.ki_long = P.ki_long
        self.L_long = P.L_long
        self.A_long = P.A_long
        self.B_long = P.B_long
        self.C_long = P.C_long

        self.x_hat_lat = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.tau_d1_lat = 0.0
        self.integrator_lat = 0.0
        self.error_d1_lat = 0.0
        self.K_lat = P.K_lat
        self.ki_lat = P.ki_lat
        self.L_lat = P.L_lat
        self.A_lat = P.A_lat
        self.B_lat = P.B_lat
        self.C_lat = P.C_lat

        self.limit = P.f_max
        self.Ts = P.Ts

    def update(self, r, y):
        # pull out the references from r
        z_r = r.item(0)
        h_r = r.item(1)

        # pull out the states from x
        z = y.item(0)
        h = y.item(1)
        theta = y.item(2)

        x_hat_long = self.update_observer_long(h)
        x_hat_lat = self.update_observer_lat(np.array([[z], [theta]]))

        # split the states into two separate state vectors x_long and x_lat
        # state_long = np.array([[h],
        #                        [hdot]])
        # state_lat = np.array([[z],
        #                       [theta],
        #                       [zdot],
        #                       [thetadot]])

        # integrate error
        error_long = h_r - h
        self.integrateError_long(error_long)

        error_lat = z_r - z
        self.integrateError_lat(error_lat)

        F_tilde = -self.K_long @ x_hat_long - self.ki_long * self.integrator_long
        F = F_tilde + P.Fe
        self.F_d1_long = F

        tau = -self.K_lat @ x_hat_lat - self.ki_lat * self.integrator_lat
        self.tau_d1_lat = tau

        x_hat = np.array([[x_hat_lat.item(0)],
                          [x_hat_long.item(0)],
                          [x_hat_lat.item(1)],
                          [x_hat_lat.item(2)],
                          [x_hat_long.item(1)],
                          [x_hat_lat.item(3)]])

        return np.array([[F.item(0)], [tau.item(0)]]), x_hat


    def integrateError_long(self, error_long):
        self.integrator_long = self.integrator_long + (self.Ts/2.0) * (error_long + self.error_d1_long)
        self.error_d1_long = error_long

    def integrateError_lat(self, error_lat):
        self.integrator_lat = self.integrator_lat + (self.Ts/2.0) * (error_lat + self.error_d1_lat)
        self.error_d1_lat = error_lat

    def update_observer_long(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f_long(self.x_hat_long, y_m)
        F2 = self.observer_f_long(self.x_hat_long + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f_long(self.x_hat_long + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f_long(self.x_hat_long + self.Ts * F3, y_m)
        self.x_hat_long += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

        return self.x_hat_long

    def observer_f_long(self, x_hat_long, y_m):
        # xhatdot = A*(xhat-xe) + B*(u-ue) + L(y-C*xhat)
        xhat_dot_long = self.A_long @ x_hat_long \
                   + self.B_long * (self.F_d1_long - P.Fe) \
                   + self.L_long * (y_m - self.C_long @ x_hat_long)

        return xhat_dot_long

    def update_observer_lat(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f_lat(self.x_hat_lat, y_m)
        F2 = self.observer_f_lat(self.x_hat_lat + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f_lat(self.x_hat_lat + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f_lat(self.x_hat_lat + self.Ts * F3, y_m)
        self.x_hat_lat += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

        return self.x_hat_lat

    def observer_f_lat(self, x_hat_lat, y_m):
        # xhatdot = A*(xhat-xe) + B*(u-ue) + L(y-C*xhat)
        xhat_dot_lat = self.A_lat @ x_hat_lat \
                        + self.B_lat * self.tau_d1_lat \
                        + self.L_lat @ (y_m - self.C_lat @ x_hat_lat)
        return xhat_dot_lat
