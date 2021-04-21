import numpy as np
import ballbeamParamHW10 as P
import sys
sys.path.append('..')
import ballbeamParam as P0

class ballbeamController:
    def __init__(self):
        self.x_hat = np.array([
            [0.0],  # initial estimate for z_hat
            [0.0],  # initial estimate for theta_hat
            [0.0],  # initial estimate for z_hat_dot
            [0.0]])  # initial estimate for theta_hat_dot
        self.F_d1 = 0.0  # Computed Force, delayed by one sample

        self.error_d1 = 0.0  # error signal delayed by 1 sample
        self.K = P.K  # state feedback gain
        self.ki = P.ki  # integrator gain
        self.error_d1 = 0.0
        self.K = P.K  # state feedback gain
        x = np.array([[P0.z0], [P0.theta0], [P0.zdot0], [P0.thetadot0]])
        self.ki = P.ki  # Input gain
        self.integrator = self.K @ x / (-self.ki) * 0.0
        self.limit = P.F_max  # Maxiumum force
        self.Ts = P.Ts  # sample rate of controller
        self.L = P.L  # observer gain
        self.A = P.A  # system model
        self.B = P.B
        self.C = P.C
        self.limit = P.F_max  # Maximum force
        self.Ts = P.Ts  # sample rate of controller
        self.Fe = P.m1*P.g*(P.ze/P.length) + P.m2*P.g/2.0
        self.xe = np.array([[P.length/2.0],
                            [0.0],
                            [0.0],
                            [0.0]])

    def update(self, z_r, y):
        # update the observer and extract z_hat
        z = y.item(0)
        x_hat = self.update_observer(y)
        z_hat = x_hat.item(0)

        # integrate error
        error = z_r - z_hat
        self.integrate_error(error)

        # Compute the state feedback controller
        F_unsat = -self.K @ (x_hat - self.xe) - self.ki*self.integrator

        F_sat = self.saturate(F_unsat.item(0)+self.Fe)

        self.integrator = self.integrator + (P.Ts / self.ki) * (F_sat - (F_unsat.item(0) + self.Fe))

        self.F_d1 = F_sat

        return F_sat, x_hat

    def update_observer(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f(self.x_hat, y_m)
        F2 = self.observer_f(self.x_hat + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f(self.x_hat + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f(self.x_hat + self.Ts * F3, y_m)
        self.x_hat += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

        return self.x_hat

    def observer_f(self, x_hat, y_m):
        # xhatdot = A*xhat + B*u + L(y-C*xhat)
        xhat_dot = self.A @ (x_hat - self.xe) \
                   + self.B * (self.F_d1-self.Fe) \
                   + self.L @ (y_m-self.C @ x_hat)

        return xhat_dot

    def integrate_error(self, error):
        self.integrator = self.integrator \
                          + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

