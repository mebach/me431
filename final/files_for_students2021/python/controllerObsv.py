import numpy as np
import massParam as P

class controllerObsv:
    def __init__(self):
        self.observer_state = np.array([])
        self.x_hat = np.array([[0.0],
                               [0.0]])
        self.F_d1 = 0.0
        self.integrator = 0.0
        self.error_d1 = 0.0
        self.K = P.K
        self.ki = P.ki2  # grab the second one, not the one found from PID control
        self.L = P.L
        self.A = P.A
        self.B = P.B
        self.C = P.C
        self.limit = P.F_max
        self.F_e = P.F_e
        self.Ts = P.Ts

    def update(self, z_r, y):
        x_hat = self.update_observer(y)
        z_hat = self.x_hat.item(0)

        # integrate error
        error = z_r - z_hat
        self.integrateError(error)

        # compute the state feedback controller
        F_tilde = -self.K @ x_hat - self.ki*self.integrator

        F_e = P.k1 * z_hat + P.k2 * z_hat ** 3 - (1 / np.sqrt(2)) * P.m * P.g
        F = self.saturate(F_tilde.item(0) + F_e)

        self.F_d1 = F
        # self.integratorAntiWindup(F, F_tilde+F_e)

        return F, x_hat

    def update_observer(self, y):
        # update the observer using RK4 integration
        F1 = self.observer_f(self.x_hat, y)
        F2 = self.observer_f(self.x_hat + self.Ts / 2 * F1, y)
        F3 = self.observer_f(self.x_hat + self.Ts / 2 * F2, y)
        F4 = self.observer_f(self.x_hat + self.Ts * F3, y)
        self.x_hat += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        x_hat = np.array([[self.x_hat.item(0)],
                          [self.x_hat.item(1)]])
        return x_hat

    def observer_f(self, x_hat, y_m):
        xhat_dot = self.A @ x_hat + self.B * self.F_d1 + self.L @ (y_m - self.C @ x_hat)
        return xhat_dot

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts / 2.0) * (error + self.error_d1)
        self.error_d1 = error

    def integratorAntiWindup(self, F, F_unsat):
        if self.ki != 0.0:
            self.integrator = self.integrator + self.Ts/self.ki * (F-F_unsat)

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

