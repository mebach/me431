import numpy as np
import rodMassParam as P

class controllerObsv:
    def __init__(self):
        self.observer_state = np.array([
            [0.0],  # estimate of theta
            [0.0],  # estimate of theta_hat
            [0.0],  # estimate of disturbance
        ])
        self.x_hat = np.array([[1.0],
                               [0.0]])
        self.tau_d1 = 0.0  # control torque, delayed by one sample
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0  # error signal delayed by 1 sample
        self.K = P.K  # state feedback gain
        self.ki = P.ki2  # Input gain
        self.L = P.L  # observer gain
        self.A = P.A  # system model
        self.B = P.B
        self.C = P.C
        self.limit = P.tau_max
        self.tau_eq = P.tau_eq
        self.Ts = P.Ts  # sample rate of controller

    def update(self, theta_r, y):
        x_hat = self.update_observer(y)
        theta_hat = self.x_hat.item(0)

        # integrate error
        error = theta_r - theta_hat
        self.integrateError(error)

        # compute the state feedback controller
        tau_tilde = -self.K @ x_hat - self.ki*self.integrator
        tau = self.saturate(tau_tilde.item(0))
        self.tau_d1 = tau

        return tau, x_hat

    def update_observer(self, y):
        # update the observer using RK4 integration
        F1 = self.observer_f(self.x_hat, y)
        F2 = self.observer_f(self.x_hat + self.Ts / 2 * F1, y)
        F3 = self.observer_f(self.x_hat + self.Ts / 2 * F2, y)
        F4 = self.observer_f(self.x_hat + self.Ts * F3, y)
        self.x_hat += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        x_hat = np.array([[self.x_hat.item(0)],
                          [self.x_hat.item(1)]])
        # d_hat = self.observer_state.item(2)
        return x_hat

    def observer_f(self, x_hat, y_m):
        xhat_dot = self.A @ x_hat + self.B * self.tau_d1 + self.L @ (y_m - self.C @ x_hat)
        return xhat_dot

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def integratorAntiWindup(self, tau, tau_unsat):
        if self.ki != 0.0:
            self.integrator = self.integrator + self.Ts/self.ki * (tau-tau_unsat)

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

