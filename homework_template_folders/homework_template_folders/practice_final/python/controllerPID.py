import numpy as np
import rodMassParam as P

class controllerPID:

    def __init__(self):
        self.kp = P.kp
        self.ki = P.ki
        self.kd = P.kd
        self.limit = P.tau_max
        self.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)
        self.Ts = P.Ts
        self.theta_d1 = 0
        self.theta_dot = 0
        self.error_dot = 0
        self.error_d1 = 0
        self.integrator = 0
        self.tau_eq = P.tau_eq

    def update(self, theta_r, y):
        theta = y.item(0)
        error = theta_r - theta

        # derivative control
        self.error_dot = self.beta * self.error_dot + (1 + self.beta) * ((error - self.error_d1) / self.Ts)
        self.error_d1 = error
        self.theta_dot = self.beta * self.theta_dot + (1 + self.beta) * ((theta - self.theta_d1) / self.Ts)
        self.theta_d1 = theta

        # integral control
        threshold = 1000.0
        if np.abs(self.error_dot) <= threshold:
            self.integrator = self.integrator + (self.Ts / 2.0) * (error + self.error_d1)
        tau_unsat = self.kp * error - self.kd * self.theta_dot + self.ki * self.integrator
        tau = self.saturate(tau_unsat + P.tau_eq*np.cos(theta))

        return tau

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u







