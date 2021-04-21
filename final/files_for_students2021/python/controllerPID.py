import numpy as np
import massParam as P

class controllerPID:

    def __init__(self):
        self.kp = P.kp
        self.ki = P.ki
        self.kd = P.kd
        self.limit = P.F_max
        self.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)
        self.Ts = P.Ts
        self.z_d1 = 0.0
        self.z_dot = 0.0
        self.error_d1 = 0.0
        self.error_dot = 0.0
        self.integrator = 0.0
        self.F_e = P.F_e

    def update(self, z_r, y):
        z = y.item(0)
        error = z_r - z

        # derivative control
        self.error_dot = self.beta * self.error_dot + (1 + self.beta) * ((error - self.error_d1) / self.Ts)
        self.error_d1 = error
        self.z_dot = self.beta * self.z_dot + (1 + self.beta) * ((z - self.z_d1) / self.Ts)
        self.z_d1 = z

        # integral control
        threshold = 0.1
        if np.abs(self.error_dot) <= threshold:
            self.integrator = self.integrator + (self.Ts / 2.0) * (error + self.error_d1)
        self.error_d1 = error

        # calculate the control force
        F_unsat = self.kp * error - self.kd * self.z_dot + self.ki * self.integrator

        F_e = P.k1 * z + P.k2 * z ** 3 - (1 / np.sqrt(2)) * P.m * P.g

        F = self.saturate(F_unsat + F_e)

        return F

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

    def integratorAntiWindup(self, F, F_unsat):
        if self.ki != 0.0:
            self.integrator = self.integrator + self.Ts/self.ki * (F - F_unsat)






