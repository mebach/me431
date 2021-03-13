import numpy as np
import sys
sys.path.append('..')
import ballbeamParam as P


class ballbeamDynamics:
    def __init__(self, alpha=0.2):
        # Initial state conditions
        self.state = np.array([
            [P.z0],  # z initial position
            [P.theta0],  # Theta initial orientation
            [P.zdot0],  # zdot initial velocity
            [P.thetadot0],  # Thetadot initial velocity
        ])

        # simulation time step
        self.Ts = P.Ts

        # Mass of the pendulum, kg
        self.m1 = P.m1 * (1. + alpha * (2. * np.random.rand() - 1.))

        # Mass of the cart, kg
        self.m2 = P.m2 * (1. + alpha * (2. * np.random.rand() - 1.))

        # Length of the rod, m
        self.length = P.length * (1. + alpha * (2. * np.random.rand() - 1.))

        # gravity constant is well known, don't change.
        self.g = P.g
        self.force_limit = P.F_max

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        # saturate the input force
        u = self.saturate(u, self.force_limit)

        self.rk4_step(u)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output

        return y

    def f(self, state, u):
        # Return xdot = f(x,u)
        z = state.item(0)
        theta = state.item(1)
        zdot = state.item(2)
        thetadot = state.item(3)
        F = u

        # # The equations of motion.
        # M = np.array([[self.m1 + self.m2,
        #                self.m1 * (self.ell / 2.0) * np.cos(theta)],
        #               [self.m1 * (self.ell / 2.0) * np.cos(theta),
        #                self.m1 * (self.ell ** 2 / 3.0)]])
        # C = np.array([[self.m1 * (self.ell / 2.0)
        #                * thetadot ** 2 * np.sin(theta)
        #                + F - self.b * zdot],
        #               [self.m1 * self.g * (self.ell / 2.0)
        #                * np.sin(theta)]])
        # tmp = np.linalg.inv(M) @ C
        zddot = (1/self.m1) * (self.m1*z*thetadot**2 - self.m1*self.g*np.sin(theta))
        thetaddot = (1/(self.m2*self.length**2/3.0 + self.m1*z**2)) * (F*self.length*np.cos(theta) - 2.0*self.m1*z*zdot*thetadot - self.m1*z*self.g*np.cos(theta) - self.m2*self.g*self.length/2.0*np.cos(theta))

        # build xdot and return
        xdot = np.array([[zdot], [thetadot], [zddot], [thetaddot]])

        return xdot

    def h(self):
        # return y = h(x)
        z = self.state.item(0)
        theta = self.state.item(1)
        y = np.array([[z], [theta]])

        return y

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit * np.sign(u)

        return u
