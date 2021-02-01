import numpy as np
import random
import VTOLParam as P

class VTOLDynamics:
    def __init__(self, alpha=0.0):
        # Initial state conditions
        self.state = np.array([
            [P.z0],  # initial base angle
            [P.h0],  # initial panel angle
            [P.theta0],  # initial angular velocity of base
            [P.zdot0],  # initial angular velocity of panel
            [P.hdot0],
            [P.thetadot0],
        ])

        # simulation time step
        self.Ts = P.Ts

        # mass of the center
        self.mc = P.mc

        # mass of the right motor (both motors are the same)
        self.mr = P.mr

        # inertia of the center piece
        self.Jc = P.Jc

        # distance of wingspan
        self.d = P.d

        # friction coefficient
        self.mu = P.mu

        # gravitational constant
        self.g = P.g

        # Max motor forces
        self.f_max = P.f_max

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        # saturate the input torque
        u = self.saturate(u, self.f_max)

        self.rk4_step(u)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output

        return y

    def f(self, state, u):
        # Return xdot = f(x,u)
        z = state.item(0)
        h = state.item(1)
        theta = state.item(2)
        zdot = state.item(3)
        hdot = state.item(4)
        thetadot = state.item(5)
        fl = u.item(0)
        fr = u.item(1)
        # # The equations of motion.
        # M = np.array([[self.Js, 0],
        #                [0, self.Jp]])
        # C = np.array([[tau -
        #                self.b*(thetadot-phidot)-self.k*(theta-phi)],
        #               [-self.b*(phidot-thetadot)-self.k*(phi-theta)
        #               ]])
        # tmp = np.linalg.inv(M) @ C
        zddot = (1/(self.mc + 2*self.mr)) * (-(fr + fl)*np.sin(theta) - self.mu*zdot)
        hddot = (1/(self.mc + 2*self.mr)) * ((fr + fl)*np.cos(theta) - self.mc*self.g - 2*self.mr*self.g)
        thetaddot = (1/(self.Jc + 2*self.mr*self.d**2)) * ((fr - fl)*self.d)

        # build xdot and return
        xdot = np.array([[zdot], [hdot], [thetadot],
                         [zddot], [hddot], [thetaddot]])
        return xdot

    def h(self):
        # return y = h(x)
        z = self.state.item(0)
        h = self.state.item(1)
        theta = self.state.item(2)
        y = np.array([[z], [h], [theta]])

        return y

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

    def saturate(self, u, limit):
        if u.any() > limit:
            u = limit*np.sign(u)
        return u
