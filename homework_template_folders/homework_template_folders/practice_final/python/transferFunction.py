import numpy as np
import matplotlib.pyplot as plt

class transferFunction:
    def __init__(self, num, den, Ts):
        # expects num and den to be numpy arrays of
        # shape (1,m+1) and (1,n+1)
        m = num.shape[1]
        n = den.shape[1]
        # set initial conditions
        self.state = np.zeros((n-1, 1))
        self.Ts = Ts
        # make the leading coef of den == 1
        if den.item(0) != 1:
            tmp = den.item(0)
            num = num / tmp
            den = den / tmp
        self.num = num
        self.den = den
        # set up state space equations in control canonic form
        self.A = np.zeros((n-1, n-1))
        self.B = np.zeros((n-1, 1))
        self.C = np.zeros((1, n-1))
        for i in range(0, n-1):
            self.A[0][i] = - den.item(i + 1)
        for i in range(1, n-1):
            self.A[i][i - 1] = 1.0
        if n>1:
            self.B[0][0] = 1.0
        if m == n:
            self.D = num.item(0)
            for i in range(0, n-1):
                self.C[0][i] = num.item(i+1) \
                               - num.item(0)*den.item(i+1)
        else:
            self.D = 0.0
            for i in range(n-m-1, n-1):
                self.C[0][i] = num.item(i)

    def update(self, u):
        x = self.rk4(u)
        y = self.C @ x + self.D * u
        return y.item(0)

    def f(self, state, u):
        xdot = self.A @ state + self.B * u
        return xdot

    def rk4(self, u):
        # Integrate ODE using Runge-Kutta 4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        return self.state

if __name__ == "__main__":
    # instantiate the system
    Ts = 0.01  # simulation step size
    # system = (s + 2)/(s^3 + 4s^2 + 5s + 6)
    #num = np.array([[1, 2]])
    num = np.array([[7, 8, 9, 10]])
    den = np.array([[1, 4, 5, 6]])
    system = transferFunction(num, den, Ts)

    # main simulation loop
    sim_time = 0.0
    time = [sim_time]  # record time for plotting
    y = system.h(0.0)
    output = [y]  # record output for plotting
    while sim_time < 10.0:
        u = np.random.randn()  # input is white noise
        y = system.update(u)  # update based on current input
        time.append(sim_time)  # record time for plotting
        output.append(y)  # record output for plotting
        sim_time += Ts   # increment the simulation time
    # plot output vs time
    plt.plot(time, output)
    plt.show()

