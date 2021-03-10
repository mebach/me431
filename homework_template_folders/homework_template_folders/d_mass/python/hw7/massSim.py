import matplotlib.pyplot as plt
import sys
sys.path.append('..')
import massParam as P
from hw1.signalGenerator import signalGenerator
from hw1.massAnimation import massAnimation
from hw1.dataPlotter import dataPlotter
from hw2.massDynamics import massDynamics
from massController import massController


# instantiate mass, controller, and reference classes
mass = massDynamics()
controller = massController()
reference = signalGenerator(amplitude=1.0, frequency=0.02)
disturbance = signalGenerator(amplitude=0.0)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = massAnimation()

t = P.t_start  # time starts at t_start
y = mass.h()

while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot:

        # get referenced inputs from signal generators
        r = reference.step(t)
        d = disturbance.step(t)
        n = 0.0  # noise.random(t)
        x = mass.state
        u = controller.update(r, y + n)  # update controller
        y = mass.update(u + d)  # propagate the dynamics
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots
    animation.update(mass.state)
    dataPlot.update(t, r, mass.state, u)

    # the pause causes the figure to be displayed during the simulation
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button
print('Press key to close')
plt.waitforbuttonpress()
plt.close()

