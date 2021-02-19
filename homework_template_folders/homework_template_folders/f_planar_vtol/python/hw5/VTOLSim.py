import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('..')
import VTOLParam as P
from hw1.signalGenerator import signalGenerator
from hw1.VTOLAnimation import VTOLAnimation
from hw1.dataPlotter import dataPlotter
from hw2.VTOLDynamics import VTOLDynamics


# instantiate VTOL, controller, and reference classes
VTOL = VTOLDynamics()
reference = signalGenerator(amplitude=0.5, frequency=0.1)
f_l = signalGenerator(amplitude=50.0, frequency=0.5)
f_r = signalGenerator(amplitude=50.0, frequency=0.5)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = VTOLAnimation()

t = P.t_start  # times starts at t_start
while t < P.t_end:

    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot:
        r = reference.square(t)
        u = np.array([[f_l.sin(t)], [f_r.sin(t)]])
        y = VTOL.update(u)
        t = t + P.Ts

    animation.update(VTOL.state, 1.0)
    dataPlot.update(t, VTOL.state, 0.0, 0.0, u[0], u[1])

    # the pause causes the figure to be displayed during the simulation
    plt.pause(0.0001)

# keeps the program from closing until the user presses a button
print('Press key to close')
plt.waitforbuttonpress()
plt.close()