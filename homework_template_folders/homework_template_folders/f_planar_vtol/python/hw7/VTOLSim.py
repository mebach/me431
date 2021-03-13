import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('..')
import VTOLParam as P
from hw1.signalGenerator import signalGenerator
from hw1.VTOLAnimation import VTOLAnimation
from hw1.dataPlotter import dataPlotter
from hw2.VTOLDynamics import VTOLDynamics
from VTOLController import VTOLController


# instantiate VTOL, controller, and reference classes
VTOL = VTOLDynamics()
controller = VTOLController()
z_reference = signalGenerator(amplitude=2.5, frequency=0.0025, y_offset=3.0)
h_reference = signalGenerator(amplitude=3.0, frequency=0.0025, y_offset=5.0)
f_l = signalGenerator(amplitude=50.0, frequency=0.5)
f_r = signalGenerator(amplitude=50.0, frequency=0.5)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = VTOLAnimation()

t = P.t_start  # times starts at t_start
y = VTOL.h()

while t < P.t_end:

    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_next_plot:
        h_ref = h_reference.square(t)
        z_ref = z_reference.square(t)
        r = np.array([[z_ref], [h_ref]])
        d = np.array([[0.0], [0.0]])
        n = 0.0  #noise.random(t)
        x = VTOL.state
        u = controller.update(r, y)
        y = VTOL.update(u)
        t = t + P.Ts

    animation.update(VTOL.state, 1.0)
    dataPlot.update(t, VTOL.state, z_ref, h_ref, u.item(0), u.item(1))

    # the pause causes the figure to be displayed during the simulation
    plt.pause(0.0001)

# keeps the program from closing until the user presses a button
print('Press key to close')
plt.waitforbuttonpress()
plt.close()