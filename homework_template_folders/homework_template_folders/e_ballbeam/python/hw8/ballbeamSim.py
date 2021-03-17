import matplotlib.pyplot as plt
import sys
sys.path.append('..')
import ballbeamParam as P
from hw1.signalGenerator import signalGenerator
from hw1.ballbeamAnimation import ballbeamAnimation
from hw1.dataPlotter import dataPlotter
from hw2.ballbeamDynamics import ballbeamDynamics
from ballbeamController import ballbeamController

# instantiate ball beam, controller, and reference classes
ballbeam = ballbeamDynamics(alpha=0.0)
controller = ballbeamController()
reference = signalGenerator(amplitude=0.15, frequency=0.01, y_offset=0.25)
disturbance = signalGenerator(amplitude=0.0)
noise = signalGenerator(amplitude=0.01)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = ballbeamAnimation()

t = P.t_start  # time starts at t_start
y = ballbeam.h()  # output at the start of the simulation

while t < P.t_end:  # main simulation loop
    # Propagate dynamics at rate Ts
    t_next_plot = t + P.t_plot

    while t < t_next_plot:
        r = reference.square(t)
        d = disturbance.step(t)
        n = 0.0  # noise.random(t)  # simulate sensor noise
        x = ballbeam.state
        u = controller.update(r, x)
        y = ballbeam.update(u + d)  # propagate the dynamics
        t = t + P.Ts  # advance time by Ts

    # update animation and data plots at rate t_plot
    animation.update(ballbeam.state)
    dataPlot.update(t, r, ballbeam.state, u)

    # the pause causes figure to be displayed during simulation
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button
print('Press key to close')
plt.waitforbuttonpress()
plt.close()