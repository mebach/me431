# Single link arm Parameter File
import sys
sys.path.append('..')  # add parent directory
import massParam as P
import numpy as np
import control as cnt
from control import TransferFunction as tf
import matplotlib.pyplot as plt
# from mpldatacursor import datacursor


# Compute plant transfer functions
th_e = 0
Plant = tf([1.0/P.m],
           [1, P.b/P.m, P.k/P.m])


# Bode plot of the plant
plt.figure(3), cnt.bode_plot(Plant, dB=False, margins=False)

# if you want specific values at specific frequencies, you can do the following:
mag, phase, omega = cnt.bode(Plant, plot=False, dB=False, omega = [0.3, 10.0, 100.0])

# Closes plot windows when the user presses a button.
plt.pause(0.0001)  # not sure why this is needed for both figures to display
print('Close window to end program')
plt.show()
