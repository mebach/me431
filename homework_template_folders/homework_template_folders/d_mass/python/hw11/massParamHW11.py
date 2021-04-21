# Inverted Pendulum Parameter File
import sys
sys.path.append('..')  # add parent directory
import massParam as P
sys.path.append('../hw7')  # add parent directory
import massParamHW7 as P7
import numpy as np
# from scipy import signal
from control import TransferFunction as tf
import matplotlib.pyplot as plt
import control as cnt

# Compute plant transfer functions
th_e = 0
Plant = tf([1.0/P.m],
           [1, P.b/P.m, P.k/P.m])

# Compute transfer function of controller
C_pid = tf([(P7.kd+P7.kp*P.sigma), (P7.kp+P7.ki*P.sigma), P7.ki],
           [P.sigma, 1, 0])
ramp = tf([1], [1, 0])


# display bode plots of transfer functions
cnt.bode(Plant, dB=False, margins=False, omega_limits=[0.001, 1000])
cnt.bode(Plant*C_pid, dB=False, margins=False, omega_limits=[0.001, 1000])
cnt.bode(ramp, dB=False, margins=False, omega_limits=[0.001, 1000])

mag_plant, phase_plant, omega_plant = cnt.bode(Plant, plot=False, dB=False, omega=[0.1, 10.0, 100.0])
mag_pid, phase_pid, omega_pid = cnt.bode(Plant*C_pid, plot=False, dB=False, omega=[0.001, 10.0, 100.0])
mag_s, phase_s, omega_s = cnt.bode(ramp, plot=False, dB=False, omega=[0.001, 10.0, 100.0])

# plt.legend('No control', 'PID')
# plt.title('Mass Spring Damper')

# Closes plot windows when the user presses a button.
plt.pause(0.0001)  # not sure why this is needed for both figures to display
print('Press key to close')
print(omega_plant)
print(mag_plant)
print(mag_pid)
print(mag_s)
# plt.waitforbuttonpress()
# plt.close()
plt.show()
