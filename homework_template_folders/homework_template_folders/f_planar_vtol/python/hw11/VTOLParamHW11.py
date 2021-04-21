# Inverted Pendulum Parameter File
import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P
sys.path.append('../hw7')  # add parent directory
import VTOLParamHW7 as P7
import numpy as np
# from scipy import signal
from control import TransferFunction as tf
import matplotlib.pyplot as plt
import control as cnt

# Compute plant transfer functions
th_e = 0
Plant_long = tf([1.0/(P.mc + 2*P.mr)],
           [1, 0.0, 0.0])
Plant_lat_inner = tf([(-P.g)],[1, (P.mu/(P.mc+2*P.mr)), 0])
Plant_lat_outer = tf([(1/(P.Jc + 2*P.mr*P.d**2))], [1, 0, 0])

# Compute transfer function of controller
C_pid_long = tf([(P7.kd_h+P7.kp_h*P.sigma), (P7.kp_h+P7.ki_h*P.sigma), P7.ki_h],
           [P.sigma, 1, 0])
C_pid_lat_inner = tf([(P7.kd_th+P7.kp_th*P.sigma), P7.kp_th, 0],
           [P.sigma, 1, 0])
C_pid_lat_outer = tf([(P7.kd_z+P7.kp_z*P.sigma), (P7.kp_z+P7.ki_z*P.sigma), P7.ki_z],
           [P.sigma, 1, 0])


# display bode plots of transfer functions
cnt.bode(Plant_long, dB=False, margins=False)
cnt.bode(Plant_long*C_pid_long, dB=False, margins=False)
mag_plant, phase_plant, omega_plant = cnt.bode(Plant_lat_outer, plot=False, dB=False, omega=[0.1, 20.0, 100.0])
mag_pid, phase_pid, omega_pid = cnt.bode(Plant_lat_outer*C_pid_lat_outer, plot=False, dB=True, omega=[0.01, 2.0, 100.0])

# plt.legend('No control', 'PID')
# plt.title('Mass Spring Damper')

# Closes plot windows when the user presses a button.
plt.pause(0.0001)  # not sure why this is needed for both figures to display
print('Press key to close')
print(omega_plant)
print(mag_plant)
print(mag_pid)
plt.waitforbuttonpress()
plt.close()
