# Single link arm Parameter File
import numpy as np
# import control as cnt
import sys
sys.path.append('..')  # add parent directory
import massParam as P

Ts = P.Ts  # sample rate of the controller
# beta = P.beta  # dirty derivative gain
F_max = P.F_max  # limit on control signal

# PD gains
kp = 6
kd = 8.985

print('kp: ', kp)
print('kd: ', kd)



