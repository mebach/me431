# mass Parameter File
import numpy as np
# import control as cnt
import sys
sys.path.append('..')  # add parent directory
import massParam as P

Ts = P.Ts  # sample rate of the controller
beta = P.beta  # dirty derivative gain
F_max = P.F_max  # limit on control signal

# tuning parameters
tr = 1.6395
zeta = 0.707
ki = 0.3  # integrator gain

# desired natural frequency
wn = 2.2/tr
alpha1 = 2.0*zeta*wn
alpha0 = wn**2

# compute PD gains
kp = (alpha0 - P.k/P.m)/(1/P.m)
kd = (alpha1 - P.b/P.m)/(1/P.m)

print('kp: ', kp)
print('ki: ', ki)
print('kd: ', kd)
