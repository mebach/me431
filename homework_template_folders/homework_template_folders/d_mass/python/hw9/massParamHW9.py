import numpy as np
import control as cnt
import sys
sys.path.append('..')
import massParam as P

Ts = P.Ts
beta = P.beta
F_max = P.F_max
m = P.m
k = P.k
b = P.b
g = P.g

# tuning parameters
tr = 1.0
zeta = 0.707
integrator_pole = np.array([-1])

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.array([[0.0, 1.0],
              [(-k/m), (-b/m)]])
B = np.array([[0.0],
              [(1/m)]])
C = np.array([[1.0, 0.0]])

# form augmented system
A1 = np.array([[0.0, 1.0, 0.0],
              [(-k/m), (-b/m), 0.0],
              [-1.0, 0.0, 0.0]])
B1 = np.array([[0.0],
               [(1/m)],
               [0.0]])

# gain calculation
wn = 2.2/tr
des_char_poly = np.convolve([1, 2*zeta*wn, wn**2], np.poly(integrator_pole))
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3:
    print('The system is not controllable')

else:
    # A just turn K matrix into a numpy array
    K1 = cnt.acker(A1, B1, des_poles)
    K = np.array([[K1.item(0), K1.item(1)]])
    ki = K1.item(2)

print('K: ', K)
print('ki: ', ki)
