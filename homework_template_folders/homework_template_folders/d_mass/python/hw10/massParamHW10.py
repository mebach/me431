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
integrator_pole = np.array([-2])
wn_obs = 20
zeta_obs = 0.707

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.array([[0.0, 1.0],
              [(-k/m), (-b/m)]])
n = A.shape[0]
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

# observer design
des_char_poly_obs = [1, 2*zeta_obs*wn_obs, wn_obs**2]
des_obsv_poles = np.roots(des_char_poly_obs)
# compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A.T, C.T)) != 2:
    print('The system is not observable')
else:
    L = cnt.acker(A.T, C.T, des_obsv_poles).T

print('K: ', K)
print('ki: ', ki)
print('L^T: ', L.T)
