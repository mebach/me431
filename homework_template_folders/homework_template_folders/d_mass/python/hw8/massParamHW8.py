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

tr = 0.4
zeta = 0.707

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.array([[0.0, 1.0], [(-k/m), (-b/m)]])
B = np.array([[0.0], [(1/m)]])
C = np.array([[1.0, 0.0]])

# gain calculation
wn = 2.2/tr
des_char_poly = [1, 2*zeta*wn, wn**2]
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A, B)) != 2:
    print('The system is not controllable')

else:
    # A just turn K matrix into a numpy array
    K = (cnt.acker(A, B, des_poles)).A
    kr = -1.0/(C @ np.linalg.inv(A - B@K) @ B)

print('K: ', K)
print('kr: ', kr)
