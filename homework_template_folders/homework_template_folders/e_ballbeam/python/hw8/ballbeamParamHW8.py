# Inverted Pendulum Parameter File
import numpy as np
import control as cnt
import sys
sys.path.append('..')  # add parent directory
import ballbeamParam as P

# sample rate of the controller
Ts = P.Ts

beta = P.beta

# saturation limits
F_max = 15.0                # Max Force, N

####################################################
#                 State Space
####################################################
# tuning parameters
tr_theta = 0.3     # Rise time for inner loop (theta)
zeta_th = 0.707       # Damping Coefficient for inner loop (theta)
M = 3.0             # Time scale separation between inner and outer loop
zeta_z = 0.707        # Damping Coefficient fop outer loop (z)
tr_z = 1.0

# State Space Equations
# xdot = A*x + B*u
# y = C*x
ze = P.length/2 * (P.m1*P.g + P.m2*P.g)
A = np.array([[0.0, 0.0, 1.0, 0.0],
               [0.0, 0.0, 0.0, 1.0],
               [0.0, -P.g,0.0, 0.0],
               [-(P.m1*P.g)/((P.m2*P.length**2)/3 + P.m1 * ze**2), 0.0, 0.0, 0.0]])
B = np.array([[0.0],
               [0.0],
               [0.0],
               [P.length/((P.m2*P.length**2)/3 + P.m1 * ze**2)]])

C = np.array([[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0]])

# gain calculation
wn_th = 2.2/tr_theta  # natural frequency for angle
wn_z = 2.2/tr_z  # natural frequency for position
des_char_poly = np.convolve([1, 2*zeta_z*wn_z, wn_z**2],
                            [1, 2*zeta_th*wn_th, wn_th**2])
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A, B)) != 4:
    print("The system is not controllable")
else:
    K = cnt.acker(A, B, des_poles)
    Cr = np.array([[1.0, 0.0, 0.0, 0.0]])
    ki = -1.0/(Cr*np.linalg.inv(A-B@K)@B)

print('K: ', K)
print('kr: ', kr)




