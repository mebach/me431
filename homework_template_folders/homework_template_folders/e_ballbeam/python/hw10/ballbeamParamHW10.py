# Inverted Pendulum Parameter File
import numpy as np
import control as cnt
import sys
from scipy import signal
sys.path.append('..')  # add parent directory
import ballbeamParam as P

# sample rate of the controller
Ts = P.Ts
m1 = P.m1
m2 = P.m2
ze = P.z0
g = P.g
length = P.length

beta = P.beta

# saturation limits
F_max = 15.0                # Max Force, N

####################################################
#                 State Space
####################################################
# tuning parameters
tr_theta = 0.2     # Rise time for inner loop (theta)
zeta_th = 0.707       # Damping Coefficient for inner loop (theta)
M = 3.0             # Time scale separation between inner and outer loop
zeta_z = 0.707        # Damping Coefficient fop outer loop (z)
tr_z = 0.8

integrator_pole = np.array([-1])
tr_z_obs = tr_z/5.0
tr_theta_obs = tr_theta/5.0

# State Space Equations
# xdot = A*x + B*u
# y = C*x
# ze = P.length/2 * (P.m1*P.g + P.m2*P.g)
A = np.array([[0.0, 0.0, 1.0, 0.0],
               [0.0, 0.0, 0.0, 1.0],
               [0.0, -P.g, 0.0, 0.0],
               [-(P.m1*P.g)/((P.m2*P.length**2)/3 + P.m1 * ze**2), 0.0, 0.0, 0.0]])
B = np.array([[0.0],
               [0.0],
               [0.0],
               [P.length/((P.m2*P.length**2)/3 + P.m1 * ze**2)]])

C = np.array([[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0]])

# form augmented system
Cout = np.array([[1.0, 0.0, 0.0, 0.0]])

A1 = np.array([[0.0, 0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 0.0, 1.0, 0.0],
               [0.0, -P.g, 0.0, 0.0, 0.0],
               [-(P.m1*P.g)/((P.m2*P.length**2)/3 + P.m1 * ze**2), 0.0, 0.0, 0.0, 0.0],
               [-1.0, 0.0, 0.0, 0.0, 0.0]])
B1 = np.array([[0.0],
               [0.0],
               [0.0],
               [P.length/((P.m2*P.length**2)/3 + P.m1 * ze**2)],
               [0.0]])

# gain calculation
wn_th = 2.2/tr_theta  # natural frequency for angle
wn_z = 2.2/tr_z  # natural frequency for position
des_char_poly = np.convolve(np.convolve([1, 2*zeta_z*wn_z, wn_z**2],
                            [1, 2*zeta_th*wn_th, wn_th**2]), np.poly(integrator_pole))
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
    print("The system is not controllable")
else:
    K1 = cnt.acker(A1, B1, des_poles)
    K = np.array([[K1.item(0), K1.item(1), K1.item(2), K1.item(3)]])
    ki = K1.item(4)

# Compute observer gains
wn_z_obs = 2.2/tr_z_obs
wn_th_obs = 2.2/tr_theta_obs
des_obs_char_poly = np.convolve([1, 2*zeta_z*wn_z_obs, wn_z_obs**2],
                                [1, 2*zeta_th*wn_th_obs, wn_th_obs**2])
des_obs_poles = np.roots(des_obs_char_poly)

# Compute the gains if the system is observable
if np.linalg.matrix_rank(cnt.ctrb(A.T, C.T)) != 4:
    print('The system is not observable')
else:
    L = signal.place_poles(A.T, C.T, des_obs_poles).gain_matrix.T

print('K: ', K)
print('ki: ', ki)
print('L^T: ', L.T)




