import numpy as np
import sys
sys.path.append('..')
import VTOLParam as P
import control as cnt
from scipy import signal

beta = 0.05
Ts = P.Ts
f_max = P.f_max

tr_h = 1.0
zeta_h = 0.707
zeta_th = 0.707
zeta_z = 0.707
tr_th = 0.8
M = 2.0
tr_z = tr_th * M

integrator_pole_long = np.array([-1])
integrator_pole_lat = np.array([-1])

tr_h_obs = tr_h/5.0
tr_z_obs = tr_z/5.0
tr_th_obs = tr_th/5.0
zeta_obs_long = 0.707
zeta_obs_long = 0.707

wn_h = 2.2 / tr_h
wn_th = 2.2 / tr_th
wn_z = 2.2 / tr_z

Fe = (P.mc + 2.0*P.mr)*P.g

### STATE SPACE EQUATIONS   ###

A_long = np.array([[0.0, 1.0],
                   [0.0, 0.0]])
B_long = np.array([[0.0],
                   [(1/(P.mc+2*P.mr))]])
C_long = np.array([[1.0, 0.0]])

A1_long = np.array([[0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [-1.0, 0.0, 0.0]])
B1_long = np.array([[0.0],
                    [(1/(P.mc+2*P.mr))],
                    [0.0]])


A_lat = np.array([[0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 1.0],
                  [0.0, (-Fe/(P.mc+2*P.mr)), (-P.mu/(P.mc+2*P.mr)), 0.0],
                  [0.0, 0.0, 0.0, 0.0]])
B_lat = np.array([[0.0],
                  [0.0],
                  [0.0],
                  [(1/(P.Jc+2*P.mr*P.d**2))]])
C_lat = np.array([[1.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0]])

A1_lat = np.array([[0.0, 0.0, 1.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 1.0, 0.0],
                   [0.0, (-Fe/(P.mc+2*P.mr)), (-P.mu/(P.mc+2*P.mr)), 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0],
                   [-1.0, 0.0, 0.0, 0.0, 0.0]])
B1_lat = np.array([[0.0],
                   [0.0],
                   [0.0],
                   [(1/(P.Jc+2*P.mr*P.d**2))],
                   [0.0]])


## LONGITUDINAL GAINS CALCULATIONS
wn_h = 2.2/tr_h  # natural frequency for angle
des_char_poly_long = np.convolve([1, 2*zeta_h*wn_h, wn_h**2], np.poly(integrator_pole_long))
des_poles_long = np.roots(des_char_poly_long)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A_long, B_long)) != 2:
    print('The system is not controllable')

else:
    # A just turn K matrix into a numpy array
    K1_long = cnt.acker(A1_long, B1_long, des_poles_long)
    K_long = np.array([[K1_long.item(0), K1_long.item(1)]])
    ki_long = K1_long.item(2)

# observer design
wn_h_obs = 2.2/tr_h_obs
des_char_poly_obs_long = [1, 2*zeta_obs_long*wn_h_obs, wn_h_obs**2]
des_obsv_poles_long = np.roots(des_char_poly_obs_long)
if np.linalg.matrix_rank(cnt.ctrb(A_long.T, C_long.T)) != 2:
    print("The system is not controllable")
else:
    L_long = cnt.acker(A_long.T, C_long.T, des_obsv_poles_long).T

print('K_long: ', K_long)
print('ki_long: ', ki_long)
print('L_long^T: ', L_long.T)


## LATERAL GAINS CALCULATIONS
wn_th = 2.2/tr_th  # natural frequency for angle
wn_z = 2.2/tr_z  # natural frequency for position
des_char_poly_lat = np.convolve(np.convolve([1, 2*zeta_z*wn_z, wn_z**2],
                            [1, 2*zeta_th*wn_th, wn_th**2]), np.poly(integrator_pole_lat))
des_poles_lat = np.roots(des_char_poly_lat)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A1_lat, B1_lat)) != 5:
    print("The system is not controllable")
else:
    K1_lat = cnt.acker(A1_lat, B1_lat, des_poles_lat)
    K_lat = np.array([[K1_lat.item(0), K1_lat.item(1), K1_lat.item(2), K1_lat.item(3)]])
    ki_lat = K1_lat.item(4)

# observer design
wn_z_obs = 2.2/tr_z_obs
wn_th_obs = 2.2/tr_th_obs
des_char_poly_obs_lat = np.convolve([1, 2*zeta_z*wn_z_obs, wn_z_obs**2],
                                    [1, 2*zeta_th*wn_th_obs, wn_th_obs**2])
des_obsv_poles_lat = np.roots(des_char_poly_obs_lat)

if np.linalg.matrix_rank(cnt.ctrb(A_lat.T, C_lat.T)) != 4:
    print('The system is not observable')
else:
    L_lat = signal.place_poles(A_lat.T, C_lat.T, des_obsv_poles_lat).gain_matrix.T

print('K_lat: ', K_lat)
print('ki_lat: ', ki_lat)
print('L_lat^T: ', L_lat.T)

