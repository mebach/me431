import numpy as np
import sys
sys.path.append('..')
import VTOLParam as P
import control as cnt

beta = 0.05
Ts = P.Ts
f_max = P.f_max

ki_h = 0.25
ki_z = -0.0002

tr_h = 1.0
zeta_h = 0.707
zeta_th = 0.707
zeta_z = 0.707
tr_th = 0.8
M = 2.0
tr_z = tr_th * M

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


## LONGITUDINAL GAINS CALCULATIONS
wn_h = 2.2/tr_h  # natural frequency for angle
des_char_poly_long = [1, 2*zeta_h*wn_h, wn_h**2]
des_poles_long = np.roots(des_char_poly_long)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A_long, B_long)) != 2:
    print('The system is not controllable')

else:
    # A just turn K matrix into a numpy array
    K_long = (cnt.acker(A_long, B_long, des_poles_long)).A
    kr_long = -1.0/(C_long @ np.linalg.inv(A_long - B_long@K_long) @ B_long)

print('K_long: ', K_long)
print('kr_long: ', kr_long)


## LATERAL GAINS CALCULATIONS
wn_th = 2.2/tr_th  # natural frequency for angle
wn_z = 2.2/tr_z  # natural frequency for position
des_char_poly_lat = np.convolve([1, 2*zeta_z*wn_z, wn_z**2],
                            [1, 2*zeta_th*wn_th, wn_th**2])
des_poles_lat = np.roots(des_char_poly_lat)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A_lat, B_lat)) != 4:
    print("The system is not controllable")
else:
    K_lat = cnt.acker(A_lat, B_lat, des_poles_lat)
    Cr = np.array([[1.0, 0.0, 0.0, 0.0]])
    kr_lat = -1.0/(Cr*np.linalg.inv(A_lat-B_lat@K_lat)@B_lat)

print('K_lat: ', K_lat)
print('kr_lat: ', kr_lat)