import numpy as np
import sys
sys.path.append('..')
import VTOLParam as P


tr_h = 8.0
zeta_h = 0.707
zeta_th = 0.707
zeta_z = 0.707
tr_th = 0.8
M = 10.0
tr_z = tr_th * M

wn_h = 2.2 / tr_h
wn_th = 2.2 / tr_th
wn_z = 2.2 / tr_z

Fe = (P.mc + 2.0*P.mr)*P.g

# define characteristic equation values for h direction
alpha0_h = wn_h**2
alpha1_h = 2 * zeta_h * wn_h
a0_h = 0.0
a1_h = 0.0
b0_h = 1 / (P.mc + 2 * P.mr)

# Calculate gains for PD control for longitudinal control
kp_h = (alpha0_h-a0_h)/b0_h
kd_h = (alpha1_h-a1_h)/b0_h
DC_gain = 1.0

# define characteristic equation values for the theta direction
alpha0_th = wn_th**2
alpha1_th = 2 * zeta_th * wn_th
a0_th = 0.0
a1_th = 0.0
b0_th = 1 / (P.Jc + 2 * P.mr * P.d**2)

# Calculate gains for PD control for lateral direction inner loop
kp_th = (alpha0_th-a0_th)/b0_th
kd_th = (alpha1_th-a1_th)/b0_th

# define characteristic equation values for the z direction
alpha0_z = wn_z**2
alpha1_z = 2 * zeta_z * wn_z
a0_z = 0.0
a1_z = (P.mu/(P.mc + 2 * P.mr))
b0_z = - (Fe/(P.mc + 2 * P.mr))

# Calculate gains for PD control for lateral direction outer loop
kp_z = (alpha0_z-a0_z)/b0_z
kd_z = (alpha1_z-a1_z)/b0_z

print('kp_h: ', kp_h)
print('kd_h: ', kd_h)
print('kp_th: ', kp_th)
print('kd_th: ', kd_th)
print('kp_z: ', kp_z)
print('kd_z: ', kd_z)

