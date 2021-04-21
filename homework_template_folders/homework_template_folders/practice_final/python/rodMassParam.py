import numpy as np
import control as cnt
import matplotlib.pyplot as plt

# physical parameters
m = 0.1
ell = 0.25
b = 0.1
g = 9.8
k1 = 0.02
k2 = 0.01
tau_max = 3

# initial conditions
theta_0 = 0
thetadot_0 = 0
theta_eq = 0

# equilibrium force
tau_eq = (k1*theta_eq) + (k2*theta_eq**3) + (m*g*ell*np.cos(theta_eq))

# Controller parameters
sigma = 0.005
tr = 0.059327
tr_obs = tr/10.0
zeta = 0.707
zeta_obs = 0.707
kp = tau_max / (20*np.pi/180)
kd = 0.3277
ki = 5.0
integrator_pole = [-10]

# Statespace equations
A = np.array([[0.0, 1.0],
              [(-k1/(m*ell**2)), (-b/(m*ell**2))]])
B = np.array([[0.0],
              [(1/(m*ell**2))]])
C = np.array([[1.0, 0.0]])

# augmented statespace system for use with integrator full state feedback
A1 = np.array([[0.0, 1.0, 0.0],
               [(-k1/(m*ell**2)), (-b/(m*ell**2)), 0.0],
               [-1.0, 0.0, 0.0]])
B1 = np.array([[0.0],
               [(1/(m*ell**2))],
               [0.0]])

# gain calculation
wn = 2.2/tr # or 0.5*np.pi/(tr*np.sqrt(1-zeta**2))
des_char_poly = [1, 2*zeta*wn, wn**2]
des_poles = np.roots(des_char_poly)
polesI = np.append(des_poles, -10)
if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3:
    print("The system is not controllable.")
else:
    K1 = cnt.place(A1, B1, polesI)
    K = np.array([K1.item(0), K1.item(1)])
    ki2 = K1.item(2)

# OBSERVER DESIGN
# wn_obs = 0.5*np.pi/(tr_obs*np.sqrt(1-zeta_obs**2))
# des_obsv_char_poly = [1, 2*zeta_obs*wn_obs, wn_obs**2]
obsv_poles = 5 * des_poles

# compute the gains if the system is observable
if np.linalg.matrix_rank(cnt.ctrb(A.T, C.T)) != 2:
    print('The system is not observable.')
else:
    L = cnt.acker(A.T, C.T, obsv_poles).T

print('K: ', K)
print('ki2: ', ki2)
print('L^T: ', L.T)

## LOOP SHAPING CONTROL ###
dB_flag = False

Plant = cnt.tf([1.0 / (m * ell**2)], [1, b/(m*ell**2), k1/(m*ell**2)])
C_pid = cnt.tf([(kd+kp*sigma), (kp+ki*sigma), ki], [sigma, 1, 0])


# fig1 = plt.figure()
# cnt.bode([Plant, Plant*C_pid, Plant*C_pid/(1+Plant*C_pid)], dB=False)
# plt.legend(('No Control - $P(s)$', '$C_{pid}(s)P(s)$', 'Closed-Loop PID'))
# fig1.axes[0].set_title('Rod Mass')
#
# fig2 = plt.figure()
# cnt.bode([Plant, Plant*C_pid], dB=dB_flag, margins=True)
# fig2.axes[0].set_title('Rod Mass - Stability Margins')
#
# mag, phase, omega = cnt.bode([Plant*C_pid], plot=False, omega=[0.001, 100.0])
# print(1/mag[0])  # gives the tracking accuracy at 0.001 rad/s
# print(mag[1])  # gives the noise attenuation at 100 rad/s


# plt.show()


# simulation parameters
t_start = 0.0
t_end = 100.0
Ts = 0.01
t_plot = 0.1


