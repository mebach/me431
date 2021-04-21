import numpy as np

# one of these two forms for importing control library functions may be helpful.
import control as cnt
#from control import tf, bode, etc.

# physical parameters
g = 9.8
theta = 45.0*np.pi/180.0   #this variable just defines the slope of the block, and may not be used explicitly
m = 0.5
k1 = 0.05
k2 = 0.02
F_max = 5.0
b = 0.1

# INITIAL CONDITIONS
z0 = 0.0
zdot0 = 0.0
z_e = 0.0

# Equilibrium Force
F_e = k1*z_e + k2*z_e**3 - (1/np.sqrt(2))*m*g

## PID CONTROL GAIN CALCULATIONS ##
sigma = 0.005
zeta = 0.707
kp = F_max / 1.0
wn = np.sqrt(k1/m + kp/m)
kd = 0.5  # m*(2*zeta*wn - (b/m))
ki = 2.0

print('kp: ', kp)
print('kd: ', kd)
print('ki: ', ki)

# ----------------------------------------------
# FULL STATE FEEDBACK AND OBSERVER DESIGN

# Statespace equations
A = np.array([[0.0, 1.0],
              [(-k1/m), (-b/m)]])
B = np.array([[0.0],
              [(1/(m))]])
C = np.array([[1.0, 0.0]])

# Augmented Statespace system for use with integrator full state feedback
A1 = np.array([[0.0, 1.0, 0.0],
               [(-k1/m), (-b/m), 0.0],
               [-1.0, 0.0, 0.0]])
B1 = np.array([[0.0],
               [(1/m)],
               [0.0]])

# gain calculation
des_poles = np.array([-5+0.1j, -5-0.1j])
polesI = np.append(des_poles, -5)
if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3:
    print('The system is not controllable.')
else:
    K1 = cnt.place(A1, B1, polesI)
    K = np.array([K1.item(0), K1.item(1)])
    ki2 = K1.item(2)

# observer calculation
obsv_poles = 10 * des_poles

# compute gains if the system is observable
if np.linalg.matrix_rank(cnt.ctrb(A.T, C.T)) != 2:
    print('The system is not observable.')
else:
    L = cnt.acker(A.T, C.T, obsv_poles).T

print('K: ', K)
print('ki2: ', ki2)
print('L^T: ', L.T)

# ----------------------------------------------


# simulation parameters
t_start = 0.0
t_end = 40.0
Ts = 0.01
t_plot = 0.1
sigma = 0.05

