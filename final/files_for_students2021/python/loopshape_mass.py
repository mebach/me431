import massParam as P
import matplotlib.pyplot as plt
from control import tf, tf2ss
import control as cnt
import numpy as np

# in this file, you should will need to:
#### 1)define xfer functions for the plant, and the PID controller
#### 2)define specifications (for C_pid * P, and for the final controller C)
#### 3)make bode plots to help you in meeting specifications and responding to questions

# Compute plant transfer functions
# using new gains
kp = 5
kd = 2.9
ki = 0.1

Plant = tf([1.0/P.m], [1, P.b/P.m, P.k1/P.m])
C_pid = tf([(kd+kp*P.sigma), (kp+ki*P.sigma), ki],
           [P.sigma, 1, 0])

# Question 5.1
omegas = [0.2, 20.0]  # omega_d, omega_no
mag, phase, omegas = cnt.bode(Plant*C_pid, plot=False, dB=False, omega = omegas)
print('Mag at 0.2 rad/s: ', mag[0])
print('Mag at 20 rad/s: ', mag[1])

# display bode plots of transfer functions
# fig1 = plt.figure()
# cnt.bode([Plant, Plant*C_pid, Plant*C_pid/(1+Plant*C_pid)], dB=False)
# plt.legend(('No control - $P(s)$', '$C_{pid}(s)P(s)$', 'Closed-loop PID'))
# fig1.axes[0].set_title('Mass Spring Damper')
#
# fig2 = plt.figure()
# cnt.bode([Plant, Plant*C_pid], dB=False, margins=True)
# fig2.axes[0].set_title('Mass Spring Damper - Stability Margins')
# plt.show()

# Calculate the phase and gain margin
gm, pm, Wcg, Wcp = cnt.margin(Plant * C_pid)

dB_flag = False
if dB_flag:
    print("gm: ", gm, " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)
else:
    print("gm: ", cnt.mag2db(gm), " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

# compute final compensator C, and prefilter F
def add_control_lag(C, z, M):
    Lag = tf([1, z], [1, z/M])
    return C * Lag

def add_control_lpf(C, p):
    LPF = tf(p, [1, p])
    return C * LPF

def add_control_proportional(C, kp):
    proportional = tf([kp], [1])
    return C * proportional

def add_spec_noise(gamma_n, omega_n, dB_flag = False):
    w = np.logspace(np.log10(omega_n), 1 + np.log10(omega_n))
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].loglog(w, gamma_n * np.ones(len(w)),
                 '--', color=[0, 1, 0], label = 'noise spec')
    else:
        fig.axes[0].semilogx(w, 20.0* np.log10(gamma_n) * np.ones(len(w)),
                 '--', color=[0, 1, 0], label = 'noise spec')

def add_spec_ref_tracking(gamma_r, omega_r, dB_flag = False):
    w = np.logspace(np.log10(omega_r)-1, np.log10(omega_r))
    fig = plt.gcf()
    if dB_flag == False:
        fig.axes[0].loglog(w, 1./gamma_r*np.ones(len(w)),
                 '.', color=[1, 0, 0],label='ref tracking spec')
    else:
        fig.axes[0].semilogx(w, 20.*np.log10(1/gamma_r)*np.ones(len(w)),
                '.', color=[1, 0, 0],label='ref tracking spec')

C = C_pid
C = add_control_lag(C, z=.001, M=(1/1e-6))
C = add_control_lpf(C, p=10000)
C = add_control_proportional(C, kp=1)
F = tf(1, 1)
F = add_control_lpf(F, p=0.8)

# convert them to state space models if you are going to use that method
# otherwise you can just use C and F directly in the controllerLoop.py file.
Css=cnt.tf2ss(C)
Fss=cnt.tf2ss(F)

if __name__ == "__main__":
    # calculate bode plot and gain and phase margin for original PID * plant dynamics
    mag, phase, omega = cnt.bode(Plant * C_pid, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$C_{pid}(s)P(s)$")

    gm, pm, Wcg, Wcp = cnt.margin(Plant * C_pid)
    print("for original C_pid system:")
    print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)
    mag, phase, omega = cnt.bode(Plant, dB=dB_flag,
                                 omega=np.logspace(-3, 5),
                                 plot=True, label="Plant")

    #########################################
    #   Define Design Specifications
    #########################################

    # ----------- noise specification --------
    omega_n = 2000  # attenuate noise above this frequency
    gamma_n = 1e-5  # attenuate noise by this amount
    add_spec_noise(gamma_n, omega_n)

    # ----------- general tracking specification --------
    omega_r = 0.001  # track signals below this frequency
    gamma_r = 1e-4  # tracking error below this value
    add_spec_ref_tracking(gamma_r, omega_r)

    ## plot the effect of adding the new compensator terms
    mag, phase, omega = cnt.bode(Plant * C, dB=dB_flag,
                             omega=np.logspace(-4, 5),
                             plot=True, label="$C_{final}(s)P(s)$")

    gm, pm, Wcg, Wcp = cnt.margin(Plant * C)
    print("for final C*P:")
    print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

    fig = plt.gcf()
    fig.axes[0].legend()
    plt.show()

    ############################################
    # now check the closed-loop response with prefilter
    ############################################
    # Closed loop transfer function from R to Y - no prefilter
    CLOSED_R_to_Y = (Plant * C / (1.0 + Plant * C))
    # Closed loop transfer function from R to Y - with prefilter
    CLOSED_R_to_Y_with_F = (F * Plant * C / (1.0 + Plant * C))
    # Closed loop transfer function from R to U - no prefilter
    CLOSED_R_to_U = (C / (1.0 + Plant * C))
    # Closed loop transfer function from R to U - with prefilter
    CLOSED_R_to_U_with_F = (F * C / (1.0 + Plant * C))

    fig = plt.figure()
    plt.grid(True)
    mag, phase, omega = cnt.bode(CLOSED_R_to_Y, dB=dB_flag, plot=True,
                             color=[0, 0, 1], label='closed-loop $\\frac{Y}{R}$ - no pre-filter')
    mag, phase, omega = cnt.bode(CLOSED_R_to_Y_with_F, dB=dB_flag, plot=True,
                             color=[0, 1, 0], label='closed-loop $\\frac{Y}{R}$ - with pre-filter')
    fig.axes[0].set_title('Closed-Loop Bode Plot')
    fig.axes[0].legend()

    plt.figure()
    plt.subplot(211), plt.grid(True)
    T = np.linspace(0, 2, 100)
    _, yout_no_F = cnt.step_response(CLOSED_R_to_Y, T)
    _, yout_F = cnt.step_response(CLOSED_R_to_Y_with_F, T)
    plt.plot(T, yout_no_F, 'b', label='response without prefilter')
    plt.plot(T, yout_F, 'g', label='response with prefilter')
    plt.legend()
    plt.ylabel('Step Response')

    plt.subplot(212), plt.grid(True)
    _, Uout_no_F = cnt.step_response(CLOSED_R_to_U, T)
    _, Uout_F = cnt.step_response(CLOSED_R_to_U_with_F, T)
    plt.plot(T, Uout_no_F, color='b', label='control effort without prefilter')
    plt.plot(T, Uout_F, color='g', label='control effort with prefilter')
    plt.ylabel('Control Effort')
    plt.legend()

    plt.show()


