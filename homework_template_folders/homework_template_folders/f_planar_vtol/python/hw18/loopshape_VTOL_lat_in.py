import sys
sys.path.append('../hw16')  # add parent directory
import VTOLParamHW16 as P16
import matplotlib.pyplot as plt
from control import tf, margin, bode, tf2ss, step_response
import numpy as np
import helper_functions as hf

# flag to define if using dB or absolute scale for M(omega)
dB_flag = False

# Compute open-loop transfer functions
Plant = P16.P_lat_in

#########################################
#   Control Design
#########################################
C = tf([1], [1])

#  phase lead: increase PM (stability)
w_lead = 8.0 #location of maximum frequency bump (desired crossover)
M = 15.0  # lead ratio
C_lead = hf.get_control_lead(w_lead, M)

#  low pass filter: decrease gain at high frequency (noise)
p = 200.0
C_lpf = hf.get_control_lpf(p)

# calculate final controller
C = C*C_lead*C_lpf

##############################################
#  Convert Controller to State Space Equations
##############################################
C_ss = tf2ss(C)  # convert to state space


if __name__=="__main__":

    # calculate bode plot and gain and phase margin for original PID * plant dynamics
    mag, phase, omega = bode(Plant, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$P_{lat,in}(s)$")

    gm, pm, Wcg, Wcp = margin(Plant)
    print("for original system:")
    print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)

    #########################################
    #   Define Design Specifications
    #########################################
    ## from the book, these were pretty simple
    #### 1) inlude 60 degrees of PM
    #### 2) add a lpf

    ## plot the effect of adding the new compensator terms
    mag, phase, omega = bode(Plant * C, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$C_{lat,in}(s)P_{lat,in}(s)$")

    gm, pm, Wcg, Wcp = margin(Plant * C)
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
    # Closed loop transfer function from R to U - no prefilter
    CLOSED_R_to_U = (C / (1.0 + Plant * C))

    fig = plt.figure()
    plt.grid(True)
    mag, phase, omega = bode(CLOSED_R_to_Y, dB=dB_flag, plot=True,
                             color=[0, 0, 1], label='closed-loop $\\frac{Y}{R}$')
    fig.axes[0].set_title('Closed-Loop Bode Plot')
    fig.axes[0].legend()

    fig2 = plt.figure()
    plt.subplot(211), plt.grid(True)
    T = np.linspace(0, 2, 100)
    _, yout_no_F = step_response(CLOSED_R_to_Y, T)
    plt.plot(T, yout_no_F, 'b', label='system step response')
    plt.legend()
    plt.ylabel('Step Response')

    plt.subplot(212), plt.grid(True)
    _, Uout_no_F = step_response(CLOSED_R_to_U, T)
    plt.plot(T, Uout_no_F, color='b', label='control effort')
    plt.ylabel('Control Effort')
    plt.legend()

    fig2.axes[0].set_title('Step Response')

    plt.show()


