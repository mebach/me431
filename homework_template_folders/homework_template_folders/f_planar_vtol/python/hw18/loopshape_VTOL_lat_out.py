import sys
sys.path.append('../hw16')  # add parent directory
import VTOLParamHW16 as P16
import matplotlib.pyplot as plt
from control import tf, margin, bode, tf2ss, step_response
import numpy as np
import helper_functions as hf
import loopshape_VTOL_lat_in as L_in


# flag to define if using dB or absolute scale for M(omega)
dB_flag = False

# import outer loop dynamics from previous homework
P_lat_out = P16.P_lat_out

# construct plant as cascade of P_out and closed inner loop
Plant = P_lat_out*(P16.P_lat_in*L_in.C/(1+P16.P_lat_in*L_in.C))


#########################################
#   Control Design
#########################################
C = tf([1], [1])

#  integral control: increase steady state tracking and dist rejection
ki = 0.4  # frequency at which integral action ends
C_int = hf.get_control_integral(ki)

# Proportional control: change cross over frequency
kp = -0.03
C_k = hf.get_control_proportional(kp)

#  phase lead: increase PM (stability)
w_lead = 1.3  #location of maximum frequency bump (desired crossover)
M = 45.0
C_lead = hf.get_control_lead(w_lead, M)

# low pass filter: decrease gain at high frequency (noise)
p = 20.0
C_lpf = hf.get_control_lpf(p)

C = C * C_lpf * C_k *C_lead *C_int  #C*C_int*C_lead*C_k*C_lpf

############################################
#  Prefilter Design
############################################
# low pass filter
p = 1
F = hf.get_control_lpf(p)


##############################################
#  Convert Controller to State Space Equations
##############################################
C_ss = tf2ss(C)
F_ss = tf2ss(F)


if __name__=="__main__":

    # calculate bode plot and gain and phase margin for outer loop plant dynamics
    mag, phase, omega = bode(Plant, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$P_{lat,out}(s)$")

    gm, pm, Wcg, Wcp = margin(Plant)
    print("for original system:")
    print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)


    #########################################
    #   Define Design Specifications
    #########################################
    #----------- noise specification --------
    omega_n = 100.0    # attenuate noise above this frequency
    gamma_n = 10**(-5.0)    # attenuate noise by this amount
    hf.add_spec_noise(gamma_n, omega_n)

    # other specifications are
    ### that w_co is near 1
    ### reject constant input disturbances (slope of C*P at low frequencies)
    ### Phase margin of about 60


    ## plot the effect of adding the new compensator terms
    mag, phase, omega = bode(Plant * C, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$C_{lat,out}(s)P(s)$")

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
    # Closed loop transfer function from R to Y - with prefilter
    CLOSED_R_to_Y_with_F = (F * Plant * C / (1.0 + Plant * C))
    # Closed loop transfer function from R to U - no prefilter
    CLOSED_R_to_U = (C / (1.0 + Plant * C))
    # Closed loop transfer function from R to U - with prefilter
    CLOSED_R_to_U_with_F = (F * C / (1.0 + Plant * C))

    fig = plt.figure()
    plt.grid(True)
    mag, phase, omega = bode(CLOSED_R_to_Y, dB=dB_flag, plot=True,
                             color=[0, 0, 1], label='closed-loop $\\frac{Y}{R}$ - no pre-filter')
    mag, phase, omega = bode(CLOSED_R_to_Y_with_F, dB=dB_flag, plot=True,
                             color=[0, 1, 0], label='closed-loop $\\frac{Y}{R}$ - with pre-filter')
    fig.axes[0].set_title('Closed-Loop Bode Plot')
    fig.axes[0].legend()

    plt.figure()
    plt.subplot(211), plt.grid(True)
    T = np.linspace(0, 2, 100)
    _, yout_no_F = step_response(CLOSED_R_to_Y, T)
    _, yout_F = step_response(CLOSED_R_to_Y_with_F, T)
    plt.plot(T, yout_no_F, 'b', label='response without prefilter')
    plt.plot(T, yout_F, 'g', label='response with prefilter')
    plt.legend()
    plt.ylabel('Step Response')

    plt.subplot(212), plt.grid(True)
    _, Uout_no_F = step_response(CLOSED_R_to_U, T)
    _, Uout_F = step_response(CLOSED_R_to_U_with_F, T)
    plt.plot(T, Uout_no_F, color='b', label='control effort without prefilter')
    plt.plot(T, Uout_F, color='g', label='control effort with prefilter')
    plt.ylabel('Control Effort')
    plt.legend()

    plt.show()


