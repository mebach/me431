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
Plant = P16.P_lon

#########################################
#   Control Design
#########################################
# we could start with C = C_pid from previous homework, but in this case,
# that made the design harder, so we start with nothing.
C = tf([1], [1])

# #  integral control: increase steady state tracking and dist rejection
# k_I = 0.5  # frequency at which integral action ends
# Integrator = tf([1, k_I], [1, 0])
ki = 0.5
C_int = hf.get_control_integral(ki)

#  phase lead: increase PM (stability)
w_lead = 1.0  #location of maximum frequency bump (desired crossover)
M = 80.0  # lead ratio
C_lead = hf.get_control_lead(w_lead, M)

# Proportional control: change cross over frequency
C_k = hf.get_control_proportional(0.25)


#  low pass filter: decrease gain at high frequency (noise)
p = 50.0
C_lpf = hf.get_control_lpf(p)

# combine all terms for C
C = C*C_int*C_lead*C_k*C_lpf


############################################
#  Prefilter Design
############################################
# low pass filter
p = 1.0
F_lpf = hf.get_control_lpf(p)
F = F_lpf

##############################################
#  Convert Controller to State Space Equations
##############################################
C_ss = tf2ss(C)  # convert to state space
F_ss = tf2ss(F)  # convert to state space



if __name__=="__main__":

    # calculate bode plot and gain and phase margin for inner loop plant dynamics
    mag, phase, omega = bode(Plant, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$P_{lon}(s)$")

    gm, pm, Wcg, Wcp = margin(Plant)
    print("for original system:")
    print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", gm, " Wcg: ", Wcg)


    #########################################
    #   Define Design Specifications
    #########################################

    #----------- general tracking specification --------
    omega_r = 0.1  # track signals below this frequency
    gamma_r = 0.01  # tracking error below this value
    hf.add_spec_ref_tracking(gamma_r, omega_r)

    #----------- noise specification --------
    omega_n = 200.0    # attenuate noise above this frequency
    gamma_n = 10**(-4.0)    # attenuate noise by this amount
    hf.add_spec_noise(gamma_n, omega_n)

    ## plot the effect of adding the new compensator terms
    mag, phase, omega = bode(Plant * C, dB=dB_flag,
                             omega=np.logspace(-3, 5),
                             plot=True, label="$C_{final}(s)P(s)$")

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



