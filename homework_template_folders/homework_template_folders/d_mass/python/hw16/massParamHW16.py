# Inverted Pendulum Parameter File
import sys
sys.path.append('..')  # add parent directory
import massParam as P
sys.path.append('../hw7')  # add parent directory
sys.path.append('../hw15')  # add parent directory
import massParamHW7 as P10
import massParamHW15 as P15
from control import tf, bode
import matplotlib.pyplot as plt

# Assign plan from previous homework solution
Plant = P15.Plant

# we can get the plant dynamics from HW 15
# Compute transfer function of controller
C_pid = tf([(P10.kd+P10.kp*P.sigma), (P10.kp+P10.ki*P.sigma), P10.ki],
           [P.sigma, 1, 0])


if __name__=="__main__":

    # display bode plots of transfer functions
    fig = plt.figure()
    bode([Plant, Plant*C_pid], dB=False)
    plt.legend(['P(s)', 'C(s)P(s)'])
    fig.axes[0].set_title('HW 16 - Freq. Response Metrics for Mass Spring Damper')

    # the order here matters and is assumed to be from smallest to largest because
    # the function re-orders the omega values to be smallest to largest regardless
    # of the order in which they are passed in.
    omegas = [0.1, 100.0]  # omega_d, omega_no
    mag_CP, phase, omegas = bode(Plant*C_pid, plot=False, dB=False, omega = omegas)
    mag_P, phase, omegas = bode(Plant, plot=False, dB=False, omega = omegas)


    # For part a), the transfer function C*P is shown in figure, this is a
    # shortcut to rendering latex from python and could be done in other ways.
    # We can use this result to find e_ss to any input.
    plt.figure()
    xfer_func = (Plant*C_pid)._repr_latex_()
    plt.text(0.1, 0.5,'$%s$'%xfer_func[2:-2], fontsize='xx-large')
    plt.tick_params(axis='both', which='both', bottom=False, top=False,
                    left=False, right=False, labelbottom=False, labelleft=False)

    # finding the necessary values for parts b) and c)
    print("values for metric calculation for parts b) and c):")
    print("gamma_d = ", mag_P[0]/mag_CP[0])
    print("gamma_n = ", mag_CP[1])

    print('Close window to end program')
    plt.show()
