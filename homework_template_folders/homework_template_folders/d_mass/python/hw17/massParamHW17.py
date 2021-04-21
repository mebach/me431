# Inverted Pendulum Parameter File
import sys
sys.path.append('..')  # add parent directory
sys.path.append('../hw16')  # add parent directory
import massParamHW16 as P16
from control import bode, margin, mag2db
import matplotlib.pyplot as plt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = False

# assigning plant and controller from past HW (to make sure we don't introduce additional errors)
Plant = P16.Plant
C_pid = P16.C_pid

if __name__ == "__main__":

    # display bode plots of transfer functions
    fig1 = plt.figure()
    bode([Plant, Plant*C_pid, Plant*C_pid/(1+Plant*C_pid)], dB=dB_flag)
    plt.legend(('No control - $P(s)$', '$C_{pid}(s)P(s)$', 'Closed-loop PID'))
    fig1.axes[0].set_title('Mass Spring Damper')

    fig2 = plt.figure()
    bode([Plant, Plant*C_pid], dB=dB_flag, margins=True)
    fig2.axes[0].set_title('Mass Spring Damper - Stability Margins')


    # Calculate the phase and gain margin
    gm, pm, Wcg, Wcp = margin(Plant * C_pid)

    if dB_flag:
        print("gm: ", gm, " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)
    else:
        print("gm: ", mag2db(gm), " pm: ", pm, " Wcg: ", Wcg, " Wcp: ", Wcp)

    plt.show()
