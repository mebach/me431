# Single link arm Parameter File
import sys
sys.path.append('..')  # add parent directory
import massParam as P
from control import tf, bode
import matplotlib.pyplot as plt


# Compute plant transfer functions
Plant = tf([1.0/P.m],
           [1, P.b/P.m, P.k/P.m])


if __name__=="__main__":

    # Bode plot of the plant
    fig = plt.figure()
    bode(Plant, dB=False)
    fig.axes[0].set_title('HW 15 - P(s) for mass-spring-damper')

    print('Close window to end program')
    plt.show()
