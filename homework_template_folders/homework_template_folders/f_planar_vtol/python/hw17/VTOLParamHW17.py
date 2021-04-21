# VTOL Parameter File
import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P
sys.path.append('../hw16')  # add parent directory
import VTOLParamHW16 as P16
from control import tf, bode, margin
import matplotlib.pyplot as plt

# flag to define if using dB or absolute scale for M(omega)
dB_flag = False

# Compute open-loop transfer functions
P_lon = P16.P_lon
P_lat_in = P16.P_lat_in
P_lat_out = P16.P_lat_out

# Compute the controller transfer functions from HW10
C_lon = P16.C_lon
C_lat_in = P16.C_lat_in
C_lat_out = P16.C_lat_out

if __name__=="__main__":

    # Plot the closed loop and open loop bode plots for the longitudinal loop
    fig1 = plt.figure()
    bode([P_lon*C_lon, P_lon*C_lon/(1+P_lon*C_lon)], dB=dB_flag)
    plt.legend(['$C_{lon}(s)P_{lon}(s)$', 'Closed-loop Longitudinal PID'])
    fig1.axes[0].set_title('Longitudinal Frequency Response')


    # Plot the closed loop and open loop bode plots for the lateral inner loop
    fig2 = plt.figure()
    bode([P_lat_in*C_lat_in, P_lat_in*C_lat_in/(1+P_lat_in*C_lat_in)], dB=dB_flag)
    plt.legend(['$C_{lat,in}(s)P_{lat,in}(s)$', 'Inner Loop Lateral PID'])
    fig2.axes[0].set_title('Lateral Inner Loop Frequency Response')

    # Plot the closed loop and open loop bode plots for the lateral outer loop
    fig3 = plt.figure()
    bode([P_lat_out*C_lat_out, P_lat_out*C_lat_out/(1+P_lat_out*C_lat_out)], dB=dB_flag)
    plt.legend(['$C_{lat,out}(s)P_{lat,out}(s)$', 'Outer Loop Lateral PID'])
    fig3.axes[0].set_title('Lateral Outer Loop Frequency Response')

    # Calculate the phase and gain margin
    gm, pm, Wcg, Wcp = margin(P_lon*C_lon)
    print("Longitudinoal Loop: gm: ",gm," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)

    gm, pm, Wcg, Wcp = margin(P_lat_in*C_lat_in)
    print("Lateral Inner Loop: gm: ",gm," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)

    gm, pm, Wcg, Wcp = margin(P_lat_out*C_lat_out)
    print("Lateral Outer Loop: gm: ",gm," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)

    plt.show()