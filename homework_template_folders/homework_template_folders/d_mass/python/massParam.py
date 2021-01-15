# Inverted Pendulum Parameter File
import numpy as np
# import control as cnt

# Physical parameters of the arm known to the controller
m = 5.0  # mass kg
k = 3.0  # spring constant Kg/s^2
b = 0.5  # damping coefficient Kg/s

# parameters for animation
length = 5.0
width = 1.0

# Initial Conditions
z0 = 0.0  # initial position of mass, m
zdot0 = 1.0  # initial velocity of mass m/s

# Simulation Parameters
t_start = 0  # Start time of simulation
t_end = 50.0  # End time of simulation
Ts = 0.5  # sample time for simulation
t_plot = 0.5  # the plotting and animation is updated at this rate

# dirty derivative parameters
# sigma =  # cutoff freq for dirty derivative
# beta =   # dirty derivative gain

# saturation limits
# F_max =   # Max force, N
