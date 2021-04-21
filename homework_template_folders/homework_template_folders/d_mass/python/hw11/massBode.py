import control as cnt
import matplotlib.pyplot as plt

plt.figure(1), plt.clf()
P = cnt.tf([0.2], [1, 0.1, 0.6])
PC = cnt.tf()
cnt.bode(P)
plt.show()