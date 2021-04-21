import control as cnt
import matplotlib.pyplot as plt

plt.figure(1), plt.clf()
P = cnt.tf([-9.81], [1, 0, 0])
cnt.bode(P)
plt.show()