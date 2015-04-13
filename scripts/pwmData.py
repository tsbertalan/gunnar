from sys import argv
import numpy as np
import matplotlib.pyplot as plt
if len(argv) > 1:
    fname = argv[1].strip()
else:
    fname = 'pwmData.dat'
data = open(fname).readlines()
data = [[float(x) for x in l.split(',') if len(x.strip()) > 0] for l in data if len(l.strip()) > 0]
data = np.array(data)
pwm = data[:,0]
left = data[:,1]
right = data[:,2]
fig = plt.figure()
ax1 = fig.add_subplot(2,1,1)
ax2 = fig.add_subplot(2,1,2)
ax1.scatter(pwm, abs(left), label="left", color="blue", s=.1)
ax1.scatter(pwm, abs(right), label="right", color="red", s=.1)
ax1.set_ylim(0, 96)
ax1.set_xlim(min(pwm), max(pwm))
ax1.set_xlabel("pwm command (negative means reverse drive)")
encLabel = "optical encoder timed speed"
ax1.set_ylabel(encLabel)
ax1.legend(loc="center")

stepSize = 100.0 / 1000.0
T = np.arange(pwm.size) * stepSize / 60.0
ax2.plot(T, pwm, label="pwm", color="green")
ax2.plot(T, left, label="left", color="blue")
ax2.plot(T, right, label="right", color="red")

ax2.set_xlim(min(T), max(T))
ax2.legend(loc="best")

ax2.set_xlabel('time [min]')
ax2.set_ylabel(encLabel)

fig.suptitle(fname)

plt.tight_layout()

plt.show()
