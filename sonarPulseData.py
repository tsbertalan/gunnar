import numpy as np
import matplotlib.pyplot as plt

data = np.array(
        [
            (float(inches), float(pulseIn))
            for
            (inches, pulseIn) in
            [
                line.strip().split()
                for line in 
                '''0 388.16
                .5 389.19
                1 388.22
                1.5 389.66
                2 387.91
                3 389.03
                4 390.56
                5 388.88
                7 387.63
                10 386.22
                12 425.84
                11 402.84
                13 454.94
                14 469.41
                15 507.94
                16 562.09
                17 576.78
                18 605.13
                19 634.38
                20 671.03
                21 700.13
                22 729.50
                23 764.31
                26 880.34
                28 945.06
                30 1010.81
                32 1073.78
                34 1135.19
                36 1203
                38 1270.59
                40 1338.41
                44 1464.81
                48 1590.84
                52 1719.59
                56 1842.75
                60 1992.63
                64 2104.50
                68 2240.47
                72 2369.28
                76 2501.56'''.split('\n')
            ]
        ]
    )
inches = data[:,0]
pulseIn = data[:,1]
fig = plt.figure()
ax = fig.add_subplot(111)

firstGood = 11
print data[firstGood]

ax.scatter(inches, pulseIn, label="data")
ax.scatter(inches[firstGood:], pulseIn[firstGood:],
           facecolors="none", edgecolors="red", s=64, marker='s',
           label="good data (starting with (%.1f, %.2f))" % (inches[firstGood], pulseIn[firstGood]))

m, b = np.polyfit(inches[firstGood:], pulseIn[firstGood:], deg=1)
print m, b
X = np.linspace(min(inches), max(inches), 100)
Y = m*X + b
ax.plot(X, Y, color="red", label="Y = %.2f X + %.2f" % (m, b))

ax.legend(loc="best")
ax.set_xlabel('true distance [inches]')
ax.set_ylabel('pulseIn() [arb]')

plt.show()