from sys import argv
import numpy as np
import matplotlib.pyplot as plt
assert len(argv) > 1
fname = argv[1]
data = [
        [x.strip()
         for x in l.split(',')
         if len(x.strip()) > 0
         ]
        for l in open(fname).readlines()
        ]
if len(argv) == 3:
    ncol = int(argv[2])
else:
    ncol = max([len(l) for l in data])
trueData = []
for l in data:
    if len(l) == ncol:
        try:
            t = float(l[0])
            trueData.append(l)
        except:
            pass  # exclude lines with a bad first field (for time)
data = trueData

t = np.array([float(x) for x in [l[0] for l in data]]) / 1e6

fig = plt.figure()
ax = fig.add_subplot(111)
for i in range(1, ncol+1):
    try:
        X = np.array([float(x) for x in [l[i] for l in data]])
        ax.plot(t, X, label="column %d" % i)
    except:
        print "Failed for column %d." % i

ax.legend()

plt.show()