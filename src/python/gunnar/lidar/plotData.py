from sys import argv
if len(argv) < 2:
    print 'USAGE: %s NPZFILEPATH' % argv[0]
    from sys import exit; exit()
else:
    fpath = argv[1]
import numpy as np
import matplotlib.pyplot as plt
f, a = plt.subplots(subplot_kw=dict(polar=True))

data = np.load(fpath)['data']
fullAngles = np.arange(360.) * np.pi / 180.
for d in data:
    a.scatter(fullAngles, [dist for (dist, quality) in d])
plt.show()

