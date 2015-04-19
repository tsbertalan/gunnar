import numpy as np
import kevrekidis as kv
from sys import argv
assert len(argv) > 1

# Load data:
fname = argv[1].strip()
data = [
        l.strip() for l in open(fname).readlines() if len(l.strip()) > 0
        ]

# Detect all direction changes:
MOTORRELEASE = 0
MOTORFORWARD = 1
MOTORBACKWARD = 2
dirChanges = []
for l in data:
    l = l.strip()
    if "changing direction" in l:
        print "parsing line:"
        print l.split()
        print
        print
        t = float(l.split()[0]) / 1e6
        which = int(l.split()[4].replace('m',''))
        oldDirection = int(l.split()[6])
        newDirection = int(l.split()[8])
        dirChanges.append((t, which, oldDirection, newDirection))
dirChanges = np.array(dirChanges)
# How many types of direction change do we have?
dirChangeTypes = set([tuple(x) for x in dirChanges[:,2:]])
colors = kv.plotting.colors(numColors=len(dirChangeTypes))
colors = {t: c for t, c in zip(dirChangeTypes, colors)}

numCols = 8
data = [
        [float(x) for x in l.split(',')
         if len(x.strip()) > 0
        ]
        for l in data
        if len(l.split(',')) == numCols
       ]
data = np.vstack([
    r for r in data
    if len(r) == numCols])
t = data[:,0] / 1e6
rightIndices = data[:,1].astype(bool)
setPoint = data[:,2]
monitored = data[:,3]
controlled = data[:,4]
position = data[:,5]
speed = data[:,6]
updateDelay = data[:,7]

f, A = kv.fa(numAxes=2, figsize=(20,11))
a = A[0]
b = A[1]

for who, ind in enumerate([rightIndices, np.logical_not(rightIndices)]):
    label = ["right", "left"][who]
    ax = [b, a][who]
    ax.plot(t[ind], monitored[ind], color="red", label="monitored")
    ax.plot(t[ind], controlled[ind], color="blue", label="controlled")
    ax.plot(t[ind], setPoint[ind], color="green", label="set point")
    ax.plot(t[ind], speed[ind]*4, color="magenta", label="speed")
    ax.legend(loc="best")
    ax.set_title(label)
    ax.set_xlabel("time [sec]")
    if ax is a:
        ax.set_ylabel("various arbitrary units")

## Indicate direction change commands:
#rightChangeIndices = dirChanges[:,1].astype(bool)
#for who, ind in enumerate([
                            #rightChangeIndices, np.logical_not(rightChangeIndices)
                           #]):
    #ax = [b, a][who]
    #for t, which, oldDirection, newDirection in dirChanges[ind,:]:
        #ax.axvline(t, color=colors[(oldDirection, newDirection)], lw=3)
    #ax.axhline(0, color="black")
        

kv.plotting.saveFig(f, "controls", dir="docs/", exts=("pdf",))

#kv.plotting.show()