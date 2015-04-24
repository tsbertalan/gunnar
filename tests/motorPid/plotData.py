import numpy as np
from sys import argv
fname = argv[1]
import kevrekidis as kv
import matplotlib.pyplot as plt

ncols = 7
data = [l for l in open(fname).readlines() if len(l.strip()) > 0 and "Estimated" not in l and "Final" not in l]
ldata = []
for l in data:
    try:
        newvec = [float(x) for x in l.split(',') if len(x.strip()) > 0]
        if len(newvec) < ncols:
            newvec.extend([0]*(ncols - len(newvec)))
        else:
            if len(newvec) > ncols:
                newvec = newvec[:ncols]
        ldata.append(newvec)
    except ValueError:
        pass
data = ldata
data = np.vstack([r for r in data if len(r) == ncols])

all_t = data[:,0] / 1e6
all_mtr = data[:,1]
all_setpoint = data[:,2]
all_measured = data[:,3]
all_controlled = data[:,4]
all_positions = data[:,5]
all_updateDelays = data[:,6]

f = plt.figure(figsize=(22,12))
A = [f.add_subplot(4,2,i+1) for i in range(8)]
#f, A = kv.fa(numAxes=8)

for ifig, which in enumerate((all_mtr, -(all_mtr-1))):
    which = which.astype(bool)
    t = all_t[which]
    setpoint = all_setpoint[which]
    measured = all_measured[which]
    controlled = all_controlled[which]
    positions = all_positions[which]
    updateDelays = all_updateDelays[which]
    
    tickSpeedUnits = " [ticks/s $\cdot$ 256000]"
    
    A[ifig].plot(t, setpoint, label="setpoint" + tickSpeedUnits, color="red")
    A[ifig].plot(t, measured, label="measured" + tickSpeedUnits, color="blue")
    #A[ifig].scatter(t, np.abs(measured), label="|measured|" + tickSpeedUnits, color="yellow", s=1, zorder=88)
    A[ifig+2].plot(t, controlled, label="controlled" + tickSpeedUnits, color="green")
    posMod = 1024
    A[ifig+4].plot(t, np.mod(positions, posMod), label="mod(positions, %d) [ticks]" % posMod, color="magenta")
    A[ifig+4].scatter(t, np.mod(positions, posMod), label="mod(positions, %d) [ticks]" % posMod, color="magenta")
    
    A[ifig+6].plot(t, updateDelays, label="delay between interrupts [$\mu s$]", color="black")

    #A[ifig+4].scatter(t, np.mod(positions, posMod), label="mod(positions, %d)" % posMod, color="magenta")
    
    A[ifig].axhline(0, color="black")
    A[ifig+2].axhline(0, color="black")
    A[ifig+6].axhline(0, color="black")
    def rescaleYlims(ax):
        lo, hi = ax.get_ylim()
        diff = hi - lo
        ax.set_ylim(lo-diff*.1, hi+diff*.1)
    
    A[ifig].set_title("motor %d" % int(not ifig))
    if ifig == 0:
        A[ifig].legend(loc="best").set_zorder(99)
        A[ifig+2].legend(loc="best")
        A[ifig+4].legend(loc="best")
        A[ifig+6].legend(loc="best")
    A[ifig+4].set_xlabel("time [s]")

for a in A:
    rescaleYlims(a)
    
#for a in A:
    #a.set_xlim(13.9, 14.1)
    
f.savefig("data.png")
    
kv.plotting.show()
    
