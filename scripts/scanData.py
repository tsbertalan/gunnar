import numpy as np
import kevrekidis as kv
data = np.load("../data/slamScanTestData.npz")
fast = data["fast"]
slow = data["slow"]
ft = fast[:,0]; fa = fast[:,1]; fr = fast[:,2]
st = slow[:,0]; sa = slow[:,1]; sr = slow[:,2]
f, A = kv.fa(numAxes=2)
fp, Ap = kv.fa(numAxes=2, polar=True)
kwargs = {'s': 20, 'lw': 0}

for (a, r, t, title, ax1, ax2) in zip(
                        (fa, sa),
                        (fr, sr),
                        (ft, st),
                        ("fast", "slow"),
                        A,
                        Ap
                        ):
    maxr = 60
    sc1 = ax1.scatter(-a*np.pi/180, r, c=t/1e6, **kwargs)
    ax1.set_ylim(maxr)
    f.colorbar(sc1, ax=ax1)
    sc2 = ax2.scatter(-a*np.pi/180, r, c=t/1e6, **kwargs)
    ax2.set_rlim(0, maxr)

    fp.colorbar(sc2, ax=ax2)

    ax1.set_title(title)
    ax2.set_title(title)

kv.plotting.show()

