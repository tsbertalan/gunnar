import numpy as np
import kevrekidis as kv
data = np.load("slamScanTest/slamScanTestData.npz")
fast = data["fast"]
slow = data["slow"]
ft = fast[:,0]; fa = fast[:,1]; fr = fast[:,2]
st = slow[:,0]; sa = slow[:,1]; sr = slow[:,2]
transs = [
    (lambda a,r: (a,r)),
    (lambda a,r: (r*np.cos(a*np.pi/180), r*np.sin(a*np.pi/180)))
]
labels = [
    (r"$\theta$", "$r$"),
    ("$x$", "$y$"),
]
for label, trans in zip(labels, transs):
    f, A = kv.fa(numAxes=2)
    x, y = trans(fa, fr)
    kwargs = {'s': 20, 'lw': 0}
    sc1 = A[1].scatter(x, y, c=ft/1e6, **kwargs)
    x, y = trans(sa, sr)
    sc0 = A[0].scatter(x, y, c=st/1e6, **kwargs)
    A[0].set_title("slow")
    A[1].set_title("fast")
    for a in A[:2]:
        a.set_xlabel(label[0])
        a.set_ylabel(label[1])
    f.colorbar(sc0, ax=A[0])
    f.colorbar(sc1, ax=A[1])
    f.suptitle(str(trans))
    f2 = kv.plotting.plt.figure()
    a = f2.add_subplot(111, projection="polar")
    a.scatter(sa*np.pi/180, sr, c=st/1e6, **kwargs)
kv.plotting.show()

