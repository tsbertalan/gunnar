import numpy as np
import kevrekidis as kv
data = [l for l in open('data.csv').readlines() if len(l.strip()) > 0]
data = [[float(x) for x in l.split(',') if len(x.strip()) > 0] for l in data]
data = np.vstack([r for r in data if len(r) == 5])
t = data[:,0] / 1e6
lspd = data[:,1]
rspd = data[:,2]
lpos = data[:,3]
rpos = data[:,4]

f, A = kv.fa(numAxes=2)
a = A[0]
b = A[1]

a.scatter(t, lspd, color="blue", label="left")
a.scatter(t, rspd, color="red", label="right")
a.legend(loc="best")
a.set_title("velocity")

b.scatter(t, lpos, color="blue", label="left")
b.scatter(t, rpos, color="red", label="right")
b.legend(loc="best")
b.set_title("position")


kv.plotting.show()