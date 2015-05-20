'''
Created on May 20, 2015

@author: bertalan@princeton.edu
'''
import numpy as np
import matplotlib.pyplot as plt

def getErr(h, g):
    for test in h, g:
        assert -180 < test < 180
    if h > g:
        delta = h - g
    else:
        delta = g - h

    whiles = 1
    while delta > 180:
        assert whiles < 2  # This should happen only once.
        delta -= 180
        delta *= -1
        delta += 180
        whiles += 1

    gp = h + delta
    whiles = 1
    while gp > 180:
        assert whiles < 2  # This should happen only once.
        gp -= 360
        whiles += 1
    if gp == g:
        m = 1
    else:
        m = -1

    delta *= m
    return delta

if __name__ == "__main__":
    fig = plt.figure()
    ax = fig.add_subplot(111)

    for g in [-179, -45, 45, 170]:
#     for h in np.arange(-179, 179, 1):
        epsFunc = np.vectorize(lambda x: getErr(x, g))
        h = np.arange(-179, 179)
        eps = epsFunc(h)

        r = (g + 180) / 360.
        ax.plot(h, eps, label="goal $=%d$" % g, c=(r, 1-r, 1-r), lw=2)
    ax.axhline(0, color="black")

    ax.set_xlabel("heading")
    ax.set_ylabel("error")
    ax.set_title("more red means larger goal")
    ax.legend()

    plt.show()
