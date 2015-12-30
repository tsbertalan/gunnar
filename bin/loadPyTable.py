'''
Created on Nov 29, 2015

@author: tsbertalan
'''
from sys import argv

import tables

if len(argv) > 1:
    fname = argv[1]
else:
    fname = 'data.h5'

f = tables.openFile(fname, 'r')
data = f.getNode('/scans')

print 'Loaded data with %d rows, each with %d columns of %d elements.' % data.shape


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
fig, ax = plt.subplots(subplot_kw={'polar': True}, figsize=(16,9))
tdata, rdata = np.linspace(0, np.pi*2, 360), [0]*360


saveVideo = True
reDraw = saveVideo
if not reDraw:
    line, = ax.plot(tdata, rdata, 'ko', ms=4,
    #                 lw=0,
                    c='k',
                    )

didColorbar = [False]
def run(row):
    ax.cla()
    distance = row[:, 0]
    thetas = np.linspace(0, np.pi*2, 360)
    quality  = row[:, 1]
    ax.set_rlim([0, data[:, :, 0].max()])

    if False:
        ok = quality > 1
        quality = quality[ok]
        distance = distance[ok]
        thetas = thetas[ok]

    if reDraw:
        out, = ax.scatter(thetas, distance, s=10, c=quality, lw=0),
        if not didColorbar[0]:
            fig.colorbar(out, label="quality")
            didColorbar[0] = True
#         plt.show()
        return out
    else:
        line.set_data(thetas, distance)
        return line,

def data_gen():
    for i in range(data.shape[0]):
        yield data[i]

if not saveVideo:
    ani = animation.FuncAnimation(fig, run, data_gen, blit=True, interval=200, repeat=True)
    plt.show()
else:
    FFMpegWriter = animation.writers['ffmpeg']
    metadata = dict(title='Movie Test', artist='Matplotlib',
            comment='Movie support!')
    writer = FFMpegWriter(fps=15, metadata=metadata)

    dpi = 300
    movieFname = fname+"-color-d.mp4"
    with writer.saving(fig, movieFname, dpi):
        from progressbar import ProgressBar, Bar, ETA
        bar = ProgressBar(widgets=["Making %s. " % movieFname, ETA(), Bar()])
        bar.start()
        for i, row in enumerate(data_gen()):
            bar.update(float(i) / data.shape[0] * 100)
            run(row)
            writer.grab_frame()
        bar.finish()





