#!/usr/bin/env python
'''
Created on Nov 29, 2015

@author: tsbertalan

Make a movie from an HDF5 file full of LIDAR scans. Coincidentally, the data is
loaded into a NumPy array--that might be useful. 
'''
from sys import argv
import numpy as np

import tables

if True:
    if len(argv) != 2:
        from sys import exit
        from os.path import basename
        exit("USAGE: %s H5FILEPATH" % basename(argv[0])
             +'\n'+
             '    Make a movie from an HDF5 file full of LIDAR scans.'
             )
    else:
        fname = argv[1]
else:
    fname = '../data/localLogger.h5'    

f = tables.openFile(fname, 'r')
scans = f.getNode('/data0_Uint32Atom')
times = f.getNode('/data1_Float64Atom')
times -= min(times)
from scipy.io import savemat
savemat(fname.replace('.h5', '')+'.mat', {
                                          'lidarScans': np.asarray(scans).astype(np.float64),
                                          'times': np.asarray(times).astype(np.float64),
                                          })

print 'Loaded scans with %d rows, each with %d columns of %d elements.' % scans.shape


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
    ax.set_rlim([0, scans[:, :, 0].max()])

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
    for i in range(scans.shape[0]):
        yield scans[i]

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
            bar.update(float(i) / scans.shape[0] * 100)
            run(row)
            writer.grab_frame()
        bar.finish()





