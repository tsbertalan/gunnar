'''
Created on Nov 29, 2015

@author: tsbertalan

Load sensor data saved in an HDF5 file, and make some sort of a plot out of it.
Coincidentally, the data gets put in a (series of) numpy array(s), which might
be useful.
'''
from sys import argv

import tables

if len(argv) > 1:
    fname = argv[1]
else:
    from sys import exit
    exit('USAGE: %s H5FILEPATH' % argv[0])

f = tables.openFile(fname, 'r')
data = f.getNode('/sensorData')

print 'Loaded data with shape %s.' % (data.shape,)

# Data order is defined by the struct "SensorResponse" in gunnar/gunnar.h .
keys = [
'ms',
'heading', 'roll', 'pitch',
'enc1pos', 'enc2pos',
'enc1spd', 'enc2spd',
'enc1stat', 'enc2stat',
'accelX', 'accelY', 'accelZ',
'magX', 'magY', 'magZ',
'gyroX', 'gyroY', 'gyroZ',
]
assert data.shape[1] == len(keys), data.shape
\
unpacked = {keys[i]: data[:, i] for i in range(data.shape[1])}

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

# Attempt to integrate the IMU heading data and encoder odometry data.
x0 = np.array([0, 0])
thetas = unpacked['heading'] * np.pi / 180.0
odometer = unpacked['enc1pos']
stepSizes = odometer[1:] - odometer[:-1]
xs = [x0]
for theta, rt in zip(thetas, stepSizes):
    xt = xs[-1]
    xtp1 = xt + rt * np.array([np.cos(theta),
                               np.sin(theta)])
    xs.append(xtp1)
xs = np.vstack(xs)

# Plot odometry.
fig = plt.figure(figsize=(16,9))
fig.suptitle('integrated odometry and IMU heading')
ax = fig.add_subplot(111)
xy = xs[:, 0], xs[:, 1]
ax.plot(*xy, c='k')
ax.scatter(*xy, c=unpacked['ms'], lw=0, cmap='hot')
ax.set_xlabel('x'); ax.set_ylabel('y')
ax.quiver(xy[0], xy[1], np.cos(thetas), np.sin(thetas),
          unpacked['ms'], cmap='hot')

# Plot all data over time.
fig = plt.figure(figsize=(16,9))
for i in range(1, len(keys)):
    key = keys[i]
    x = unpacked['ms']
    y = unpacked[key]
    ax = fig.add_subplot(len(keys)-1, 1, i)
    ax.plot(x, y)
    ax.set_ylabel(key+'       .', rotation=12)
    ax.set_yticks([])
    if i != len(keys)-1:
        ax.set_xticks([])
    else:
        ax.set_xlabel('time [ms]')
fig.tight_layout()
fig.subplots_adjust(hspace=0)   

plt.show()
