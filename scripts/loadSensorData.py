'''
Created on Nov 29, 2015

@author: tsbertalan
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
assert data.shape[1] == 10
keys = (
    'ms', 
    'heading', 'roll', 'pitch',
    'encoder 1 position', 'encoder 2 position',
    'encoder 1 speed', 'encoder 2 speed',
    'motor 1 status', 'motor 2 status',
    )

unpacked = {keys[i]: data[:, i] for i in range(data.shape[1])}

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

# Plot 3D data.
fig = plt.figure(figsize=(16,9))
fig.suptitle('angular data')
ax = fig.add_subplot(211, projection='3d')
ax.plot(unpacked['heading'], unpacked['roll'], unpacked['pitch'], c='k')
ax.scatter(unpacked['heading'], unpacked['roll'], unpacked['pitch'], c=unpacked['ms'], lw=0, cmap='hot')
ax.set_xlabel('heading'); ax.set_ylabel('roll'); ax.set_zlabel('pitch')

for i in range(3):
    ax = fig.add_subplot(6,1,i+4)
    ax.plot(unpacked['ms'], [unpacked['heading'],
                             unpacked['roll'],
                             unpacked['pitch'],
                             ][i])
    if i != 2:
        ax.set_xticks([])
    else:
        ax.set_xlabel('time [ms]')
    ax.set_ylabel(['heading', 'roll', 'pitch'][i])
fig.tight_layout()

# Plot all data over time.
fig = plt.figure(figsize=(16,9))
for i in range(1, len(keys)):
    key = keys[i]
    x = unpacked['ms']
    y = unpacked[key]
    ax = fig.add_subplot(len(keys)-1, 1, i)
    ax.plot(x, y)
    ax.set_ylabel(key)
    if i != len(keys)-1:
        ax.set_xticks([])
    else:
        ax.set_xlabel('time [ms]')
        

# Plot 2D data with heading.
fig, ax = plt.subplots(figsize=(16,9))
ax.plot(unpacked['heading'], unpacked['roll'], c='k')
h = unpacked['heading']
ax.quiver(unpacked['heading'], unpacked['roll'], np.cos(h), np.sin(h),
          unpacked['ms'], cmap='hot')
ax.set_xlabel('heading'); ax.set_ylabel('roll')

plt.show()




