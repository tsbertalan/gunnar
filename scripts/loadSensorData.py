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
assert data.shape[1] == 14
keys = (
    'ms', 
    'heading', 'roll', 'pitch',
    'x', 'y', 'z',
    'enc1pos', 'enc2pos',
    'enc1spd', 'enc2spd',
    'enc1stat', 'enc2stat',
    'isTurning'
    )

unpacked = {keys[i]: data[:, i] for i in range(14)}

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
fig, ax = plt.subplots(figsize=(16,9))
ax.plot(unpacked['x'], unpacked['y'], c='k')
ax.plot(unpacked['x'], unpacked['y'], c='k')
h = unpacked['heading']
ax.quiver(unpacked['x'], unpacked['y'], np.cos(h), np.sin(h),
          unpacked['ms'], cmap='hot')
        
plt.show()




