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
