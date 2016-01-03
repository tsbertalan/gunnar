import logging

import numpy as np
import tables  # apt-get installed version 2.3.1, which wasn't new enough. "pip install --upgrade tables" updated to 3.2.2.

from gunnar.lidar import LidarParser
from gunnar.utils import print_function


class Handler(object):
    pass

# class ReprSavingHandler(Handler):
# 
#     def __init__(self, fname):
#         if fname[-3:] != ".py":
#             fname += ".py"
#         self.fname = fname
#         self.f = open(fname, 'w')
#         self.f.write('from numpy import array, dstack\n')
#         self.f.write('data = dstack((\n')
# 
#     def enqueue(self, data):
#         if (
#             isinstance(data, np.ndarray)
#             and data.size > 0
#             and len(data[0]) == 2
#             ):
#             self.f.writelines(repr(data).split('\n'))
#             self.f.write(',\n')
# 
#     def __del__(self):
#         print 'Closing file %s.' % self.fname
#         self.f.write('))\n\n')
#         self.f.close()


class PyTableSavingHandler(Handler):

    def __init__(self, fname, dataShapes=((360, 2),), printFn=print_function, AtomClasses=(tables.UIntAtom,)):
        self.printFn = printFn

        # Open the table file.
        if fname[-3:] != ".h5":
            fname += ".h5"
        self.fname = fname
        self.file = tables.open_file(fname, mode='w', title='lidar data file')

        self.nsaved = 0

        # Define the data shapes.
        self.dataShapes = []
        self.array_cs = []
        assert len(dataShapes) == len(AtomClasses), (len(dataShapes), len(AtomClasses)) 
        for dataShape, AtomClass, i in zip(
                                           dataShapes, AtomClasses, range(len(AtomClasses))
                                           ):
            atom = AtomClass()
            dataName = 'data%d_' % i + ''.join(type(atom).__name__.title().split())
            # Make the enlargable array.
            self.dataShapes.append(dataShape)
            dataShape = list(reversed(dataShape))
            dataShape.append(0)  # Add a dimension for the array to grow.
            dataShape = tuple(reversed(dataShape))
            array_c = self.file.createEArray(self.file.root, dataName, atom, dataShape,
                                             dataName + " EArray", expectedrows=100000)
            self.array_cs.append(array_c)

    def enqueue(self, arrays):
        if (
            isinstance(arrays, tuple)
            and
            len(arrays) == len(self.array_cs)
            ):
            for (
                 data, dataShape, array_c
                 ) in zip(
                          arrays,
                          self.dataShapes,
                          self.array_cs
                          ):
                if (
                    isinstance(data, np.ndarray)
                    and data.shape == dataShape
                    ):
        
                    # Save the data array.
                    array_c.append([data])
                    self.file.flush()
                    self.nsaved += 1
                    self.printFn('\r%09d scans saved.' % (self.nsaved,))
                else:
                    raise ValueError('Array had shape %s instead of the expected %s.' % (data.shape, dataShape))
        else:
            raise ValueError('An tuple of arrays of sizes %s should be passed to enqueue.' % (self.dataShapes,))
        
    # PyTables handles this for us at program exit, but this is useful
    # for interactive sessions.
    def __del__(self):
        self.printFn('Closing file %s.' % self.fname)
        self.file.close()


class PyTableSavingLidarcharHandler(PyTableSavingHandler):

    def __init__(self, *args):
        self.parser = LidarParser()
        super(PyTableSavingLidarcharHandler, self).__init__(*args)

    def enqueue(self, line):
        self.parser.readLine(line)
        logging.info("Got line of length %d." % len(line))
        if len(self.parser) > 0:
            super(PyTableSavingLidarcharHandler, self).enqueue(self.parser.pop())

