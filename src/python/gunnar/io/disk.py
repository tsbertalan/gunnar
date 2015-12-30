import logging

import numpy as np
import tables  # apt-get installed version 2.3.1, which wasn't new enough. "pip install --upgrade tables" updated to 3.2.2.

from gunnar.lidar import LidarParser

def print_function(s):
    print s

class Handler(object):
    pass

class ReprSavingHandler(Handler):

    def __init__(self, fname):
        if fname[-3:] != ".py":
            fname += ".py"
        self.fname = fname
        self.f = open(fname, 'w')
        self.f.write('from numpy import array, dstack\n')
        self.f.write('data = dstack((\n')

    def enqueue(self, data):
        if (
            isinstance(data, np.ndarray)
            and data.size > 0
            and len(data[0]) == 2
            ):
            self.f.writelines(repr(data).split('\n'))
            self.f.write(',\n')

    def __del__(self):
        print 'Closing file %s.' % self.fname
        self.f.write('))\n\n')
        self.f.close()


class PyTableSavingHandler(Handler):

    def __init__(self, fname, dataShape=(360, 2), printFn=print_function, dataName='scans', AtomClass=tables.UInt32Atom):
        self.printFn = printFn

        # Open the table file.
        if fname[-3:] != ".h5":
            fname += ".h5"
        self.fname = fname
        self.file = tables.open_file(fname, mode='w', title='lidar data file')

        self.nsaved = 0

        # Define the data shape.
        atom = AtomClass()
        # Make the enlargable array.
        self.dataShape = dataShape
        dataShape = list(reversed(dataShape))
        dataShape.append(0)
        dataShape = tuple(reversed(dataShape))
        self.array_c = self.file.createEArray(self.file.root, dataName, atom, dataShape,
                                              dataName + " EArray", expectedrows=100000)

    def enqueue(self, data):
        if (
            isinstance(data, np.ndarray)
            and data.shape == self.dataShape
            ):

            # Save the data array.
            self.array_c.append([data])
            self.file.flush()
            self.nsaved += 1
            self.printFn('\r%09d scans saved.' % (self.nsaved,))

    # pytables handles this for us:
    #def __del__(self):
    #    self.file.close()


class PyTableSavingLidarcharHandler(PyTableSavingHandler):

    def __init__(self, *args):
        self.parser = LidarParser()
        super(PyTableSavingLidarcharHandler, self).__init__(*args)

    def enqueue(self, line):
        self.parser.readLine(line)
        logging.info("Got line of length %d." % len(line))
        if len(self.parser) > 0:
            super(PyTableSavingLidarcharHandler, self).enqueue(self.parser.pop())

