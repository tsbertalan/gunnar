# Log data from Neato LIDAR via socket server

from threading import Thread
import logging
from time import time

import numpy as np
import tables

from server import Server, Handler


class ReprSavingHandler(Handler):

    def __init__(self, fname):
        if fname[-3:] != ".py":
            fname += ".py"
        self.fname = fname
        self.f = open(fname, 'w')
        self.f.write('from numpy import array, dstack\n')
        self.f.write('data = dstack((\n')

    def enquque(self, data):
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

    def __init__(self, fname):

        # Open the table file.
        if fname[-3:] != ".h5":
            fname += ".h5"
        self.fname = fname
        self.file = tables.open_file(fname, mode='w', title='lidar data file')

        # Define the data shape.
        atom = tables.UInt32Atom()
        # Make the enlargable array.
        self.array_c = self.file.createEArray(self.file.root, 'scans', atom, (0, 360, 2), "Scans EArray", expectedrows=100000)


    def enquque(self, data):
        if (
            isinstance(data, np.ndarray)
            and data.size > 0
            and len(data[0]) == 2
            ):
            self.array_c.append([data])
            #t = time()
            #logging.info('Saving data at t=%s.' % t)
            self.file.flush()


    def __del__(self):
        print 'Closing file %s.' % self.fname
        self.file.close()


SavingHandler = PyTableSavingHandler


class Watcher():

    exitNow = False

    def watch(self):
        msg = "Type 'exit' and push enter to quit:\n"
        print msg

        while True:
            self.exitNow = 'exit' in raw_input(msg).lower()
            if self.exitNow:
                break

        return "done"

    def exitNowCallback(self):
        return self.exitNow


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)

    handler = SavingHandler("data")
    watcher = Watcher()
    server = Server(handler, exitTimeCallback=watcher.exitNowCallback)

    serverThread = Thread(target=server.serve)
    watcherThread = Thread(target=watcher.watch)
    serverThread.start()
    result = watcherThread.start()

