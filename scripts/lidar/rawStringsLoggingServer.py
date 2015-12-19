# Log data from Neato LIDAR via socket server

from threading import Thread, Event
from sys import stdout
import logging
from time import time, sleep

import numpy as np
import tables  # apt-get installed version 2.3.1, which wasn't new enough. "pip install --upgrade tables" updated to 3.2.2.

from server import Server
from parseLidar import LidarParser
from collections import deque

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

    def __init__(self, fname, dataShape=(360, 2)):

        # Open the table file.
        if fname[-3:] != ".h5":
            fname += ".h5"
        self.fname = fname
        self.file = tables.open_file(fname, mode='w', title='lidar data file')

        self.nsaved = 0

        # Define the data shape.
        atom = tables.UInt32Atom()
        # Make the enlargable array.
        dataShape = list(reversed(dataShape))
        dataShape.append(0)
        dataShape = tuple(reversed(dataShape))
        self.array_c = self.file.createEArray(self.file.root, 'scans', atom, dataShape, "Scans EArray", expectedrows=100000)

    def enquque(self, data):
        if (
            isinstance(data, np.ndarray)
            and data.size > 0
            and len(data.shape) == 2
            and data.shape[1] == 2
            ):
            
            # Save the data array.
            self.array_c.append([data])
            self.file.flush()
            self.nsaved += 1
            print '\r%09d scans saved.' % self.nsaved,
            stdout.flush()
            #t = time()
            #logging.info('Saving data of shape %s at t=%s.' % (data.shape, t))

    # pytables handles this for us:
    #def __del__(self):
    #    self.file.close()


class PyTableSavingLidarcharHandler(PyTableSavingHandler):

    def __init__(self, *args):
        self.parser = LidarParser()
        super(PyTableSavingLidarcharHandler, self).__init__(*args)

    def enquque(self, line):
        self.parser.readLine(line)
        logging.info("Got line of length %d." % len(line))
        if len(self.parser) > 0:
            super(PyTableSavingLidarcharHandler, self).enquque(self.parser.pop())


SavingHandler = PyTableSavingHandler


class Watcher():

    def __init__(self):
        self.exitNow = False

    def watch(self):
        msg = "Type 'exit' and push enter to quit:\n"
        while True:
            try:
                sleep(0.00001)  # do not hog the processor power
                inputText = raw_input(msg).lower()
                logging.debug("Input text was %s." % inputText)
                self.exitNow = 'exit' in inputText
                if self.exitNow:
                    logging.debug("Should be exiting now.")
                    sleep(1.0)
                    break
            except KeyboardInterrupt:
                self.exitNow = True
                break


    def exitNowCallback(self):
        return self.exitNow


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.INFO)

    watcher = Watcher()
    server = Server(exitTimeCallback=watcher.exitNowCallback)
    parser = LidarParser(server, exitTimeCallback=watcher.exitNowCallback)
    handler = SavingHandler('data')
    def watchForNewParsedArrays():
        while True:
            sleep(0.00001)  # do not hog the processor power
            if watcher.exitNowCallback():
                break
            if len(parser) > 0:
                handler.enquque(parser.pop())


    threads = []
    for target in server.serve, parser.parse, watchForNewParsedArrays:
        threads.append(Thread(target=target))
        threads[-1].start()
        sleep(.1)

    watcher.watch()
