'''
Created on Dec 30, 2015

@author: tsbertalan
'''
from time import sleep, time

import tables

from gunnar.io.disk import PyTableSavingHandler
from gunnar.lidar import LidarSerialConnection, LidarParser
from gunnar.utils import printFnMulticall
from multiprocessing import Process  # TODO: This might be preventing clean exit of the CLUI.

class LocalLogger(object):
    
    def __init__(self, fpath='data/localLogger.h5', printFn=printFnMulticall):
        self.conn = LidarSerialConnection()
        self.parser = LidarParser(self.conn, exitTimeCallback=self._isItTimeToStop)
        self.printFn = printFn
        self.handler = PyTableSavingHandler(fpath,
                                            dataShapes=(
                                                        (360, 2),
                                                        (1,)
                                                        ),
                                            AtomClasses=(
                                                         tables.UIntAtom,
                                                         tables.FloatAtom,
                                                         ),
                                            printFn=printFn,
                                            )
        self.processes = [
                        Process(target=f)
                        for f in (
                                  self.parser.parse,  # Reads LIDAR data from USB.
                                  self._packData  # Periodically puts that data in an HDF5 file. 
                                  )
                        ]
        self.stopNow = False
        
    def _isItTimeToStop(self):
        return self.stopNow
        
    def _packData(self):
        while True:
            if self._isItTimeToStop():
                break
            sleep(0.00001)
            if len(self.parser) > 0:
                self.printFn('Found new LIDAR data!')
                self.handler.enqueue(self.parser.pop())
                
    def startThreads(self):
        for process in self.processes:
            process.start()
            
    def stopThreads(self, timeout=10.0):
        self.printFn('Stopping Lidar logger with thread timeout %s.' % timeout)
        self.stopNow = True
        startTime = time()
        while True:
            if True not in [t.is_alive() for t in self.processes]:
                # If no processes are still alive, we're done here.
                break
            else:
                # Otherwise, give them a little time to see the stop flag. 
                if time() - startTime >= timeout:
                    raise RuntimeError("Threads took too long to exit.")
                else:
                    sleep(0.01)
                    
    def main(self):
        self.startThreads()
        
        while True:
            try:
                sleep(1)
            except KeyboardInterrupt:
                print "Caught KeyboardInterrupt. Stopping logging processes."
                self.stopThreads()
                break
                
            
    
    
