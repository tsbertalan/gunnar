'''
Created on Dec 30, 2015

@author: tsbertalan
'''
from threading import Thread
from time import sleep, time

import tables

from gunnar.io.disk import PyTableSavingHandler
from gunnar.lidar import LidarSerialConnection, LidarParser
from gunnar.utils import printFnMulticall

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
        self.threads = [
                        Thread(target=f)
                        for f in (
                                  self.parser.parse,
                                  self._packData
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
        for thread in self.threads:
            thread.start()
            
    def stopThreads(self, timeout=10.0):
        self.printFn('Stopping Lidar logger with thread timeout %s.' % timeout)
        self.stopNow = True
        startTime = time()
        while True:
            if True not in [t.isAlive() for t in self.threads]:
                # If no threads are still alive, we're done here.
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
                print "Caught KeyboardInterrupt. Stopping logging threads."
                self.stopThreads()
                break
                
            
    
    