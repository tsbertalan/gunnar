# Log data from Neato LIDAR via socket server

import logging
from threading import Thread
from time import sleep

from gunnar.io.network import Server
from gunnar.io.disk import PyTableSavingHandler as SavingHandler
from gunnar.lidar.parseLidar import LidarParser


class Watcher():
    '''Watch for an exit command, to stop the server.'''

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


class LidarLogger(object):
    def __init__(self):
        logging.getLogger().setLevel(logging.INFO)
    
        self.watcher = Watcher()
        self.server = Server(exitTimeCallback=self.watcher.exitNowCallback)
        self.parser = LidarParser(self.server, exitTimeCallback=self.watcher.exitNowCallback)
        self.handler = SavingHandler('data')
        
    def watchForNewParsedArrays(self):
        while True:
            sleep(0.00001)  # do not hog the processor power
            if self.watcher.exitNowCallback():
                break
            if len(self.parser) > 0:
                self.handler.enquque(self.parser.pop())
    
    
    
    def waitForConnections(self):
        threads = []
        for target in self.server.serve, self.parser.parse, self.watchForNewParsedArrays:
            threads.append(Thread(target=target))
            threads[-1].start()
            sleep(.1)
        self.watcher.watch()

