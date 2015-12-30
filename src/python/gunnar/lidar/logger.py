# Log data from Neato LIDAR via socket server

from array import array
import logging
from threading import Thread
from time import sleep
import sys
import serial

from gunnar.io.network import Server, Client
from gunnar.io.disk import PyTableSavingHandler as SavingHandler
from gunnar.lidar.parseLidar import LidarParser


class Watcher():
    '''Watch for an exit command, to stop the server. Doesn't work.'''

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


class LidarLoggerServer(object):
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


class LidarLoggerClient(object):
    
    def __init__(self,
                 com_port="/dev/ttyUSB0",  # example: 5 == "COM6" == "/dev/tty5"
                 baudrate=115200,
                 ):
        self.ser = serial.Serial(com_port, baudrate)
    
    def sendData(self, hostname, port):
        client = Client(hostname, port, timeout=10)
        print "Connected to %s:%d. Will begin sending data. Ctrl+C to quit." % (hostname, port)
        sleep(1)
        BUFLEN = 512
        buf = array('c', ['a'] * BUFLEN)

        while True:
            try:
                for i in range(BUFLEN):
                    buf[i] = self.ser.read(1)
                print "Sending buffer."
                client.send(buf.tostring())
            except KeyboardInterrupt:
                break
    
    def main(self):
        hostname = 'localhost'
        port = 9009
        if len(sys.argv) > 1:
            print sys.argv
            hostname = sys.argv[1]
            if len(sys.argv) > 2:
                port = int(sys.argv[2])
    
        self.sendData(hostname, port)
