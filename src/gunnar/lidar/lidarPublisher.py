# Log data from Neato LIDAR via socket server

import logging
from threading import Thread
from time import sleep
import sys

from gunnar.io.network import Server
from gunnar.lidar import LidarParser, LidarSerialConnection

from sensor_msgs.msg import LaserScan

class LidarPublisher(object):
    def __init__(self):
        self.connection = LidarSerialConnection()
        self.parser = LidarParser(self.connection)
        
    def main(self):
        for scan in self.parser.parse():
            if scan is not None:
#                 print 'Got scan:',
                print scan[:, 0].mean(),
            else:
                print scan,
                    
