# Log data from Neato LIDAR via socket server
'''
Created on Dec 29, 2015

@author: tsbertalan

Read serial data from the Neato LIDAR sensor, and pack it into a topic.
'''
import logging
from threading import Thread
from time import sleep
import sys

import numpy as np


import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32 

from gunnar.lidar import LidarParser, LidarSerialConnection

from __init__ import ROSNode

class LidarPublisher(ROSNode):
    def __init__(self, device='/dev/ttyUSB0', baudrate=115200):
        self.connection = LidarSerialConnection(com_port=device, baudrate=baudrate)
        self.parser = LidarParser(self.connection)
        
        self.messageScan = LaserScan()
        self.messageScan.angle_min = 0
        self.messageScan.angle_max = np.pi*2
        self.messageScan.angle_increment = np.pi*2 / 360
        self.messageScan.header.frame_id = 'lidar'
        
        self.messageRpm = Float32()
        
        self.publisherScan = rospy.Publisher('/scan', LaserScan, queue_size=20)
        self.publisherRPM = rospy.Publisher('/lidar_rpm', Float32, queue_size=20)
        
    def main(self):
        for scan, rpm in self.parser.parse():
            if scan is not None:
                # Construct and publish the LIDAR scan messageScan.
                # scans are (360, 2) arrays; first column is distance in mm, second is quality.
                self.messageScan.header.stamp = rospy.get_rostime()
                self.messageScan.scan_time = 1
                self.messageScan.range_min = .06  # cut off 
                self.messageScan.range_max = 50
                self.messageScan.ranges = scan[:, 0].astype(float) / 1000
                self.messageScan.intensities = scan[:, 1]
                self.publisherScan.publish(self.messageScan)
                
                # Construct and publish the LIDAR rpm messageScan.
                self.messageRpm.data = rpm
                self.publisherRPM.publish(self.messageRpm)
