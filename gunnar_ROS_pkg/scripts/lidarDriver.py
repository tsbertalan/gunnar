#!/usr/bin/env python
'''
Created on Dec 29, 2015

@author: tsbertalan

Read serial data from the Neato LIDAR sensor, and pack it into a topic.
'''
from gunnar.lidar.networkLogger import LidarLoggerClient
client = LidarLoggerClient()
client.main()
