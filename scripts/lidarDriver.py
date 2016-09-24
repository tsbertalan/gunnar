#!/usr/bin/env python
'''
Created on Dec 29, 2015

@author: tsbertalan

Read serial data from the Neato LIDAR sensor, and pack it into a topic.
'''
from gunnar.lidar.lidarPublisher import LidarPublisher
import logging
logging.getLogger().setLevel(logging.DEBUG)
publisher = LidarPublisher()
publisher.main()
