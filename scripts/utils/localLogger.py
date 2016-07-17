#!/usr/bin/env python
'''
Created on Dec 30, 2015

@author: tsbertalan

Read serial data from the Neato LIDAR sensor, parse it, and save each
completed scan to an HDF5 file on the robot.
'''
from gunnar.lidar.localLogger import LocalLogger
logger = LocalLogger()
logger.main()
