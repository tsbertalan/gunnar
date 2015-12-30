'''
Created on Dec 29, 2015

@author: tsbertalan

Accept a stream of LIDAR serial data from a client, parse it, and save each
completed scan to an HDF5 file.
'''
from gunnar.lidar.networkLogger import LidarLoggerServer

logger = LidarLoggerServer()
logger.main()
