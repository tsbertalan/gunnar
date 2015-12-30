'''
Created on Dec 29, 2015

@author: tsbertalan
'''
from gunnar.lidar.networkLogger import LidarLoggerServer

logger = LidarLoggerServer()
logger.waitForConnections()
