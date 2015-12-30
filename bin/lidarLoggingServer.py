'''
Created on Dec 29, 2015

@author: tsbertalan
'''
from gunnar.lidar.logger import LidarLoggerServer

logger = LidarLoggerServer()
logger.waitForConnections()
