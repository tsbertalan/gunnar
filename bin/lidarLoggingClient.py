'''
Created on Dec 29, 2015

@author: tsbertalan

Read serial data from the Neato LIDAR sensor, and send it via network socket
to a server. 
'''
from gunnar.lidar.networkLogger import LidarLoggerClient
client = LidarLoggerClient()
client.main()
