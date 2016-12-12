'''
Created on Dec 11, 2016

@author: tsbertalan
'''
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from __init__ import ROSNode
    
class CameraPublisher(ROSNode):
    
    def __init__(self):
        """OpenCV feature detectors with ros CompressedImage Topics in python.
        
        This example subscribes to a ros topic containing sensor_msgs 
        CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
        then detects and marks features in that image. It finally displays 
        and publishes the new image - again as CompressedImage topic.
        """
        self.publisher = rospy.Publisher("/image_raw/compressed", CompressedImage, queue_size=4)
        self.msg = CompressedImage()
        
        rospy.loginfo('Setting camera level...')
        self.camera = cv2.VideoCapture(int(rospy.get_param('camera_port', 0)))
        
        self.takeImage(lightSettingSnaps=30)
        
    def takeImage(self, lightSettingSnaps=0):
        for i in range(lightSettingSnaps):
            self.takeImage(lightSettingSnaps=0)
        return self.camera.read()[1]
    
    def publishImage(self):
        image = self.takeImage()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.format = "jpeg"
        self.msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        # Publish new image
        self.publisher.publish(self.msg)
        
    def main(self):
        r = rospy.Rate(rospy.get_param('imageRate', 10))
        idle = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publishImage()
            r.sleep()
            idle.sleep()
