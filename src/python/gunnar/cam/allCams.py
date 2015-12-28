#!/usr/bin/env python
import pygame
import pygame.camera
from pygame.locals import *      
from time import time, sleep
from os import system, listdir
from sys import argv

pygame.init()                    
pygame.camera.init()             

cameras = pygame.camera.list_cameras()[:]  # We only have enough USB bandwidth for 1 camera, it seems.
print "cameras:", cameras

cams = [pygame.camera.Camera(camPath, (640,480))
        for camPath in cameras]

def getImage(cam, nBrightAdjust=8):
    cam.start()
    for i in range(nBrightAdjust):
        #sleep(.1)
        image = cam.get_image()
    cam.stop()
    return image

dated = len(argv) > 1 and argv[1]

if len(argv) > 2 and argv[2]:
    continuous = True
    sessionIdentifier = str(time())
    system("ssh sigurd.tomsb.net mkdir -p /home/tsbertalan/cam/%s" % sessionIdentifier)
else:
    continuous = False

def sendImage(dated=False, flip=False, rotate=90):
    for i, cam in enumerate(cams):
        image = getImage(cam)
#        image = cam.get_image()
        now = time()
        if flip:
            image = pygame.transform.flip(image, False, True)  # flip vertically
        if rotate:
            if i == 1:
                rotate *= -1
            if i==2:
                rotate *= 0
            image = pygame.transform.rotate(image, rotate)  # rotate CCW

        fpaths = ["/tmp/cam%d.jpg" % i]
        if dated:
            fpaths.append("/tmp/cam%d_%s.jpg" % (i, now))
#        system("rm /tmp/cam*.jpg")
        for fpath in fpaths:
            pygame.image.save(image, fpath)
            cmd = "scp %s sigurd.tomsb.net:~/cam/" % fpath
            print "fpath is ", fpath
            if continuous and "_" in fpath:
                cmd += "%s/" % sessionIdentifier
            print cmd
            system(cmd)

while True:
    sendImage(dated=dated)
    if not continuous:
        break
    sleep(.01)


