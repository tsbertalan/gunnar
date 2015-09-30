import pygame
import pygame.camera
from pygame.locals import *      
from time import time, sleep
from os import system
from sys import argv

pygame.init()                    
pygame.camera.init()             

print "cameras:", pygame.camera.list_cameras()

cam = pygame.camera.Camera("/dev/video0",(640,480))                                  
cam.start()

# Take a few pictures to get the brightness set correctly.
for i in range(4):
	cam.get_image()
	sleep(1)

if argv[1]:
    dated = True

if argv[2]:
    continuous = True
    sessionIdentifier = str(time())
    system("ssh sigurd.tomsb.net mkdir -p /home/tsbertalan/cam/%s" % sessionIdentifier)

def sendImage(dated=False, flip=False, rotate=90):
    image = cam.get_image()
    now = time()
    if flip:
        image = pygame.transform.flip(image, False, True)  # flip vertically
    if rotate:
        image = pygame.transform.rotate(image, rotate)  # rotate CCW

    fpaths = ["/tmp/cam.jpg"]
    if dated:
        fpaths.append("/tmp/cam_%s.jpg" % now)
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

cam.stop()

