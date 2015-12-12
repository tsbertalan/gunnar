#!/usr/bin/env python
from os import listdir, system
from time import sleep

while True:
    dirs = [d for d in listdir("/media/samsungsd") if d[:5] == "cams_"]
    dirs.sort()  # Dirs are named by time()
    mostRecentDir = dirs[-1].replace("/", "")
    
    verbosity = ""
    #verbosity = "--progress"

    cmd = "rsync  --recursive %s /media/samsungsd/%s sigurd.tomsb.net:~/cam/" % (verbosity, mostRecentDir, )
    print "$ %s" % cmd
    system(cmd)


    sleep(16.0)

