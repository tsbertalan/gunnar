from sys import argv
from sys import exit
# from os import system
import subprocess

if len(argv) < 2:
    msg = 'USAGE: %s /dev/sdx' % argv[0]
#     stderr.write(msg+'\n')
    exit(msg)
    
dev = argv[1]
    

def getOutput(cmd):
    return subprocess.check_output(cmd, shell=True)

mountedDecices = getOutput('mount | grep "/dev/sd"').split('\n')
def getMountPoints(device):
    mountPoints = []
    for m in mountedDecices:
        if device in m:
            tokens = m.split()
            assert tokens[1] == 'on', m
            path = tokens[2]
            mountPoints.append(path)
    return mountPoints
    
if '/' in getMountPoints(dev):
    exit('Device %s is mounted at / !!!' % dev)

alldisks = [d for d in getOutput('sudo lshw -class disk').split('*-disk') if len(d.strip()) > 0]
diskDict = {}
for d in alldisks:
    data = {}
    for l in d.split('\n'):
        if ':' in l:
            l = l.split(':')
            l = l[0], ':'.join(l[1:])
            assert len(l) == 2, l
            key, val = l
            data[key.strip()] = val.strip()
    assert 'logical name' in data.keys()
    diskDict[data['logical name']] = data
from pprint import pprint

def p(s, space=True):
    print s,
for diskName, data in diskDict.items():
    if diskName == dev:
        p('>>>>')
    else:
        p('    ')
    p(diskName)
    p(data['vendor'])
    p(data['product'])
    p('(%s)' % data['description'])
    mountPoints = getMountPoints(diskName)
    if len(mountPoints) > 0:
        print
        p('       Mounted at:')
        print
        print '      ',
        p('\n       '.join(mountPoints))
    print
    
        
    
            
    

#  2001  lshw -class disk  
#  2002  sudo lshw -class disk  
#  2003  hwinfo --disk
#  2004  sudo aptitude install hwinfo
#  2005  sudo lshw -class disk  
#  2006  cat /sys/class/block/sdb/device/model
#  2007  cat /sys/class/block/sdb/device/vendor
#  2008* cat /sys/class/block/sdb/device/
#  2009  cat /sys/class/block/sdb/device/modalias
#  2010  cat /sys/class/block/sdb/device/rev 
#  2011  cat /sys/class/block/sdb/capability 
#  2012  cat /sys/class/block/sdb/range 
#  2013  history
# 
#     