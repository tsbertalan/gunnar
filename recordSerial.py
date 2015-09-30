import serial
from time import time
import serialFinder
from config import baudRate
from sys import stdout
from os.path import join

verbose = False

port = serialFinder.port

ser = serial.Serial(
    port=port,\
    baudrate=baudRate,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)

print("Connected to serial port %s . " % ser.portstr)

#this will store the line
line = []

fname = join("data", "serial%d.out" % time())
f = open(fname, "w")
print "Opened file %s ." % fname

while True:
    try:
        for c in ser.read():
            line.append(c)
            if c == '\n':
                line = "#%s#    " % time() + "".join(line)
                f.write(line)
                f.flush();
                if verbose:
                    print "wrote %s" % line.strip()
                line = []
                break
    except KeyboardInterrupt:
        break

print "Closing serial port %s ..." % port,
ser.close()
print "done."

print "Closing output file %s..." % fname,
f.close()
print "done."
