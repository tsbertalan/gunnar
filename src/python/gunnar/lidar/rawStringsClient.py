# Display Data from Neato LIDAR
# based on code from Nicolas "Xevel" Saugnier
# requires pyserial


import sys
import serial
import time
from array import array

from client import Client

com_port = "/dev/ttyUSB0"  # example: 5 == "COM6" == "/dev/tty5"
baudrate = 115200
ser = serial.Serial(com_port, baudrate)

def sendData(hostname, port):
    client = Client(hostname, port, timeout=10)
    print "Connected to %s:%d. Will begin sending data. Ctrl+C to quit." % (hostname, port)
    time.sleep(1)
    BUFLEN = 512
    buf = array('c', ['a']*BUFLEN)

    while True:
        try:
            for i in range(BUFLEN):
                buf[i] = ser.read(1)
            print "Sending buffer."
            client.send(buf.tostring())
        except KeyboardInterrupt:
            break

hostname = 'localhost'
port = 9009
if len(sys.argv) > 1:
    print sys.argv
    hostname = sys.argv[1]
    if len(sys.argv) > 2:
        port = int(sys.argv[2])

sendData(hostname, port)
