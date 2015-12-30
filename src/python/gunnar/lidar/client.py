# Log data from Neato LIDAR
# based on code from Nicolas "Xevel" Saugnier
# requires pyserial


from array import array
import sys
import serial
from time import sleep

from gunnar.io.network import Client

class LidarLoggerClient(object):
    
    def __init__(self,
                 com_port = "/dev/ttyUSB0",  # example: 5 == "COM6" == "/dev/tty5"
                 baudrate = 115200,
                 ):
        self.ser = serial.Serial(com_port, baudrate)
    
    def sendData(self, hostname, port):
        client = Client(hostname, port, timeout=10)
        print "Connected to %s:%d. Will begin sending data. Ctrl+C to quit." % (hostname, port)
        sleep(1)
        BUFLEN = 512
        buf = array('c', ['a']*BUFLEN)

        while True:
            try:
                for i in range(BUFLEN):
                    buf[i] = self.ser.read(1)
                print "Sending buffer."
                client.send(buf.tostring())
            except KeyboardInterrupt:
                break
    
    def main(self):
        hostname = 'localhost'
        port = 9009
        if len(sys.argv) > 1:
            print sys.argv
            hostname = sys.argv[1]
            if len(sys.argv) > 2:
                port = int(sys.argv[2])
    
        self.sendData(hostname, port)


if __name__=='__main__':
    client = LidarLoggerClient()
    client.main()
