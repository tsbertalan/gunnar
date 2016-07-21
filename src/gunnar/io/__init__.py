import serial
from collections import deque
import logging
from time import sleep
from gunnar.config import baudRate, systemOut


class Lights(object):
    
    @property
    def forwardLight(self):
        raise NotImplementedError
        
    @forwardLight.setter
    def forwardLight(self, onOffBool):
        raise NotImplementedError


class Robot(object):
    
    def __init__(self, serialPath=None):
        if serialPath is None:
            from serialFinder import port as serialPath
            
        # Make a new serial port object with the default parameters from some tutorial.
        self.serial = serial.Serial(
                                    port = serialPath,
                                    baudrate=baudRate,
                                    parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE,
                                    bytesize=serial.EIGHTBITS,
                                    timeout=0
                                    )
        #sleep(2.0)   # Give the arduino time to initialize.
        logging.info("Connected to serial port %s." % self.serial.portstr)
    
        # Make a double-ended queue to store incoming messages.
        # Outgoing messages will be sent immediately, and the Arduino must deal
        # somehow. Maybe we'll have it send an AWK.
        self.incoming = deque()
        self.maxChar = 1000  # Limit on number of characters in a line-message.
        
        self.lights = Lights()
        
    def __del__(self):
        print "Closing serial connection."
        self.serial.close()
        
        
    def getLine(self):
        line = []
        i = 0
        while True:
            if i == self.maxChar:
                break
            else:
                i += 1
            charsRead = [c for c in self.serial.read()]
            if len(charsRead) > 0:
                line.extend(charsRead)
                if charsRead[-1] == "\n":
                    line = "".join(line[:-1])
                    break
                #else:
                    #line = None
                    #break
        if len(line) == 0:
            line = None
        else:
            logging.info("[ARDUINO OUT]: %s" % line)
        return line
        
    def checkIncoming(self):
        """Read one line from serial."""
        line = self.getLine()
        if line is not None:
            self.incoming.appendleft(line)
        
    def send(self, msg, requireAwk=None):
        self.serial.write(msg)
        self.serial.flush()
        
        # We could have some logic here for verifying the receipt of a message,
        # such as requiring that the arduino send back "AWK"+msg or something 
        # like that.
        
    def getPhoto(self, cameraIndex):
        raise NotImplementedError
        
    @property
    def leftMotorLevel(self):
        raise NotImplementedError
    
    @leftMotorLevel.setter
    def leftMotorLevel(self, speed):
        assert -256 < speed < 256
        raise NotImplementedError
    
    @property
    def rightMotorLevel(self):
        raise NotImplementedError
    
    @leftMotorLevel.setter
    def rightMotorLevel(self, speed):
        assert -256 < speed < 256
        raise NotImplementedError
        
    @property
    def heading(self):
        raise NotImplementedError
    
    @property
    def pitch(self):
        raise NotImplementedError
    
    @property
    def roll(self):
        raise NotImplementedError
    
    @property
    def sonarDistance(self):
        raise NotImplementedError
    
    @property
    def leftMotorCurrent(self):
        raise NotImplementedError
    
    @property
    def rightMotorCurrent(self):
        raise NotImplementedError
    
    @property
    def leftEncoderTicks(self):
        raise NotImplementedError
    
    @property
    def leftOdometer(self):
        raise NotImplementedError
    
    @property
    def rightOdometer(self):
        raise NotImplementedError
        
if __name__ == "__main__":
    from time import sleep
    from pprint import pprint as pp
    logging.getLogger().setLevel(logging.DEBUG)
    
    r = Robot()
    
    for i in range(32):
        r.checkIncoming()
        sleep(.1)
    