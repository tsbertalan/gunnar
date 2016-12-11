import RPi.GPIO as GPIO
GPIO.setwarnings(False)
import numpy as np
import time
from collections import deque

GPIO.setmode(GPIO.BOARD)

class Motor(object):
    def __init__(self, spdPin, dirPin, curPin, hz=10000):
        self.pins = spdPin, dirPin, curPin
        GPIO.setup(spdPin, GPIO.OUT)
        self.p = GPIO.PWM(spdPin, hz)

        GPIO.setup(dirPin, GPIO.OUT)
        self.dp = GPIO.PWM(dirPin, 10)
        self.dp.start(0)

        self._isStarted = False
        self.frac = self.pct = 0
        
    def __repr__(self):
        return 'Motor(%d, %d, %d)' % self.pins
    
    def setFrac(self, frac, verbose=False):
        fwd = (frac > 0)
        
        # Bound frac within [-1, 1].
        oldFrac = frac
        frac = max(min(frac, 1.0), -1.0)
        
        if np.abs(oldFrac) > 1.0:
            from warnings import warn
            warn("Capping motor frac from %s to %s." % (oldFrac, frac))
        
        
        # Get magnitude as a percent.
        pct = np.abs(frac * 100.0)
        

        if np.abs(frac) < .01:
            self.p.ChangeDutyCycle(0)
            self.stop()
        else:
            if verbose: print 'Setting frac to %f.' % frac
            if not self._isStarted:
                self.p.start(pct)
                self._isStarted = True
            else:
                self.p.ChangeDutyCycle(pct)
            if fwd:
                self.dp.ChangeDutyCycle(0)
            else:
                self.dp.ChangeDutyCycle(100)

        self.frac = frac
        self.pct = pct
    
    def stop(self):
        self.p.stop()
        self._isStarted = False


class Encoder(object):
    
    def __init__(self, intPin, aPin, bPin, backward=False, dtHistMaxLen=10, halfRes=True, detectionDirection='rising'):
        # a  b  i  a  b  i
        # 12 16 7 11 13 15
        if halfRes:
            intPin = aPin
        self.intPin = intPin
        self.wavePins = aPin, bPin
        self.prev = (0, 0)
        self.prevTime = time.time()
        self.dtHist = deque()
        self.dtHistMaxLen = dtHistMaxLen
        self.dt = 1
        self.pos = 0
        for pin in intPin, aPin, bPin:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        if backward:
            self.reverser = -1
        else:        
            self.reverser = 1
            
        direction = {
            'rising': GPIO.RISING,
            'falling': GPIO.FALLING,
            'both': GPIO.BOTH
            }[detectionDirection.lower()]
                
        GPIO.add_event_detect(intPin, direction, callback=self.interruptCallback)
        
    def __del__(self):
        try:
            GPIO.remove_event_detect(self.intPin)
        except:
            pass
        
    @property
    def dt(self):
        return self._dt
    
    @dt.setter
    def dt(self, dtVal):
        self._dt = dtVal
        self.dtHist.append(dtVal)
        if len(self.dtHist) > self.dtHistMaxLen:
            self.dtHist.popleft()
            
    def smoothedDt(self):
        return sum(self.dtHist) / len(self.dtHist)
            
    def interruptCallback(self, data):
        self.pos += self.reverser
        
        
if __name__ == '__main__':
    print 'motor.main'
    try:
        enc = Encoder(7, 12, 16, backward=True)
        while True:
            time.sleep(.01)
            print 'position =',
            print enc.pos
    except KeyboardInterrupt:
        GPIO.cleanup()
    GPIO.cleanup()
    