#import serial
from os import system
from subprocess import Popen, PIPE
from time import sleep
import unittest

def msg(text):
    print
    print ">>>>>>>>>>>>>>>", text, "<<<<<<<<<<<<<<<<<"

def systemOut(cmdList, sayCmd=True):
    '''Run a command and capture the output.'''
    if sayCmd:
        print "$", " ".join(cmdList)
    return Popen(cmdList, stdout=PIPE).communicate()[0]
#out = systemOut(["ping", "google.com", "-c", "4"])
#print "out is %s, dawg" % out.strip()

devices = systemOut(["ls", "/dev/"])
for i, dev in enumerate(devices.split('\n')):
    if "ACM" in dev:
        port = "/dev/%s" % dev.strip()
msg("port is %s" % port)
board = "arduino:avr:mega"
baudRate = 9600

def arduinoGo(arduCmd):
    cmd = "arduino --port %s --board %s %s" % (port, board, arduCmd)
    print "$", cmd 
    return system(cmd)

def verify(testName):
    return arduinoGo("--verify %s/%s.ino" % (testName, testName))

def upload(testName):
    return arduinoGo("--upload %s/%s.ino" % (testName, testName))

def monitor(sayCmd=False):
    print "To exit the serial monitor, do \"Ctrl+a, k, y\"."
    cmd = "gnome-terminal --disable-factory --command \"screen %s %s\" 2>&1 | grep -v \"format string\"" % (port, baudRate)
    if sayCmd:
        print "$", cmd
    system(cmd)
    
def printInstructions(test):
    prefix = " >>>     "
    try:
        lines = open("%s/README.txt" % test)
        print
        print "Inspection instructions:"
        for line in lines:
            print "%s %s" % (prefix, line.strip())
        return True
    except IOError:
        raise IOError("No instructions written for %s." % (test))
        return False

def yn(prompt):
    # stackoverflow.com/questions/3041986
    # raw_input returns the empty string for "enter"
    yes = set(['yes','y', 'ye', ''])
    no = set(['no','n'])
    print
    print prompt
    while True:
        choice = raw_input().lower()
        if choice in yes:
            return True
        elif choice in no:
            return False
        else:
            print "Please respond with '[y]es' or '[n]o'."

def doTest(testName=None):
    if testName is None:
        import traceback
        stack = traceback.extract_stack()
        filename, codeline, funcName, text = stack[-2]
        testName = funcName.replace("test_", '').strip()
    #msg(testName)
    testFailed = upload(testName)
    
    if testFailed:
        raise RuntimeError("Test %s failed to upload." % testName)
    else:
        if testName != "stop":
            haveInstructions = printInstructions(testName)
        if testName != "stop":
            monitor()
            #if haveInstructions:
            monitorPassed = yn("Did the test pass inspection?")
            if not monitorPassed:
                raise RuntimeError("Test %s failed inspection." % testName)

class TestGunnar(unittest.TestCase):

    def setUp(self):
        pass
    
    def test_daguMotorBoard(self):
        doTest()
        
    def test_encodersMotors(self):
        doTest()
                
    def test_encodersNoMotors(self):
        doTest()
                
    def test_motorDirections(self):
        doTest()
                
    def test_motorPid(self):
        doTest()
                
    def test_motorsSparkfun(self):
        doTest()
                
    def test_servos(self):
        doTest()

    @classmethod
    def tearDownClass(cls):
        doTest("stop")
        
if __name__=="__main__":
#     docTestAll()
    unittest.main(verbosity=9001)
#     t = TestKevrekidis()
#     t.test_chungLu()
