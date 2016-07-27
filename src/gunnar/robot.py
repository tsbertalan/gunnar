#!/usr/bin/python
import logging
import random
import sys
import glob
import serial
from struct import unpack, calcsize
import time
import rospy

import matplotlib.pyplot as plt

import numpy as np

from gunnar.io.usb import CmdMessenger
from serial.tools import list_ports

from geometry_msgs.msg import QuaternionStamped
from std_msgs.msg import Header
import tf

# from gunnar.io.disk import PyTableSavingHandler

def get_serial_ports():
    """ Lists serial port names
    
    http://stackoverflow.com/a/14224477/1224886

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        if port not in result:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
    return result


class GunnarCommunicator(object):

    def __init__(self, fname="data/gunnarCommunicator.h5", logLength=28):
        rospy.logdebug('Begin GunnarCommunicator init.')
        self.statusHistory = [''] * logLength
        
        self.millisDiff = None

        # Define the ordering and sizes of data in incoming sensor packets.
        self.sensorFields = [
            'ms',
            'heading', 'roll', 'pitch',
            'enc1pos', 'enc2pos',
            'enc1spd', 'enc2spd',
            'enc1stat', 'enc2stat',
            'accelX', 'accelY', 'accelZ',
            'magX', 'magY', 'magZ',
            'gyroX', 'gyroY', 'gyroZ',
            ]
        # L : unsigned long
        # l : signed long
        # f : float / double
        # H : bool
        self.types = (
            'L'
            'fff'
            'll'
            'll'
            'HH'
            'fff'
            'fff'
            'fff'
            )
        self.sensorDataSize = calcsize(self.types)
        self.nfields = nfields = len(self.sensorFields)
        
#         # Create a handler object for saving data.
#         rospy.logdebug('Make a PyTable saving object.')
#         self.handler = PyTableSavingHandler(fname, dataShapes=((nfields,),),
#                                             printFn=False,
#                                             AtomClasses=(tables.FloatAtom,),
#                                             )
        
        # Make sure this matches the baudrate on the Arduino.
        self.baud = 19200
        
        self.commands = [
                         'acknowledge',    # outgoing
                         'error',          
                         'speedSet',       # outgoing
                         'sensorsRequest', # outgoing
                         'sensorsResponse',
                         'acknowledgeResponse',
                         'msg',
                         ]

        try:
            # Try to open a USB port.
            ports = get_serial_ports()
            rospy.logdebug('USB ports available are %s.' % (ports,))
            if len(ports) == 0:
                raise EnvironmentError('No active USB ports found!')
            ports.sort(key=lambda port: 'C' in port)
            self.port_name = ports[-1]
            rospy.logdebug('Trying to open USB port at %s.' % self.port_name)
            self.serial_port = serial.Serial(self.port_name, self.baud, timeout=0)
        except (serial.SerialException, IndexError) as e:
            raise SystemExit('Could not open serial port %s: %s' % (self.port_name, e))
        else:
            rospy.logdebug('Serial port %s sucessfully opened.' % self.port_name)
            self.messenger = CmdMessenger(self.serial_port, cmdNames=self.commands)
            
            # attach callbacks
            self.messenger.attach(func=self.onError, msgid=self.commands.index('error'))
            self.messenger.attach(func=self.onSensorsResponse,
                                  msgid=self.commands.index('sensorsResponse'))
            self.messenger.attach(func=self.onError, msgid=None)
            self.messenger.attach(func=self.logMessage, msgid=self.commands.index('msg'))

            # Send a command that the arduino will acknowledge.
            time.sleep(1.0);  # Give the Arduino time to initialize first.
            
            rospy.logdebug('Requesting ACK from arduino.')
            self.acknowledge(wait=False)
            rospy.logdebug('Arduino ready.')
            
        # Assign topics for publishing sensor data.
        self.header = Header()
        self.quaternionStamped = QuaternionStamped()
        self.orientationPublisher = rospy.Publisher('~IMUorientation', QuaternionStamped, queue_size=5)
       
        self.doPlot = False
        if self.doPlot: 
            self.fig = plt.figure()
            self.axes = {
                 'heading': self.fig.add_subplot(3, 3, 1, projection='polar'),
                 'roll':    self.fig.add_subplot(3, 3, 2, projection='polar'),
                 'pitch':   self.fig.add_subplot(3, 3, 3, projection='polar'),
                 'encpos':  self.fig.add_subplot(3, 2, 3),
                 'encspd':  self.fig.add_subplot(3, 2, 4),
                 '9dof':    self.fig.add_subplot(3, 1, 3)
                         }
            from collections import deque
            self.orientationHistory = {'heading': deque(), 'roll': deque(), 'pitch': deque()}
            self.fig.subplots_adjust(wspace=.3)
            plt.ion()
            plt.show()
            
    def mcuSecs2RosSecs(self, mcuSecs):
        if self.millisDiff is None:
            now = rospy.rostime.get_rostime().to_sec()
            self.millisDiff = now - mcuSecs
        return mcuSecs + self.millisDiff

    def list_usb_ports(self):
        """ Use the grep generator to get a list of all USB ports.
        """
        ports = [port for port in list_ports.grep('usb')]
        return ports

    def loopOnce(self):
        # Request sensor data.
        self.sensorsRequest()

        # Check to see if any data has been received
        try:
            self.messenger.feed_in_data()
        except (ValueError, RuntimeWarning) as e:
            from sys import exc_info
            import traceback
            errtype, unused_errval, errtb = exc_info()
            tbstr = traceback.format_tb(errtb)
            rospy.logerr('%s:\n%s' % (errtype, ''.join(tbstr).strip())) 

    ############################### C O M M A N D S ###########################
    def acknowledge(self, wait=True):
        rospy.logdebug('Sending ACK command.')
        self.messenger.send_cmd(self.commands.index('acknowledge'))
        if wait:
            # Wait until the arduino sends an acknowledgement back
            self.messenger.wait_for_ack(ackid=self.commands.index('acknowledgeResponse'))

    def sensorsRequest(self):
        self.messenger.send_cmd(self.commands.index('sensorsRequest'))
        ## This doesn't work:
        #self.messenger.wait_for_ack(ackid=self.commands.index('acknowledgeResponse'),
                                    #msgid=self.commands.index('sensorsResponse')
            #)
        
    def speedSet(self, left, right):
        msg = 'Sending speedSet(%s, %s) command.' % (left, right)
        rospy.logdebug(msg)
        self.messenger.send_cmd(self.commands.index('speedSet'), left, right)

    ####################### R E S P O N S E   C A L L B A C K S ###############
    
def onError(self, received_command, *args, **kwargs):
        """Callback function to handle errors
        """
        rospy.logdebug('Got unrecognized data (id %s args %s kwargs %s) from Arduino.' % (received_command, args, kwargs))

    nresp = 0.
    def onSensorsResponse(self, received_command, *args, **kwargs):
        """Callback to handle the sensor data response
        """
        s = self.sensorDataSize
        types = self.types
        try:
            byteString = args[0][-1]
            if len(byteString) >= s:
                arr = unpack(types, byteString[:s])
                self.nresp += 1
                
                # Put the sensor data into ROS topics.
                data = np.empty((self.nfields,))
                assert len(arr) == self.nfields, (len(arr), self.nfields)
                data[:] = arr
                timestampSecs = self.mcuSecs2RosSecs(data[0] / 1000.)
                
                dataDict = {k:v for (k,v) in zip(self.sensorFields, data)}
                
                # We'll have to transform the roll, pitch, and heading somehow
                # since the IMU board is mounted vertically.
                q = self.quaternionStamped.quaternion
                q.x, q.y, q.z, q.w = tf.transformations.quaternion_from_euler(
                    dataDict['roll'], dataDict['pitch'], dataDict['heading']
                    )
                self.quaternionStamped.header.stamp = rospy.Time(secs=timestampSecs)
                self.orientationPublisher.publish(self.quaternionStamped)

                ## Plot sensor data (slow).
                if self.doPlot:
                    start = time.time()
                    for k in self.axes:
                        self.axes[k].cla()
                        self.axes[k].set_title(k)
                    
                    for key in 'heading', 'roll', 'pitch':
                        theta = dataDict[key]
                        self.orientationHistory[key].append(theta)
                        L = lambda : len(self.orientationHistory[key])
                        if L() > 4:
                            self.orientationHistory[key].popleft()
                        l = float(L())
                        for i, theta in enumerate(self.orientationHistory[key]):
                            color = [i/l]*3
                            ax = self.axes[key]
                            ax.plot([theta]*2, [0,1], color=color)
                            ax.set_rticks([])
                            ax.set_xticks([])
                        
                    for key in 'encpos', 'encspd', '9dof':
                        if key == 'encpos':
                            nx = 2
                            y = dataDict['enc1pos'], dataDict['enc2pos']
                            labels = '1', '2' 
                        elif key == 'encspd':
                            nx = 2
                            y = dataDict['enc1spd'], dataDict['enc2spd']
                            labels = '1', '2'
                        else:
                            nx = 9
                            y = [dataDict[k] for k in self.sensorFields[-9:]]
                            labels = self.sensorFields[-9:]
                        x = np.arange(1, nx+1).astype(float)
                        ax = self.axes[key]
                        ax.bar(x, y)
                        ax.set_xticks(x + .5)
                        ax.set_xticklabels(labels, rotation=45)
                    
                    plt.draw()
                    rospy.loginfo('Took %.3f seconds to draw figure.' % (time.time() - start,))
                
            else:
                rospy.logdebug('byteString of length %d is not long enough (%d).' % (len(byteString), s))

                
            
        except Exception as e:
            from traceback import format_exception
            from sys import exc_info
            tb = ''.join(['!! ' + l for l in format_exception(*exc_info())])
            self.statusMessage = "Failed with %s: %s" % (type(e), e) + '\n' + tb
            rospy.logerr(self.statusMessage)

    def logMessage(self, received_command, *args, **kwargs):
        '''Callback to log string messages sent by for debugging.'''
        rospy.loginfo('Got message from Arduino: %s' % ('|'.join(args[0]),))
        
    def printStatus(self, msg):
        print msg

        
if __name__ == '__main__':
    from time import sleep
    
    gunnarCommunicator = GunnarCommunicator()
    gunnarCommunicator.loopOnce()
