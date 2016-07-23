#!/usr/bin/python
import logging
import random
import sys
import glob
import serial
from struct import unpack, calcsize
import time
import rospy

import numpy as np

from gunnar.io.usb import CmdMessenger
from serial.tools import list_ports

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
            rospy.logerr(str(e))

    ############################### C O M M A N D S ###########################
    def acknowledge(self, wait=True):
        rospy.logdebug('Sending ACK command.')
        self.messenger.send_cmd(self.commands.index('acknowledge'))
        if wait:
            # Wait until the arduino sends an acknowledgement back
            self.messenger.wait_for_ack(ackid=self.commands.index('acknowledgeResponse'))

    def sensorsRequest(self):
        rospy.logdebug('Sending sensor data request.')
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
        msg = 'Received command %s' % received_command
        if isinstance(received_command, int) and received_command < len(self.commands):
            msg += ' (%s)' % self.commands[received_command]
        msg += '.'
        rospy.logdebug(msg)
        s = self.sensorDataSize
        types = self.types
        try:
            byteString = args[0][-1]
            if len(byteString) >= s:
                arr = unpack(types, byteString[:s])
                self.nresp += 1
                
                # Save the data in our HDF5 file.
                data = np.empty((self.nfields,))
                assert len(arr) == self.nfields, (len(arr), self.nfields)
                data[:] = arr
#                 self.handler.enqueue((data,))
                  
                # Construct a sensor status message.
                m = ' '.join([
                        '%s=%s' % (k, v)
                        for (k, v) in zip(self.sensorFields, data)
                    ])
                # Ensure each line of the sensor status message is close to 80 chars.
                statements = m.split()
                segments = []
                segmentSize = 79
                segment = ''
                for statement in statements:
                    if len(segment) < segmentSize:
                        segment += ' ' + statement
                        m = m[segmentSize:]
                    else:
                        segments.append(segment)
                        segment = statement
                if segment != '':
                    segments.append(segment)
                segments.extend(['%s=%s' % (k, v) for (k, v) in zip(self.sensorFields[4:6], data[4:6])])
                rospy.logdebug('\n '.join(segments))
            else:
                rospy.logdebug('byteString of length %d is not long enough (%d).' % (len(byteString), s))

                
            
        except Exception as e:
            from traceback import format_exception
            from sys import exc_info
            tb = ''.join(['!! ' + l for l in format_exception(*exc_info())])
            self.statusMessage = "Failed with %s: %s" % (type(e), e) + '\n' + tb

    def logMessage(self, received_command, *args, **kwargs):
        '''Callback to log string messages sent by for debugging.'''
        rospy.loginfo('Got message from Arduino: %s' % ('|'.join(args[0]),))
        
    def printStatus(self, msg):
        print msg

        
if __name__ == '__main__':
    from time import sleep
    
    gunnarCommunicator = GunnarCommunicator()
    gunnarCommunicator.loopOnce()
