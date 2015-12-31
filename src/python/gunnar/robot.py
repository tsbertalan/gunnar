#!/usr/bin/python
import logging
import random
import sys
import serial
from struct import unpack, calcsize
import time


import numpy as np

from gunnar.io.usb import CmdMessenger
from serial.tools import list_ports

from gunnar.io.disk import PyTableSavingHandler


class GunnarCommunicator(object):

    def __init__(self, fname="data/gunnarCommunicator.h5", logLength=28):
        import tables
        logging.debug('Begin GunnarCommunicator init.')
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
        
        # Create a handler object for saving data.
        logging.debug('Make a PyTable saving object.')
        self.handler = PyTableSavingHandler(fname, dataShape=(nfields,),
                                            printFn=lambda s: setattr(self, 'statusMessage', s),
                                            dataName='sensorData',
                                            AtomClass=tables.Float64Atom,
                                            )
        
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
            self.port_name = self.list_usb_ports()[-1][0]
            logging.debug('Trying to open USB port at %s.' % self.port_name)
            self.serial_port = serial.Serial(self.port_name, self.baud, timeout=0)
        except (serial.SerialException, IndexError) as e:
            raise SystemExit('Could not open serial port %s: %s' % (self.port_name, e))
        else:
            m = self.statusMessage = 'Serial port %s sucessfully opened.' % self.port_name
            logging.debug(m)
            self.messenger = CmdMessenger(self.serial_port, cmdNames=self.commands)
            # attach callbacks
            self.messenger.attach(func=self.onError, msgid=self.commands.index('error'))
            self.messenger.attach(func=self.onSensorsResponse,
                                  msgid=self.commands.index('sensorsResponse'))
            self.messenger.attach(func=self.onError, msgid=None)
            self.messenger.attach(func=self.logMessage, msgid=self.commands.index('msg'))

            # Send a command that the arduino will acknowledge.
            time.sleep(1.0);  # Give the Arduino time to initialize first.
            self.acknowledge()
            m = self.statusMessage = 'Arduino ready.'
            logging.debug(m)
        
    @property
    def statusMessage(self):
        return self._statusMessage
    
    @statusMessage.setter
    def statusMessage(self, args):
        if isinstance(args, tuple):
            self._statusMessage = ' '.join(['%s' % (s,) for s in args])
            lines = self._statusMessage.split('\n')
            for newLine in lines:
                for i in range(1, len(self.statusHistory)):  # Keep a scrolling history of status messages.
                    self.statusHistory[i - 1] = self.statusHistory[i]
                self.statusHistory[i] = newLine
        else:
            self.statusMessage = (args,)

    def list_usb_ports(self):
        """ Use the grep generator to get a list of all USB ports.
        """
        ports = [port for port in list_ports.grep('usb')]
        return ports

    def loopOnce(self):
        # Request sensor data.
        self.sensorsRequest()

        # Check to see if any data has been received
        self.messenger.feed_in_data()

    ############################### C O M M A N D S ###########################
    def acknowledge(self, wait=True):
        logging.debug('Sending ack command.')
        self.messenger.send_cmd(self.commands.index('acknowledge'))
        if wait:
            # Wait until the arduino sends an acknowledgement back
            self.messenger.wait_for_ack(ackid=self.commands.index('acknowledgeResponse'))

    def sensorsRequest(self):
        logging.debug('Sending sensor data request.')
        self.messenger.send_cmd(self.commands.index('sensorsRequest'))
        ## This doesn't work:
        #self.messenger.wait_for_ack(ackid=self.commands.index('acknowledgeResponse'),
                                    #msgid=self.commands.index('sensorsResponse')
            #)
        
    def speedSet(self, left, right):
        logging.debug('Sending speedSet(%s, %s) command.' % (left, right))
        self.statusMessage = 'Setting motor speeds to left=%f, right=%f.' % (left, right)
        self.messenger.send_cmd(self.commands.index('speedSet'), left, right)

    ####################### R E S P O N S E   C A L L B A C K S ###############
    def onError(self, received_command, *args, **kwargs):
        """Callback function to handle errors
        """
        self.statusMessage = 'Got unrecognized data from Arduino.'

    nresp = 0.
    def onSensorsResponse(self, received_command, *args, **kwargs):
        """Callback to handle the float addition response
        """
        s = self.sensorDataSize
        types = self.types
        try:
            byteString = args[0][-1]
            if len(byteString) >= s:
                arr = unpack(types, byteString[:s])
                self.nresp += 1
                #self.statusMessage =  "Response data:", arr
                
                # Save the data in our HDF5 file.
                data = np.empty((self.nfields,))
                assert len(arr) == self.nfields, (len(arr), self.nfields)
                data[:] = arr
                self.statusMessage = 'Saving data of shape %s to HDF5.' % (data.shape,)
                self.handler.enqueue(data)
                
#                 # Construct a sensor status message.
#                 m = ' '.join([
#                         '%s=%s' % (k, v)
#                         for (k, v) in zip(self.sensorFields, data)
#                     ])
#                 # Ensure each line of the sensor status message is close to 80 chars.
#                 statements = m.split()
#                 segments = []
#                 segmentSize = 79
#                 segment = ''
#                 for statement in statements:
#                     if len(segment) < segmentSize:
#                         segment += ' ' + statement
#                         m = m[segmentSize:]
#                     else:
#                         segments.append(segment)
#                         segment = statement
#                 if segment != '':
#                     segments.append(segment)
#                 self.statusMessage = '\n '.join(segments)
#                 self.statusMessage = ' '.join(['%s=%s' % (k, v) for (k, v) in zip(self.sensorFields[4:6], data[4:6])])
                
            
        except Exception as e:
            from traceback import format_exception
            from sys import exc_info
            tb = ''.join(['!! ' + l for l in format_exception(*exc_info())])
            self.statusMessage = "Failed with %s: %s" % (type(e), e) + '\n' + tb

    def logMessage(self, received_command, *args, **kwargs):
        '''Callback to log string messages sent by for debugging.'''
        logging.debug('Got message from Arduino: %s %s' % (args, kwargs))

        
if __name__ == '__main__':
    from time import sleep
    
    gunnarCommunicator = GunnarCommunicator()
    gunnarCommunicator.loopOnce()
