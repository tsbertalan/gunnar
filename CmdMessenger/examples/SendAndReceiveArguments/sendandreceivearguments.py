#!/usr/bin/python
# sendandreceivearguments.py
# Author: Adrien Emery
# Make sure the you have the SendAndReceiveArguments example loaded onto the Arduino
import random
import sys
import serial
import time

from CmdMessenger import CmdMessenger
from serial.tools import list_ports


class SendAndReceiveArguments(object):

    def __init__(self):
        # make sure this baudrate matches the baudrate on the Arduino
        self.running = False
        self.baud = 115200
        self.commands = ['acknowledge',
                         'error',
                         'float_addition',
                         'float_addition_result',
                         ]

        try:
            # try to open the first available usb port
            self.port_name = self.list_usb_ports()[1][0]
            self.serial_port = serial.Serial(self.port_name, self.baud, timeout=0)
        except (serial.SerialException, IndexError):
            raise SystemExit('Could not open serial port.')
        else:
            print 'Serial port %s sucessfully opened.' % self.port_name
            self.messenger = CmdMessenger(self.serial_port)
            # attach callbacks
            self.messenger.attach(func=self.on_error, msgid=self.commands.index('error'))
            self.messenger.attach(func=self.on_float_addition_result,
                                  msgid=self.commands.index('float_addition_result'))

            # send a command that the arduino will acknowledge
            self.messenger.send_cmd(self.commands.index('acknowledge'))
            # Wait until the arduino sends and acknowledgement back
            self.messenger.wait_for_ack(ackid=self.commands.index('acknowledge'))
            print 'Arduino Ready'

    def list_usb_ports(self):
        """ Use the grep generator to get a list of all USB ports.
        """
        ports =  [port for port in list_ports.grep('usb')]
        return ports

    def on_error(self, received_command, *args, **kwargs):
        """Callback function to handle errors
        """
        print 'Error:', args[0][0]

    def on_float_addition_result(self, received_command, *args, **kwargs):
        """Callback to handle the float addition response
        """
        print "Got cmd=%s, args=%s, kwargs=%s." % (received_command, args, kwargs)
        print 'Addition Result:', args[0][0]
        print 'Subtraction Result:', args[0][1]
        print

    def stop(self):
        self.running = False

    def run(self):
        """Main loop to send and receive data from the Arduino
        """
        self.running = True
        timeout = .01
        t0 = time.time()
        while self.running:
            # Send two random integers to be added/subtracted every timeout seconds
            if time.time() - t0 > timeout:
                lastT0 = t0
                t0 = time.time()
                print t0-lastT0, 'elapsed.'
                a = random.randint(0, 10)
                b = random.randint(0, 10)
                print 'Sending: {}, {}'.format(a, b)
                self.messenger.send_cmd(self.commands.index('float_addition'), a, b)

            # Check to see if any data has been received
            self.messenger.feed_in_data()


if __name__ == '__main__':
    send_and_receive_args = SendAndReceiveArguments()

    try:
        print 'Press Ctrl+C to exit...'
        print
        send_and_receive_args.run()
    except KeyboardInterrupt:
        send_and_receive_args.stop()
        print 'Exiting...'
