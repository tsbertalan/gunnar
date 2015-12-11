#!/usr/bin/python
# receive.py
# Author: Adrien Emery
# Make sure the you have the Receive example loaded onto the Arduino

import sys
import serial
import time

from CmdMessenger import CmdMessenger
from serial.tools import list_ports


class Receive(object):

    def __init__(self):
        self.running = False
        self.led_state = False
        # make sure this baudrate matches the baudrate on the Arduino
        self.baud = 115200
        self.commands = ['set_led']

        try:
            # gets the first available USB port
            self.port_name = self.list_usb_ports()[1][0]
            self.serial_port = serial.Serial(self.port_name, self.baud, timeout=0)
        except (serial.SerialException, IndexError):
            raise SystemExit('Could not open serial port.')
        else:
            print 'Serial port %s sucessfully opened.' % self.port_name
            self.messenger = CmdMessenger(self.serial_port)

    def list_usb_ports(self):
        """Use the grep generator to get a list of all USB ports.
        """
        ports =  [port for port in list_ports.grep('usb')]
        print "Found ports %s." % [p[0] for p in ports]
        return ports

    def stop(self):
        """Stops the main run loop
        """
        self.running = False

    def run(self):
        """Main loop to send data to the Arduino
        """
        self.running = True
        timeout = 1
        t0 = time.time()
        while self.running:
            # Update the led state once every second
            if time.time() - t0 > timeout:
                t0 = time.time()
                if self.led_state == True:
                    self.messenger.send_cmd(self.commands.index('set_led'), False)
                    self.led_state = False
                else:
                    self.messenger.send_cmd(self.commands.index('set_led'), True)
                    self.led_state = True


if __name__ == '__main__':
    receive = Receive()

    try:
        print 'Press Ctrl+C to exit...'
        receive.run()
    except KeyboardInterrupt:
        receive.stop()
        print 'Exiting...'
