'''
Created on Nov 29, 2015

@author: tsbertalan
'''

import time, logging
from collections import deque
from time import sleep

import numpy as np


def checksum(data):
    """Compute and return the checksum as an int.

    data -- list of 20 bytes (as ints), in the order they arrived.
    """
    # group the data by word, little-endian
    data_list = []
    for t in range(10):
        data_list.append(data[2 * t] + (data[2 * t + 1] << 8))

    # compute the checksum on 32 bits
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d

    # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    checksum = (chk32 & 0x7FFF) + (chk32 >> 15)  # wrap around to fit into 15 bits
    checksum = checksum & 0x7FFF  # truncate to 15 bits
    return int(checksum)


def compute_speed(data):
    speed_rpm = float(data[0] | (data[1] << 8)) / 64.0
    return speed_rpm


class CharStream(object):
    
    def getChar(self, numChars=1):
        from gunnar.utils import VirtualClassError
        raise VirtualClassError(self)


class LidarParser:

    def __init__(self, server, exitTimeCallback):
        assert isinstance(server, CharStream)
        self.lidarData = [[]] * 360  # A list of 360 elements Angle, Distance , quality
        self.dataArrs = deque()
        self.init_level = 0
        self.server = server
        self.exitTimeCallback = exitTimeCallback

    def __len__(self):
        return len(self.dataArrs)

    def pop(self, maxAttempts=100):
        for i in range(maxAttempts):
            if len(self.dataArrs) > 0:
                break
            else:
                logging.debug("Parser has no data yet (attempt %d of %d)" % (i, maxAttempts))
                sleep(1)  # Block this thread until we have data.
        return self.dataArrs.popleft()

    def savePacketQuarter(self, angle, data):
        """Save a sample.

        Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived.
        """
        logging.debug("Saving quarter-packet of sum %s at angle %s." % (sum(data), angle))
        # unpack data using the denomination used during the discussions
        x = data[0]
        x1 = data[1]
        x2 = data[2]
        x3 = data[3]

        dist_mm = x | ((x1 & 0x3f) << 8)  # distance is coded on 13 bits ? 14 bits ?
        quality = x2 | (x3 << 8)  # quality is on 16 bits
        self.lidarData[angle] = [dist_mm, quality]

    def saveScan(self):
        try:
            ragged = False
            for angle in self.lidarData:
                if len(angle) != 2:
                    ragged = True
                    break
            if not ragged:
                dataArr = np.vstack(self.lidarData)
                if dataArr.size > 0:
                    self.dataArrs.append(dataArr)
        except ValueError as e:  # Data is likely ragged (not all filled).
            logging.debug("Got a ValueError.")
            raise e

    def parse(self):
        while True:
            logging.debug("self.exitTimeCallback() gives %s" % self.exitTimeCallback())
            if self.exitTimeCallback():
                logging.debug("Breaking from parser.parse().")
                break
            time.sleep(0.00001)  # do not hog the processor power
            nb_errors = 0

            self.init_level

            if self.init_level == 0:  # We're searching for a start byte.
                logging.debug("Looking for a start byte now.")
                char = self.server.getChar()
                if char is None:
                    continue
                b = ord(char)
                # start byte
                if b == 0xFA:
                    logging.debug("Got a start byte, 0xFA!")
                    self.init_level = 1
                else:
                    self.init_level = 0
            elif self.init_level == 1:  # We found a start byte; now we're parsing.
                logging.debug("Looking for an index byte now.")
                # position index
                b = ord(self.server.getChar(1))
                if b >= 0xA0 and b <= 0xF9:  # The angle indices range from 160 to 249
                    if b == 0xA0:  # We're at the first index; save the previous scan.
                        self.saveScan()
                    index = b - 0xA0
                    logging.debug("At index %d." % index)
                    self.init_level = 2
                elif b != 0xFA:  # If the next byte after the start byte isn't in the index range, go back to searching for a start byte.
                    self.init_level = 0
            elif self.init_level == 2:
                logging.debug("Getting two speed bytes.")
                # speed
                b_speed = [ ord(b) for b in self.server.getChar(2)]

                # data
                #
                # (Data description from
                # https://xv11hacking.wikispaces.com/LIDAR+Sensor .)
                #
                # Each packet is organized as follows:
                # <start> <index> <speed_L> <speed_H>
                # [Data 0] [Data 1] [Data 2] [Data 3] <checksum_L> <checksum_H>
                #
                # where:
                #
                # start is always 0xFA=250
                # index is the index byte in the 90 packets, going from 0xA0
                # (packet 0, readings 0 to 3) to 0xF9 (packet 89, readings 356 to 359).
                # speed is a two-byte information, little-endian. It represents
                # the speed, in 64th of RPM (aka value in RPM represented in
                # fixed point, with 6 bits used for the decimal part).
                # [Data 0] to [Data 3] are the 4 readings. Each one is 4 bytes
                # long, and organized as follows :
                #
                # `byte 0 : <distance 7:0>`
                # `byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>`
                # `byte 2 : <signal strength 7:0>`
                # `byte 3 : <signal strength 15:8>`

                logging.debug("Getting four data0 bytes.")
                b_data0 = [ ord(b) for b in self.server.getChar(4)]
                logging.debug("Getting four data1 bytes.")
                b_data1 = [ ord(b) for b in self.server.getChar(4)]
                logging.debug("Getting four data2 bytes.")
                b_data2 = [ ord(b) for b in self.server.getChar(4)]
                logging.debug("Getting four data3 bytes.")
                b_data3 = [ ord(b) for b in self.server.getChar(4)]

                # for the checksum, we need all the data of the packet...
                # this could be collected in a more elegent fashion...
                all_data = [ 0xFA, index + 0xA0 ] + b_speed + b_data0 + b_data1 + b_data2 + b_data3

                # checksum
                logging.debug("Getting two checksum bytes.")
                b_checksum = [ ord(b) for b in self.server.getChar(2) ]
                incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)
                logging.debug("incoming_checksum is %s" % incoming_checksum)

                # verify that the received checksum is equal to the one computed from the data
                if checksum(all_data) == incoming_checksum:
                    # speed_rpm = compute_speed(b_speed)
                    self.savePacketQuarter(index * 4 + 0, b_data0)
                    self.savePacketQuarter(index * 4 + 1, b_data1)
                    self.savePacketQuarter(index * 4 + 2, b_data2)
                    self.savePacketQuarter(index * 4 + 3, b_data3)
                else:
                    # the checksum does not match, something went wrong...
                    nb_errors += 1
                    logging.debug("Checksum errors!")

                    # "saveScan" data in error state.
                    self.savePacketQuarter(index * 4 + 0, [0, 0x80, 0, 0])
                    self.savePacketQuarter(index * 4 + 1, [0, 0x80, 0, 0])
                    self.savePacketQuarter(index * 4 + 2, [0, 0x80, 0, 0])
                    self.savePacketQuarter(index * 4 + 3, [0, 0x80, 0, 0])
                self.init_level = 0  # reset and wait for the next packet

            else:  # default, should never happen...
                self.init_level = 0
