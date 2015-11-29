# Display Data from Neato LIDAR
# based on code from Nicolas "Xevel" Saugnier
# requires pyserial


import sys, math
import serial
import time

import numpy as np

from client import Client

com_port = "/dev/ttyUSB0"  # example: 5 == "COM6" == "/dev/tty5"
baudrate = 115200

offset = 140
init_level = 0
index = 0

lidarData = [[] for i in range(360)]  # A list of 360 elements Angle, Distance , quality

allData = []

use_points = True
use_outer_line = False
use_lines = False
use_intensity = True

def update_view(angle, data):
    """Updates the view of a sample.

Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived.
"""
    global offset, use_outer_line, use_line
    # unpack data using the denomination used during the discussions
    x = data[0]
    x1 = data[1]
    x2 = data[2]
    x3 = data[3]

    angle_rad = angle * math.pi / 180.0
    c = math.cos(angle_rad)
    s = -math.sin(angle_rad)

    dist_mm = x | ((x1 & 0x3f) << 8)  # distance is coded on 13 bits ? 14 bits ?
    quality = x2 | (x3 << 8)  # quality is on 16 bits
    lidarData[angle] = [dist_mm, quality]
    dist_x = dist_mm * c
    dist_y = dist_mm * s


def checksum(data):
    """Compute and return the checksum as an int.

data -- list of 20 bytes (as ints), in the order they arrived in.
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


def read_Lidar(hostname, port, SEND=True):
    client = Client(hostname, port, timeout=10)
    print "Connected to %s:%d. Will begin sending data. Ctrl+C to quit." % (hostname, port)
    time.sleep(1)
    global init_level, angle, index

    nb_errors = 0
    while True:
        try:
            time.sleep(0.00001)  # do not hog the processor power

            if init_level == 0 :
                b = ord(ser.read(1))
                # start byte
                if b == 0xFA :
                    init_level = 1
                    try:
                        dataArr = np.vstack(lidarData)
                        if dataArr.size > 0:
                            allData.append(lidarData)
                            if SEND:
                                client.send(dataArr)
                    except ValueError:  # Data is likely ragged (not all filled).
                        pass
                else:
                    init_level = 0
            elif init_level == 1:
                # position index
                b = ord(ser.read(1))
                if b >= 0xA0 and b <= 0xF9 :
                    index = b - 0xA0
                    init_level = 2
                elif b != 0xFA:
                    init_level = 0
            elif init_level == 2 :
                # speed
                b_speed = [ ord(b) for b in ser.read(2)]

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
                # start is always 0xFA
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

                b_data0 = [ ord(b) for b in ser.read(4)]
                b_data1 = [ ord(b) for b in ser.read(4)]
                b_data2 = [ ord(b) for b in ser.read(4)]
                b_data3 = [ ord(b) for b in ser.read(4)]

                # for the checksum, we need all the data of the packet...
                # this could be collected in a more elegent fashion...
                all_data = [ 0xFA, index + 0xA0 ] + b_speed + b_data0 + b_data1 + b_data2 + b_data3

                # checksum
                b_checksum = [ ord(b) for b in ser.read(2) ]
                incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)

                # verify that the received checksum is equal to the one computed from the data
                if checksum(all_data) == incoming_checksum:
                    speed_rpm = compute_speed(b_speed)

                    update_view(index * 4 + 0, b_data0)
                    update_view(index * 4 + 1, b_data1)
                    update_view(index * 4 + 2, b_data2)
                    update_view(index * 4 + 3, b_data3)
                else:
                    # the checksum does not match, something went wrong...
                    nb_errors += 1

                    # display the samples in an error state
                    update_view(index * 4 + 0, [0, 0x80, 0, 0])
                    update_view(index * 4 + 1, [0, 0x80, 0, 0])
                    update_view(index * 4 + 2, [0, 0x80, 0, 0])
                    update_view(index * 4 + 3, [0, 0x80, 0, 0])

                init_level = 0  # reset and wait for the next packet

            else:  # default, should never happen...
                init_level = 0
        except KeyboardInterrupt:
            break


ser = serial.Serial(com_port, baudrate)

hostname = 'localhost'
port = 9009
SEND = False
if len(sys.argv) > 1:
    print sys.argv
    hostname = sys.argv[1]
    if len(sys.argv) > 2:
        port = int(sys.argv[2])
        if len(sys.argv) > 3:
            SEND = 'do' in sys.argv[3]

read_Lidar(hostname, port, SEND=SEND)
print
print "Saving data..."
allData = np.array(allData)
np.savez("lidar.npz", data=allData)
print "all the data is of shape %s" % (allData.shape,)

