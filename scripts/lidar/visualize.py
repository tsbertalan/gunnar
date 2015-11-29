# Display Data from Neato LIDAR via socket server
# based on code from Nicolas "Xevel" Saugnier
# requires vpython

from threading import Thread
import logging

import numpy as np

from server import Server, QueueHandler

timeout = np.inf

offset = 140
init_level = 0
index = 0

lidarData = [[] for i in range(360)]  # A list of 360 elements Angle, Distance , quality

allData = []


from visual import *
# sample and intensity points
point = points(pos=[(0, 0, 0) for i in range(360)], size=5, color=(0 , 1, 0))
pointb = points(pos=[(0, 0, 0) for i in range(360)], size=5, color=(0.4, 0, 0))
point2 = points(pos=[(0, 0, 0) for i in range(360)], size=3, color=(1 , 1, 0))
point2b = points(pos=[(0, 0, 0) for i in range(360)], size=3, color=(0.4, 0.4, 0))
# lines
outer_line = curve (pos=[(0, 0, 0) for i in range(360)], size=5, color=(1 , 0, 0))
lines = [curve(pos=[(offset * cos(i * pi / 180.0), 0, offset * -sin(i * pi / 180.0)), (offset * cos(i * pi / 180.0), 0, offset * -sin(i * pi / 180.0))], color=[(0.1, 0.1, 0.2), (1, 0, 0)]) for i in range(360)]
zero_intensity_ring = ring(pos=(0, 0, 0), axis=(0, 1, 0), radius=offset - 1, thickness=1, color=color.yellow)

label_speed = label(pos=(0, -500, 0), xoffset=1, box=False, opacity=0.1)
label_errors = label(pos=(0, -1000, 0), xoffset=1, text="errors: 0", visible=False, box=False)

use_points = True
use_outer_line = False
use_lines = False
use_intensity = True


def update_view(angle, dist_mm, quality):
    """Updates the view of a sample.

Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived.
"""
#     logging.info("update_view(%s, %s, %s)" % (angle, dist_mm, quality))
    global offset, use_outer_line, use_line
    # unpack data using the denomination used during the discussions

    angle_rad = angle * math.pi / 180.0
    c = math.cos(angle_rad)
    s = -math.sin(angle_rad)

    lidarData[angle] = [dist_mm, quality]
    dist_x = dist_mm * c
    dist_y = dist_mm * s

    # reset the point display
    point.pos[angle] = vector(0, 0, 0)
    pointb.pos[angle] = vector(0, 0, 0)
    point2.pos[angle] = vector(0, 0, 0)
    point2b.pos[angle] = vector(0, 0, 0)
    if not use_lines : lines[angle].pos[1] = (offset * c, 0, offset * s)
    if not use_outer_line :
        outer_line.pos[angle] = (offset * c, 0, offset * s)
        outer_line.color[angle] = (0.1, 0.1, 0.2)

    # display the sample
    if use_lines : lines[angle].pos[1] = (dist_x, 0, dist_y)
    if use_outer_line : outer_line.pos[angle] = (dist_x, 0, dist_y)


def read_Lidar(handler):
    angles = range(360)
    while True:
        try:
            scan = handler.dequque()
            if isinstance(scan, np.ndarray) and scan.size > 0:
                for angle, distQual in zip(angles, scan):
                    if len(distQual) == 2:
                        dist_mm, quality = distQual
                        update_view(angle, dist_mm, quality)
        except KeyboardInterrupt:
            break


def checkKeys():
    global use_outer_line, use_lines, use_points, use_intensity
    if scene.kb.keys:  # event waiting to be processed?
        s = scene.kb.getkey()  # get keyboard info

        if s == "o":  # Toggle outer line
            use_outer_line = not use_outer_line
        elif s == "l":  # Toggle rays
            use_lines = not use_lines
        elif s == "p":  # Toggle points
            use_points = not use_points
        elif s == "i":  # Toggle intensity
            use_intensity = not use_intensity
            zero_intensity_ring.visible = use_intensity
        elif s == "j":  # Toggle rpm
            label_speed.visible = not label_speed.visible
        elif s == "k":  # Toggle errors
            label_errors.visible = not label_errors.visible


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)

    handler = QueueHandler()
    server = Server(handler)

    serverThread = Thread(target=server.serve)
    visualizationThread = Thread(target=read_Lidar, args=(handler,))
    serverThread.start()
    visualizationThread.start()

