# Display Data from Neato LIDAR via socket server
# based on code from Nicolas "Xevel" Saugnier
# requires vpython

from threading import Thread  # TODO: Use processes to make the GUI more fluid.
import logging

import numpy as np
from visual import *

from server import Server, QueueHandler


class Visualizer:

    def __init__(self, handler):
        self.lidarData = [[] for i in range(360)]  # A list of 360 elements Angle, Distance , quality
        offset = self.offset = 140
        init_level = 0
        index = 0

        # sample and intensity points
        self.point = points(pos=[(0, 0, 0) for i in range(360)], size=5, color=(0 , 1, 0))
        self.pointb = points(pos=[(0, 0, 0) for i in range(360)], size=5, color=(0.4, 0, 0))
        self.point2 = points(pos=[(0, 0, 0) for i in range(360)], size=3, color=(1 , 1, 0))
        self.point2b = points(pos=[(0, 0, 0) for i in range(360)], size=3, color=(0.4, 0.4, 0))

        # lines
        self.outer_line = curve (pos=[(0, 0, 0) for i in range(360)], size=5, color=(1 , 0, 0))
        self.lines = [curve(pos=[(offset * cos(i * pi / 180.0), 0, offset * -sin(i * pi / 180.0)), (offset * cos(i * pi / 180.0), 0, offset * -sin(i * pi / 180.0))], color=[(0.1, 0.1, 0.2), (1, 0, 0)]) for i in range(360)]
        self.zero_intensity_ring = ring(pos=(0, 0, 0), axis=(0, 1, 0), radius=offset - 1, thickness=1, color=color.yellow)

        self.label_speed = label(pos=(0, -500, 0), xoffset=1, box=False, opacity=0.1)
        self.label_errors = label(pos=(0, -1000, 0), xoffset=1, text="errors: 0", visible=False, box=False)

        self.use_points = True
        self.use_outer_line = True
        self.use_lines = True
        self.use_intensity = True

        self.scans = []  # Save all the gathered scans.


    def update_view(self, angle, dist_mm, quality):
        """Updates the view of a sample.

    Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived.
    """
    #     logging.info("update_view(%s, %s, %s)" % (angle, dist_mm, quality))
        # unpack data using the denomination used during the discussions

        angle_rad = angle * math.pi / 180.0
        c = math.cos(angle_rad)
        s = -math.sin(angle_rad)

        self.lidarData[angle] = [dist_mm, quality]
        dist_x = dist_mm * c
        dist_y = dist_mm * s

        # reset the point display
        self.point.pos[angle] = vector(0, 0, 0)
        self.pointb.pos[angle] = vector(0, 0, 0)
        self.point2.pos[angle] = vector(0, 0, 0)
        self.point2b.pos[angle] = vector(0, 0, 0)
        if not self.use_lines : self.lines[angle].pos[1] = (self.offset * c, 0, self.offset * s)
        if not self.use_outer_line :
            self.outer_line.pos[angle] = (self.offset * c, 0, self.offset * s)
            self.outer_line.color[angle] = (0.1, 0.1, 0.2)

        # display the sample
        if self.use_lines : self.lines[angle].pos[1] = (dist_x, 0, dist_y)
        if self.use_outer_line : self.outer_line.pos[angle] = (dist_x, 0, dist_y)


    def read_Lidar(self):
        angles = range(360)
        while True:
            try:
                scan = handler.dequque()
                if isinstance(scan, np.ndarray) and scan.size > 0:
                    goodScan = False
                    for angle, distQual in zip(angles, scan):
                        if len(distQual) == 2:
                            goodScan = True
                            dist_mm, quality = distQual
                            self.update_view(angle, dist_mm, quality)
                    if goodScan:
                        self.scans.append(scan)
                self.checkKeys()
            except KeyboardInterrupt:
                break
        fname = "visualizedData.npz"
        logging.info("saving %s" % fname)
        np.savez_compressed(fname, data=np.vstack(self.scans))

    def checkKeys(self):
        if scene.kb.keys:  # event waiting to be processed?
            s = scene.kb.getkey()  # get keyboard info

            if s == "o":  # Toggle outer line
                use_outer_line = not self.use_outer_line
            elif s == "l":  # Toggle rays
                use_lines = not self.use_lines
            elif s == "p":  # Toggle points
                use_points = not self.use_points
            elif s == "i":  # Toggle intensity
                use_intensity = not self.use_intensity
                self.zero_intensity_ring.visible = use_intensity
            elif s == "j":  # Toggle rpm
                self.label_speed.visible = not self.label_speed.visible
            elif s == "k":  # Toggle errors
                self.label_errors.visible = not self.label_errors.visible


if __name__ == "__main__":
    logging.getLogger().setLevel(logging.DEBUG)

    handler = QueueHandler()
    server = Server(handler)
    visualizer = Visualizer(handler)

    serverThread = Thread(target=server.serve)
    visualizationThread = Thread(target=visualizer.read_Lidar)
    serverThread.start()
    visualizationThread.start()

