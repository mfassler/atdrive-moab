#!/usr/bin/env python3
  
import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import socket
import struct
import numpy as np
import cv2

from misc_utils import get_last_packet
from misc_map_tools import make_map


class LidarWithVisual:
    def __init__(self, *args, **kwargs):
        #super(LidarWithVisual, self).__init__(*args, **kwargs)
        self._a_map = make_map(800, 800, 100)
        self.amap = np.copy(self._a_map)
        self.prev_angle = 0.0


    def parse_pc_packet(self, angle_start, angle_end, payload):
        rad_start = np.radians(angle_start / 128.0)
        rad_end = np.radians(angle_end / 128.0)
        if rad_end < rad_start:
            rad_end += 2*np.pi

        ranges = np.frombuffer(payload, np.uint16) / 1000.0  # in meters
        angleSpace = np.linspace(rad_start, rad_end, len(ranges))

        for i,theta in enumerate(angleSpace):
            r = ranges[i]
            x = r * np.sin(theta)
            y = r * np.cos(theta)

            x_pixel = int(round(-x*100 + 400))
            y_pixel = int(round(y*100 + 400))

            if x_pixel > 1 and x_pixel < 798:
                if y_pixel > 1 and y_pixel < 798:
                    self.amap[y_pixel, x_pixel] = 0,0,0

            if angle_start < self.prev_angle:
                cv2.imshow('asdf', self.amap)
                cv2.waitKey(1)
                self.amap = np.copy(self._a_map)

            self.prev_angle = angle_start


lidarVis = LidarWithVisual()

RX_PORT = 27118
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", RX_PORT))


while True:
    pkt, addr = sock.recvfrom(1500)

    try:
        fsa, lsa = struct.unpack('<HH', pkt[:4])
        payload = pkt[4:]

    except:
        print('failed to parse packet')

    else:
        lidarVis.parse_pc_packet(fsa, lsa, payload)



