# DO NOT REDISTRIBUTE/SHARE WITHOUT WRITTEN APPROVAL
# (c) Onur Toker, Florida Polytechnic University, 2025
# DO NOT REDISTRIBUTE/SHARE WITHOUT WRITTEN APPROVAL

import math
import numpy as np
import socket
import matplotlib.pyplot as plt
import cv2

img_height, img_width, dist_max = 500, 500, 300 # 500x500 pixels, 300 cm max distance
lidar_port_number = 56301
frame_udp_count = 209

def network_lidar_reader(filter_func = lambda x, y, z: True):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = ('0.0.0.0', lidar_port_number)
    sock.bind(server_address)

    udp_pktnum = frame_udp_count
    ld = np.empty((udp_pktnum * 96, 4))

    while True:
        packets = []
        for p in range(udp_pktnum):
            data, address = sock.recvfrom(1380)
            packets.append(data)
            if len(data) < 1380:
                raise Exception("Lidar I/O error")

        row = 0
        for k in range(udp_pktnum):
            data = packets[k]
            # if (k == 0):
            #     t = int.from_bytes(data[28:36], byteorder='little', signed=False) / 1e9
            for l in range(36, 36 + 96 * 14, 14):
                x = int.from_bytes(data[l + 0:l + 4], byteorder='little', signed=True) / 10
                y = int.from_bytes(data[l + 4:l + 8], byteorder='little', signed=True) / 10
                z = int.from_bytes(data[l + 8:l + 12], byteorder='little', signed=True) / 10
                r = int.from_bytes(data[l + 12:l + 14], byteorder='little', signed=False)
                if filter_func(x, y, z):
                    ld[row, :] = [x, y, z, r]
                    row += 1

        yield ld


def file_lidar_reader(filename):
    raise NotImplementedError("Not implemented")


def lidar2img(ld):
    # CONSTRUCT IMAGE
    xyz = ld[:, [0, 1, 2]]
    img = np.zeros((img_height, img_width, 3), np.uint8)

    rows = (np.round(np.interp(xyz[:,1], [-dist_max, dist_max], [img_height - 1, 0]))).astype(np.int32)
    cols = (np.round(np.interp(xyz[:,0], [-dist_max, dist_max], [0, img_width - 1]))).astype(np.int32)
    img[rows, cols, :] = (255, 255, 255)

    return img
