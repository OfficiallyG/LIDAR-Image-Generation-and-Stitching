# DO NOT REDISTRIBUTE/SHARE WITHOUT WRITTEN APPROVAL
# (c) Onur Toker, Florida Polytechnic University, 2025
# DO NOT REDISTRIBUTE/SHARE WITHOUT WRITTEN APPROVAL

# NOTES: All units are in cm
import numpy as np
import matplotlib.pyplot as plt
import cv2
import myLivox360
from plyfile import PlyData, PlyElement

# A boolean function to filter/remove undesired points
filter_func = lambda x, y, z : ((z > 10) and (z < 100) and (x ** 2 + y ** 2 + z ** 2 < 500 ** 2))
filter_func = lambda x, y, z : True

# Create network lidar reader
nlr = myLivox360.network_lidar_reader(filter_func)

counter = 0
while True:
        # Get lidar data as a matrix
        ld = next(nlr)

        # PART1: Process lidar data
        # Obstacle detection calculations will be here

        # PART2: Navigational decision making
        # Based on the obstacle detection results, decide FORWARD, LEFT, RIGHT, etc.

        # PART3: Send decisions to motor controllers
        # Possibly send messages through UART to your microcontroller

        # Optional visualization

        img = myLivox360.lidar2img(ld)
        cv2.imshow('Live Lidar Image (Top View)', img)
        key = cv2.waitKey(1)
        
        if key == ord('q'):
            break
            
        if key == ord('s'):
            matrix = ld[:,:3]
            vertices = np.array([tuple(row) for row in matrix.tolist()],
                                dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
            vertex_element = PlyElement.describe(vertices, 'vertex')
            ply_data = PlyData([vertex_element], text=True) # Set text=False for binary
            ply_data.write(f"output{counter:06d}.ply")
            print("PLY file 'created successfully.")
            counter += 1
            
