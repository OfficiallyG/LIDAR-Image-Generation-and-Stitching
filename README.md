# LIDAR-Image-Generation-and-Stitching
LIDAR Image Generation and Stitching project
This Project is for the Polk County Sherrifs Department via the Capstone course at Florida Polytechnic University.
This project has 5 files in total all of which are coded in Python.
Files:
1. interface.py
2. myLivox360.py
3. sender.py
4. test_lidar.py
5. stitch.py
The files that go on the users laptop are: Interface.py, //stitch.py, 
The files that go in the Raspberry pi are: mylovox.py, test_lidar.py, sender.py
interface.py: this file is the window that is opened on the users laptop. this interfcae is what lets the user view the .ply file and edit/stitch the files. It is the whole UI and the only file that needs to be on the users computer/laptop.
myLivox360.py: this file reads the output from the LiDAR scanner and displays a live visual of the generated point cloud.
sender.py: this file lets the raspberry pi send the resulting .ply file from the lidar to the users laptop.
test_lidar: this file is what makes the live display from myLivox360.py stop and send a snapshot of what is currently scanned. this is then saved as a .ply file in the outgoing files folder.
//stitch.py: this file ...

edit the point density and distance with udp and max_distance in mylivox360.
