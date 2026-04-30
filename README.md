# LIDAR-Image-Generation-and-Stitching
This Project was developed for the Polk County Sherrifs Department via the Capstone Design course at Florida Polytechnic University.

This project has 5 files in total all of which use the Python programming language.

## **WARNING**

This project was made with the **Livox Mid360** in mind and may not work with other lidar scanning devices.

## Files in this repo:

- **interface.py**
- **myLivox360.py**
- **sender.py**
- **test_lidar.py**

## Recieving laptop files: 
- interface.py

## Raspberry pi Files: 
- mylivox360.py 
- test_lidar.py 
- sender.py

## File Descriptions

### interface.py: 
This file is the window that is opened on the users laptop. This interfcae is what lets the user view the .ply file and edit/stitch the files. It is the whole UI and the only file that needs to be on the users computer/laptop.

### myLivox360.py: 
This file reads the output from the LiDAR scanner and creates the image that will be shown in test_lidar.py. You can edit the point density and distance with udp and max_distance values.

### sender.py: 
This file lets the Raspberry Pi send the resulting .ply file from the device to the receiving laptop.

### test_lidar.py: 
This file is what displays the live feed from the lidar and allows the user to send a snapshot of what is currently scanned. This is then saved as a .ply file in the outgoing files folder and sent to the laptop using sender.py.


## Instructions
Clone the repository to Raspberry pi and laptop

```bash
git clone https://github.com/OfficiallyG/LIDAR-Image-Generation-and-Stitching 
```



### Raspberry Pi setup

1. Clone the [Livox SDK](github.com/Livox-SDK/Livox-SDK2) and the [Input Remapper](https://github.com/sezanzeb/input-remapper) repositories to the device.

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
```

```bash
git clone https://github.com/sezanzeb/input-remapper
```

2. Follow the instructions on the Livox SDK repo to make sure lidar device is working.
    - you may need to edit the IP address in myLivox360.py if your device doesn't match the ip in the file.

**For Livox Mid360, the default device IP address is usually 192.168.1.1XX, XX being the last 2 digits of the serial number of the device.**






### Laptop Setup
1. Make sure Raspberry Pi device and laptop are on the same network


