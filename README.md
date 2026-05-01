# LIDAR-Image-Generation-and-Stitching
This Project was developed for the Polk County Sherrifs Department via the Capstone Design course at Florida Polytechnic University.


## **WARNING**

This project was made with the **Livox Mid360** in mind and may not work with other lidar scanning devices.

This project also utilizes a **Raspberry Pi 5**, flashed with PiOS on 64-bit Bookworm

## Files in this repo:

- **interface.py**
- **myLivox360.py**
- **sender.py**
- **test_lidar.py**
- **senderconfig.ini**
- **requirements.txt**
- **setup.md**

## Recieving laptop files: 
- **interface.py**

## Raspberry pi Files: 
- **myLivox360.py**
- **test_lidar.py** 
- **sender.py**
- **senderconfig.ini**

## File Descriptions

### interface.py: 
This file is the window that is opened on the users laptop. This interfcae is what lets the user view the .ply file and edit/stitch the files. It is the whole UI and the only file that needs to be on the users computer/laptop.

### myLivox360.py: 
This file reads the output from the LiDAR scanner and creates the image that will be shown in test_lidar.py. You can edit the point density and distance with udp and max_distance values.

### sender.py: 
This file lets the Raspberry Pi send the resulting .ply file from the device to the receiving laptop.

### senderconfig.ini
This file is the configuration file for the sender.py IP address, port, and device number.

### test_lidar.py: 
This file is what displays the live feed from the lidar and allows the user to send a snapshot of what is currently scanned. This is then saved as a .ply file in the outgoing files folder and sent to the laptop using sender.py.


## Instructions
- Clone the repository to Raspberry pi and laptop

```bash
git clone https://github.com/OfficiallyG/LIDAR-Image-Generation-and-Stitching 
```

### Raspberry Pi setup

- Refer to setup.md for Raspberry Pi setup instructions.

### Laptop Setup
1. Make sure Raspberry Pi device and laptop are on the same network.

2. Navigate to the directory you cloned the repository to.

3. Create a virtual environment if not already created, and start the virtual environment.

4. Uncomment and install the requirements outlined in the **Laptop** section of requirements.txt.
```bash
pip install -r requirements.txt
```

5. Run the interface.py file.

```bash
python interface.py
```

