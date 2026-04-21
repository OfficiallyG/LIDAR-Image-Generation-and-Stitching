# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

A distributed LiDAR scanning system for the Polk County Sheriff's Department (Florida Polytechnic University capstone). A Raspberry Pi captures 3D point cloud scans from a Livox360 LiDAR scanner and transmits them over TCP to a laptop running a PyQt6 GUI for visualization and stitching.

## Python Version Requirement

**Python 3.12 is required on the laptop.** Open3D has no wheels for Python 3.13 on Windows (as of 2026). Install Python 3.12 from python.org — it installs side-by-side with any other version.

First-time setup (laptop):
```bash
py -3.12 -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

After that, just activate before each session: `venv\Scripts\activate`

## Running the Project

**Laptop (main GUI):**
```bash
venv\Scripts\activate   # if not already active
python interface.py
```

**Raspberry Pi (capture + send):**
```bash
python test_lidar.py    # Interactive capture: 's' = snapshot, 'i' = change IP, 'q' = quit
python sender.py        # File watcher daemon that transmits PLY files to the laptop
```

## Architecture

This is a **three-component distributed system**:

### 1. LiDAR Reader (`myLivox360.py`)
Receives UDP multicast from the Livox360 scanner (multicast group `224.1.1.5:56301`, bound to Pi's `eth0` at `192.168.10.50`). A full frame = 418 UDP packets × 96 points each. Returns a NumPy array of XYZ coordinates.

### 2. Raspberry Pi Side (`test_lidar.py` + `sender.py`)
- `test_lidar.py` captures snapshots on keypress and saves them as PLY files to `~/lidar_transfer/outgoing_ply/`
- `sender.py` watches that folder and sends new files to the laptop over TCP using a simple wire protocol: `[4B filename_len][filename][8B file_size][data]`
- Config in `senderConfig.ini` sets `LAPTOP_IP`, `PORT` (default 5001), and `DEVICE_NUM`

### 3. Laptop GUI (`interface.py`)
The main application (~2000 lines). Key subsystems:
- **TCP receiver thread**: listens on port 5001, saves incoming PLY files to `~/Desktop/LiDAR_Inbox/`
- **PLY loader**: custom ASCII/binary reader (not using plyfile) that handles XYZ + optional RGB/intensity fields
- **3D viewer**: PyQtGraph OpenGL widget with real-time point cloud rendering
- **Processing pipeline**: voxel downsampling → statistical outlier removal → intensity-to-color gradient mapping (viridis-style)
- **Stitching**: `stitch.py` places two clouds side-by-side with a configurable X-axis gap; `interface.py` also has manual stitch controls

## Key Technical Details

- The TCP protocol between sender and receiver is custom/binary — any changes must be mirrored in both `sender.py` and the receiver thread in `interface.py`
- `DEVICE_NUM` in `senderConfig.ini` identifies which Pi is sending; the GUI uses this for file naming
- Floor plane detection uses RANSAC via Open3D; camera view methods orient the viewer relative to the detected plane
- PLY files use the standard format but the loader in `interface.py` manually parses headers — adding new fields requires updating the header parser and the data reader together
