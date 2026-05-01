# Raspberry Pi Setup Guide — Livox Mid360 LiDAR

Complete, start-to-finish workflow for configuring a fresh **Raspberry Pi OS Bookworm (64-bit)** as the LiDAR sender node. 

---

## 1 - Operating System Installation

Start with a clean 64-bit installation of the official Raspberry Pi OS Bookworm.

1. Download the **Raspberry Pi Imager** to your computer.
2. Select **Raspberry Pi OS (64-bit)** (the standard version with the desktop environment).
3. Flash it to your microSD card.
4. Insert the SD card into the Pi, connect it to a monitor, boot it up, and connect to Wi-Fi.

> **Expected output:** The standard Raspberry Pi desktop environment. Open a terminal (`Ctrl+Alt+T`) to begin.

---

## 2 - Dependencies & Repositories

Updates the package manager and installs all necessary system compilers, Git, Python environment tools, and the underlying GUI libraries required for OpenCV to render windows. Then creates a workspace and clones the three required repositories.

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-pip python3-setuptools python3-venv python3-tk libgtk2.0-dev pkg-config gettext debhelper
mkdir -p ~/capstone
cd ~/capstone
git clone https://github.com/OfficiallyG/LIDAR-Image-Generation-and-Stitching.git
git clone https://github.com/Livox-SDK/Livox-SDK2.git
git clone https://github.com/sezanzeb/input-remapper.git
```

> **Expected output:** Long scrolling lists of packages downloading and installing, followed by Git progress bars showing 100% for the three cloned repositories.

---

## 3 - Build & Install Input Remapper

Because Bookworm OS restricts global Python installations, input-remapper must be compiled into a native Debian `.deb` package directly from its source code. After building it, any broken older versions are purged and the custom package is installed.

```bash
cd ~/capstone/input-remapper
./scripts/build-deb.sh
sudo apt purge -y input-remapper input-remapper-daemon input-remapper-gtk python3-inputremapper
sudo apt install -y -f ./dist/input-remapper-*.deb
sudo systemctl enable input-remapper
sudo systemctl start input-remapper
```

> **Expected output:** Debian packaging logs from the build script, followed by the `.deb` extraction. The `systemctl enable` command will output a `Created symlink /etc/systemd/system/...` message confirming the service is registered with the OS.

---

## 4 - Compile the Livox SDK2

Prepares the build environment and compiles the core C++ libraries. The `-Wno-error` flag bypasses strict compiler warnings that would otherwise halt the build on Bookworm. Libraries are then installed system-wide.

```bash
cd ~/capstone/Livox-SDK2
mkdir build
cd build
cmake -DCMAKE_CXX_FLAGS="-Wno-error" ..
make -j4
sudo make install
```

> **Expected output:** Green percentage indicators climbing line-by-line until `[100%] Built target livox_lidar_sdk_shared`. The `make install` command will output `Install the project...` and show files being copied into `/usr/local/lib/` and `/usr/local/include/`.

---

## 5 - Compile the Capstone C++ Architecture

Builds the main capstone repository's C++ components so they link against the freshly installed Livox SDK.

```bash
cd ~/capstone/LIDAR-Image-Generation-and-Stitching
mkdir build
cd build
cmake ..
make -j4
```

> **Expected output:** CMake configuration logs verifying compiler settings, followed by `[100%] Built target ...` confirming the project compiled cleanly.

---

## 6 - Network Configuration

Forces the Pi's ethernet port (`eth0`) onto the `192.168.1.x` subnet so it can communicate directly with the Livox Mid360, which defaults to `192.168.1.162`. The multicast routing rule allows Python to intercept the LiDAR's UDP data stream.

```bash
sudo nmcli con add con-name "Livox" ifname eth0 type ethernet ipv4.method manual ipv4.addresses 192.168.1.50/24
sudo nmcli con up "Livox"
sudo ip route add 224.0.0.0/4 dev eth0
ip a show eth0
ping -c 4 192.168.1.162
```

> **Expected output:** NetworkManager outputs `Connection successfully activated`. The `ip a show eth0` command displays `inet 192.168.1.50/24`. The ping shows 4 lines of data returning with millisecond times and `0% packet loss`.

---

## 7 - Python Virtual Environment & Installation

Creates an isolated Python environment to bypass Bookworm's `externally-managed-environment` restriction, then installs project dependencies.

**Before running:** Confirm that `requirements.txt` has `opencv-python>=4.5.0` uncommented under the Raspberry Pi section, and that all laptop-specific packages (`PyQt6`, `open3d`, etc.) remain commented out.

```bash
cd ~/capstone/LIDAR-Image-Generation-and-Stitching
python3 -m venv venv
source venv/bin/activate
pip install --default-timeout=1000 -r requirements.txt
```

> **Expected output:** The terminal prompt changes to `(venv) lidar@raspberrypi:...`. Pip shows download progress bars ending with `Successfully installed contourpy-... matplotlib-... numpy-... opencv-python-4.x.x`.

---

## Configuration & Final Execution

### 8 — Set the laptop destination IP

Edit `senderConfig.ini` to point the Pi at your receiving laptop:

```bash
nano senderConfig.ini
```

Update `LAPTOP_IP` to your laptop's IP address and save (`Ctrl+O`, `Enter`, `Ctrl+X`).

```ini
[NETWORK]
LAPTOP_IP = <your-laptop-ip>   # e.g. 192.168.1.100
PORT = 5001

[DEVICE]
DEVICE_NUM = 1
```

### 9 — Verify the Pi interface IP (if changed)

`myLivox360.py` hardcodes the Pi's `eth0` IP at line 25 for the multicast subscription. If you used a different address in Phase 5, update it to match:

```python
iface_ip = socket.inet_aton("192.168.1.50")  # must match the eth0 IP set in Phase 5
```

### 10 — Run the LiDAR stream

```bash
python3 test_lidar.py
```

> **Expected output:** A new desktop window titled **Live Lidar Image (Top View)** opens, rendering the real-time point-cloud scan from the sensor.

### Keyboard shortcuts while running

| Key | Action |
|-----|--------|
| `s` | Capture a snapshot, save as `.ply`, and send to laptop |
| `i` | Open the GUI to change the laptop IP, port, or device number at runtime |
| `q` | Quit |

---

## Quick Reference

| Setting | Value |
|---------|-------|
| Pi ethernet IP | `192.168.1.50` |
| LiDAR IP (Mid360 default) | `192.168.1.162` |
| Multicast group | `224.1.1.5` |
| LiDAR UDP port | `56301` |
| Transfer TCP port | `5001` |
| Outgoing `.ply` folder | `~/lidar_transfer/outgoing_ply/` |
| Sent archive folder | `~/lidar_transfer/sent_ply/` |
