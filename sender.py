import time
import socket
import struct
import shutil
from pathlib import Path

#NETWORK CONFIG
LAPTOP_IP = "192.168.1.157"
#Receiver laptop address (change as needed)

PORT = 5001
#Receiver listening port

#FOLDER CONFIG
BASE_DIR = Path.home() / "lidar_transfer"
#Root folder for all transfers

OUTGOING_DIR = BASE_DIR / "outgoing_ply"
#LiDAR drops new .ply files here (code lidar to auto drop in this folder)

SENT_DIR = BASE_DIR / "sent_ply"
#Archive for already-sent files

BASE_DIR.mkdir(parents=True, exist_ok=True)
#Ensure base folder exists

OUTGOING_DIR.mkdir(parents=True, exist_ok=True)
#Ensure outgoing folder exists

SENT_DIR.mkdir(parents=True, exist_ok=True)
#Ensure sent folder exists

# DEVICE IDENTIFICATION
DEVICE_ID = "LIDAR_01"
#Used to tag files from this sender (when we have multiple lidars sending scans to the same laptop)
#this helps avoid filename collisions and identify source

# TRANSFER TUNING
POLL_SECONDS = 1.0
#Folder scan interval
STABLE_SECONDS = 2.0
#Required no-change time before sending
RETRY_DELAY = 5.0
#Wait time between failed sends
CONNECT_TIMEOUT = 5.0
#Socket connect timeout
CHUNK_SIZE = 65536
#Bytes sent per socket write

def list_ply_files(folder: Path):
    #Finds all .ply files in folder
    return sorted([p for p in folder.iterdir() if p.is_file() and p.suffix.lower() == ".ply"])

def file_is_stable(path: Path, stable_seconds: float) -> bool:
    #Prevents sending partially-written files
    try:
        last_size = path.stat().st_size
    except FileNotFoundError:
        return False

    start = time.time()
    while time.time() - start < stable_seconds:
        time.sleep(0.25)
        try:
            new_size = path.stat().st_size
        except FileNotFoundError:
            return False

        if new_size != last_size:
            last_size = new_size
            start = time.time()

    try:
        with open(path, "rb"):
            pass
    except OSError:
        return False

    return True

def build_remote_filename(local_path: Path) -> bytes:
    #Avoids filename collisions across multiple devices
    ts = time.strftime("%Y%m%d_%H%M%S")
    return f"{DEVICE_ID}__{ts}__{local_path.name}".encode("utf-8")

def send_file(path: Path) -> None:
    #Implements sender-side transfer protocol
    file_size = path.stat().st_size
    remote_name = build_remote_filename(path)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(CONNECT_TIMEOUT)
        s.connect((LAPTOP_IP, PORT))

        s.sendall(struct.pack("!I", len(remote_name)))
        #Filename length header
        s.sendall(remote_name)
        #Filename payload
        s.sendall(struct.pack("!Q", file_size))
        #File size header

        with open(path, "rb") as f:
            while True:
                chunk = f.read(CHUNK_SIZE)
                if not chunk:
                    break
                s.sendall(chunk)
                #Raw file data stream

def move_to_sent(src: Path) -> Path:
    #Prevents re-sending same file
    dst = SENT_DIR / src.name
    base, suffix = dst.stem, dst.suffix
    i = 1

    while dst.exists():
        dst = SENT_DIR / f"{base}_{i}{suffix}"
        i += 1

    shutil.move(str(src), str(dst))
    return dst

def main():
    sent_signatures = set()
    #Tracks (path,size) pairs already sent

    while True:
        try:
            for ply in list_ply_files(OUTGOING_DIR):
                sig = (str(ply.resolve()), ply.stat().st_size)
                if sig in sent_signatures:
                    continue

                if not file_is_stable(ply, STABLE_SECONDS):
                    continue

                while True:
                    try:
                        send_file(ply)
                        break
                    except (OSError, socket.error):
                        time.sleep(RETRY_DELAY)

                sent_signatures.add(sig)
                move_to_sent(ply)

            time.sleep(POLL_SECONDS)

        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    main()
