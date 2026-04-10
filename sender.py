#===== SECTION 1: IMPORTS START POINT =====
#import time for timestamps, polling loop timing, and stability checks
import time
#import socket for tcp connection to the receiver laptop
import socket
#import struct for packing protocol headers (filename length and file size)
import struct
#import shutil for moving files into the sent archive after successful transfer
import shutil
#import pathlib path for clean folder and file path handling
from pathlib import Path
#import configParser for reading config file
import configparser
#import tkinter for ip change gui
import tkinter as tk
#import messagebox for ip change verification
from tkinter import messagebox 
#===== SECTION 1: IMPORTS END POINT =====


#===== SECTION 2: NETWORK CONFIG START POINT =====
#load and read config file
CONFIG_FILE = "senderConfig.ini"

config = configparser.ConfigParser()
config.read(CONFIG_FILE)

#Set default laptop ip and port settings
LAPTOP_IP = config.get("NETWORK","LAPTOP_IP")
PORT = config.getint("NETWORK","PORT")

#set default device number and create default device ip
device_number = config.getint("DEVICE","DEVICE_NUM")
DEVICE_ID = f"LIDAR_{device_number:02d}"

#===== SECTION 2: NETWORK CONFIG END POINT =====


#===== SECTION 3: FOLDER CONFIG START POINT =====
#root folder for all transfers on this device
BASE_DIR = Path.home() / "lidar_transfer"

#lidar drops new .ply files here
OUTGOING_DIR = BASE_DIR / "outgoing_ply"

#archive folder for files that were already sent
SENT_DIR = BASE_DIR / "sent_ply"

#create base and subfolders if missing
BASE_DIR.mkdir(parents=True, exist_ok=True)
OUTGOING_DIR.mkdir(parents=True, exist_ok=True)
SENT_DIR.mkdir(parents=True, exist_ok=True)
#===== SECTION 3: FOLDER CONFIG END POINT =====

#===== SECTION 4: CHANGE IP START POINT =====
def changeAddress():
    config = configparser.ConfigParser()
    config.read(CONFIG_FILE) # read current config file information

    #get current device settings
    current_ip = config.get("NETWORK","LAPTOP_IP",fallback="192.168.1.157")
    current_device = config.get("DEVICE","DEVICE_NUM",fallback="1")
    current_port = config.get("NETWORK","PORT",fallback="5001")

    #create ip changer GUI and entry boxes
    root = tk.Tk()
    root.title("Network and Device Settings")
    root.geometry("300x300")

    tk.Label(root,text="New IP:").pack(pady=(10,0))
    ip_entry = tk.Entry(root)
    ip_entry.insert(0,current_ip)
    ip_entry.pack()

    tk.Label(root,text="New Port:").pack(pady=(10,0))
    port_entry = tk.Entry(root)
    port_entry.insert(0,current_port)
    port_entry.pack()

    tk.Label(root,text="New Device Number").pack(pady=(10,0))
    num_entry = tk.Entry(root)
    num_entry.insert(0,current_device)
    num_entry.pack()

    def save_button(): # take new device and network settings and save to config file
        new_ip = ip_entry.get().strip()
        new_device = num_entry.get().strip()
        new_port = port_entry.get().strip()

        #if box is empty, show error message
        if not new_ip:
            messagebox.showerror("Error", "IP cannot be empty.")
            return
        if not new_device:
            messagebox.showerror("Error", "Device number cannot be empty.")
            return
        if not new_port:
            messagebox.showerror("Error", "Port number cannot be empty.")
            return
        
        #enter new settings into config file
        config["NETWORK"]["LAPTOP_IP"] = new_ip
        config["DEVICE"]["DEVICE_NUM"] = new_device
        config["NETWORK"]["PORT"] = new_port

        with open(CONFIG_FILE, "w") as f:
            config.write(f)

        messagebox.showinfo("Success", "Settings updated.")
        root.destroy()

    tk.Button(root, text="Save", command=save_button).pack(pady=15)
    root.mainloop()

#===== SECTION 4: CHANGE IP END POINT =====


#===== SECTION 5: TRANSFER TUNING START POINT =====
POLL_SECONDS = 1.0
STABLE_SECONDS = 2.0
RETRY_DELAY = 5.0
CONNECT_TIMEOUT = 5.0
CHUNK_SIZE = 65536
#===== SECTION 5: TRANSFER TUNING END POINT =====


#===== SECTION 6: FILE DISCOVERY START POINT =====
def list_ply_files(folder: Path):
    #returns sorted list of .ply files in the outgoing folder
    return sorted(
        [p for p in folder.iterdir() if p.is_file() and p.suffix.lower() == ".ply"]
    )
#===== SECTION 6: FILE DISCOVERY END POINT =====


#===== SECTION 7: FILE STABILITY CHECK START POINT =====
def file_is_stable(path: Path, stable_seconds: float) -> bool:
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
#===== SECTION 7: FILE STABILITY CHECK END POINT =====


#===== SECTION 8: REMOTE FILENAME BUILDING START POINT =====
def build_remote_filename(local_path: Path, device_id: str) -> bytes:
    ts = time.strftime("%Y%m%d_%H%M%S")
    return f"{device_id}__{ts}__{local_path.name}".encode("utf-8")
#===== SECTION 8: REMOTE FILENAME BUILDING END POINT =====


#===== SECTION 9: TCP SEND PROTOCOL START POINT =====
def send_file(path: Path) -> None:
    config = configparser.ConfigParser()
    config.read(CONFIG_FILE)

    laptop_ip = config.get("NETWORK","LAPTOP_IP").strip()
    port = config.getint("NETWORK","PORT")

    device_number = config.getint("DEVICE","DEVICE_NUM")
    device_id = f"LIDAR_{device_number:02d}"

    file_size = path.stat().st_size
    remote_name = build_remote_filename(path,device_id)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(CONNECT_TIMEOUT)
        s.connect((laptop_ip, port))

        s.sendall(struct.pack("!I", len(remote_name)))
        s.sendall(remote_name)
        s.sendall(struct.pack("!Q", file_size))

        with open(path, "rb") as f:
            while True:
                chunk = f.read(CHUNK_SIZE)
                if not chunk:
                    break
                s.sendall(chunk)
#===== SECTION 9: TCP SEND PROTOCOL END POINT =====


#===== SECTION 10: POST-SEND ARCHIVE START POINT =====
def move_to_sent(src: Path) -> Path:
    dst = SENT_DIR / src.name
    base, suffix = dst.stem, dst.suffix
    i = 1

    while dst.exists():
        dst = SENT_DIR / f"{base}_{i}{suffix}"
        i += 1

    shutil.move(str(src), str(dst))
    return dst
#===== SECTION 10: POST-SEND ARCHIVE END POINT =====


#===== SECTION 11: MAIN LOOP START POINT =====
def main():
    sent_signatures = set()

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
#===== SECTION 11: MAIN LOOP END POINT =====


#===== SECTION 12: ENTRYPOINT START POINT =====
if __name__ == "__main__":
    main()
#===== SECTION 12: ENTRYPOINT END POINT =====
