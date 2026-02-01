import os
import socket
import struct
from pathlib import Path

LAPTOP_IP = "000.000.0.00"  #change this # to your laptop IP
PORT = 5001

def send_file(filepath: str):
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(filepath)
    if path.suffix.lower() != ".ply":
        raise ValueError("Only .ply files allowed in this sender")

    filename = path.name.encode("utf-8")
    file_size = os.path.getsize(path)

    print(f"Sending {path.name} ({file_size} bytes) -> {LAPTOP_IP}:{PORT}")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((LAPTOP_IP, PORT))

        # Send header
        s.sendall(struct.pack("!I", len(filename)))
        s.sendall(filename)
        s.sendall(struct.pack("!Q", file_size))

        # Send file
        with open(path, "rb") as f:
            while True:
                chunk = f.read(65536)
                if not chunk:
                    break
                s.sendall(chunk)

    print("Transfer complete.")

if __name__ == "__main__":
    # Example usage: python3 sender.py /home/pi/scans/scan1.ply
    import sys
    if len(sys.argv) != 2:
        print("Usage: python3 sender.py /path/to/file.ply")
        raise SystemExit(1)

    send_file(sys.argv[1])
