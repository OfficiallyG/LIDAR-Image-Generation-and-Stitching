import os
import socket
import struct
from pathlib import Path

HOST = "0.0.0.0"   #listens on all interfaces
PORT = 5001
SAVE_DIR = Path("received_ply")
SAVE_DIR.mkdir(exist_ok=True)

def recv_exact(conn, n: int) -> bytes:
    data = b""
    while len(data) < n:
        chunk = conn.recv(n - len(data))
        if not chunk:
            raise ConnectionError("Connection closed while receiving data")
        data += chunk
    return data

def main():
    print(f"[Laptop] Listening on {HOST}:{PORT}")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(5)

        while True:
            conn, addr = s.accept()
            with conn:
                print(f"\n[+] Connection from {addr}")

                name_len = struct.unpack("!I", recv_exact(conn, 4))[0]
                filename = recv_exact(conn, name_len).decode("utf-8")
                file_size = struct.unpack("!Q", recv_exact(conn, 8))[0]

                if not filename.lower().endswith(".ply"):
                    print(f"[!] Rejected non-.ply file: {filename}")
                    continue

                out_path = SAVE_DIR / os.path.basename(filename)
                base = out_path.stem
                suffix = out_path.suffix
                i = 1
                while out_path.exists():
                    out_path = SAVE_DIR / f"{base}_{i}{suffix}"
                    i += 1

                print(f"[*] Receiving: {filename} ({file_size} bytes) -> {out_path}")
                received = 0
                with open(out_path, "wb") as f:
                    while received < file_size:
                        chunk = conn.recv(min(65536, file_size - received))
                        if not chunk:
                            raise ConnectionError("Connection closed mid-transfer")
                        f.write(chunk)
                        received += len(chunk)

                print(f"Saved {received} bytes.")

if __name__ == "__main__":
    main()
