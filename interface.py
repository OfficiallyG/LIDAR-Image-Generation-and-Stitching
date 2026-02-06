import sys
import os
import socket
import struct
from pathlib import Path
from typing import Optional
import numpy as np
import pyqtgraph.opengl as gl
from plyfile import PlyData
from PyQt6.QtCore import Qt, QThread, pyqtSignal
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QListWidget,
    QListWidgetItem,
    QProgressBar,
    QCheckBox,
    QFileDialog
)

DARK_STYLESHEET = """
QWidget { background-color: #121212; color: #e0e0e0; }
QGroupBox { border: 1px solid #333; margin-top: 6px; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 3px; }
QPushButton { background-color: #1e1e1e; border: 1px solid #333; padding: 6px; }
QListWidget, QProgressBar { background-color: #0f0f0f; border: 1px solid #333; }
QLabel, QCheckBox { color: #e0e0e0; }
QLineEdit { background-color: #1b1b1b; color: #e0e0e0; }
QScrollBar:vertical { background: #121212; }
"""
#Dark-mode theme for the whole app


def get_desktop_path() -> Path:
    #Finds a Desktop path that works on common Windows setups
    home = Path.home()
    candidates = [
        home / "Desktop",
        home / "OneDrive" / "Desktop",
        home / "OneDrive - Personal" / "Desktop",
    ]
    for p in candidates:
        if p.exists():
            return p
    return home
    #Fallback if Desktop not found


INBOX_DIR = get_desktop_path() / "LiDAR_Inbox"
#Incoming .ply files land here
INBOX_DIR.mkdir(parents=True, exist_ok=True)
#Auto-create inbox folder if missing
RECV_HOST = "0.0.0.0"
#Listen on all network interfaces
RECV_PORT = 5001
#Must match sender port


def recv_exact(conn: socket.socket, n: int) -> bytes:
    #Reads exactly n bytes or raises if the connection closes.
    data = b""
    while len(data) < n:
        chunk = conn.recv(n - len(data))
        if not chunk:
            raise ConnectionError("Connection closed while receiving data")
        data += chunk
    return data


class ReceiverWorker(QThread):
    file_received = pyqtSignal(str)
    #Emits saved file path for UI updates
    log = pyqtSignal(str)
    #Emits status messages
    error = pyqtSignal(str)
    #Emits error messages

    def __init__(self, host: str, port: int, inbox_dir: Path, parent=None):
        super().__init__(parent)
        self.host = host
        self.port = port
        self.inbox_dir = inbox_dir
        self._running = True
        #Loop control flag
        self._server_socket: Optional[socket.socket] = None
        #Used to interrupt accept() on stop

    def stop(self):
        self._running = False
        #Signals thread loop to exit
        try:
            if self._server_socket:
                self._server_socket.close()
                #Forces accept() to break quickly
        except Exception:
            pass

    def run(self):
        inbox = self.inbox_dir
        inbox.mkdir(parents=True, exist_ok=True)
        #Ensure inbox exists before receiving

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                self._server_socket = s
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                #Allows quick restart after closing
                s.bind((self.host, self.port))
                s.listen(5)
                s.settimeout(1.0)
                #Periodic wake-up to check _running

                self.log.emit(f"[Receiver] Listening on {self.host}:{self.port}")
                self.log.emit(f"[Receiver] Saving incoming files to: {inbox.resolve()}")

                while self._running:
                    try:
                        conn, addr = s.accept()
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                        #Socket closed by stop()

                    with conn:
                        try:
                            self.log.emit(f"[Receiver] Connection from {addr}")

                            name_len = struct.unpack("!I", recv_exact(conn, 4))[0]
                            #Filename length header
                            filename = recv_exact(conn, name_len).decode("utf-8")
                            #Filename payload
                            file_size = struct.unpack("!Q", recv_exact(conn, 8))[0]
                            #File size header

                            if not filename.lower().endswith(".ply"):
                                self.log.emit(f"[Receiver] Rejected non-.ply file: {filename}")
                                continue
                                #Hard-filter to only accept .ply

                            safe_name = os.path.basename(filename)
                            #Drops any directory paths from sender
                            out_path = inbox / safe_name
                            base = out_path.stem
                            suffix = out_path.suffix
                            i = 1
                            while out_path.exists():
                                out_path = inbox / f"{base}_{i}{suffix}"
                                i += 1
                                #De-dupe collisions in inbox

                            self.log.emit(f"[Receiver] Receiving: {safe_name} ({file_size} bytes)")
                            received = 0
                            with open(out_path, "wb") as f:
                                while received < file_size:
                                    chunk = conn.recv(min(65536, file_size - received))
                                    if not chunk:
                                        raise ConnectionError("Connection closed mid-transfer")
                                    f.write(chunk)
                                    received += len(chunk)
                                    #Stream file bytes to disk

                            self.log.emit(f"[Receiver] Saved {received} bytes -> {out_path}")
                            self.file_received.emit(str(out_path))
                            #Notifies UI to add file to queue

                        except Exception as e:
                            self.error.emit(f"[Receiver] Error: {e}")

        except Exception as e:
            self.error.emit(f"[Receiver] Fatal server error: {e}")


class SimplePLYViewport(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        #Fill the widget with the 3D view
        layout.setContentsMargins(0, 0, 0, 0)

        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=30)
        #Default camera distance
        layout.addWidget(self.view)

        self.points = gl.GLScatterPlotItem()
        #Point cloud renderer
        self.view.addItem(self.points)

    def load_ply(self, path: str):
        ply = PlyData.read(path)
        #Parse .ply file into structured data

        vertices = ply["vertex"].data
        #Vertex table inside the .ply

        pos = np.vstack((
            vertices["x"],
            vertices["y"],
            vertices["z"]
        )).T.astype(np.float32)
        #Build Nx3 float array for XYZ

        if all(name in vertices.dtype.names for name in ("red", "green", "blue")):
            rgb = np.vstack((
                vertices["red"],
                vertices["green"],
                vertices["blue"]
            )).T.astype(np.float32)

            if rgb.max() > 1.0:
                rgb /= 255.0
                #Normalize 0-255 to 0-1

            color = np.hstack((
                rgb,
                np.ones((rgb.shape[0], 1), dtype=np.float32)
            ))
            #Add alpha channel (RGBA)
        else:
            color = (1, 1, 1, 1)
            #Fallback color if no RGB

        self.points.setData(pos=pos, color=color, size=2)
        #Push data into OpenGL renderer


class LidarWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("LIDAR Image Generation & Stitching")
        self.setGeometry(250, 250, 1100, 650)

        self._build_ui()
        self._wire_min_signals()

        self._start_receiver()
        #Start TCP receiver while UI is open

        self._log("UI initialized. Receiver is running.")

    def _start_receiver(self):
        self.receiver = ReceiverWorker(RECV_HOST, RECV_PORT, INBOX_DIR, parent=self)
        self.receiver.log.connect(self._log)
        self.receiver.error.connect(self._log)
        self.receiver.file_received.connect(self._on_file_received)
        self.receiver.start()

    def _on_file_received(self, saved_path: str):
        filename = os.path.basename(saved_path)

        item = QListWidgetItem(filename)
        item.setData(Qt.ItemDataRole.UserRole, saved_path)
        #Store full path on the list item
        self.scan_list.addItem(item)

        self._log(f"Received -> Desktop/LiDAR_Inbox: {filename}")

        if self.auto_load_chk.isChecked():
            #Optional auto-preview of newest file
            try:
                self.viewport3d.load_ply(saved_path)
                self.info_lbl.setText(f"Loaded file:\n{filename}")
                self._log("Auto-loaded newest received file into 3D viewport.")
            except Exception as e:
                self._log(f"Auto-load failed: {e}")

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)

        main_layout = QHBoxLayout(root)
        #3-column layout (left|center|right)

        left = QVBoxLayout()
        main_layout.addLayout(left, 1)

        left.addWidget(QLabel("Scans Queue"))

        self.scan_list = QListWidget()
        left.addWidget(self.scan_list, 1)

        file_btn_row = QHBoxLayout()
        self.btn_import_local = QPushButton("Import Local")
        self.btn_delete = QPushButton("Delete Selected")
        file_btn_row.addWidget(self.btn_import_local)
        file_btn_row.addWidget(self.btn_delete)
        left.addLayout(file_btn_row)

        self.auto_load_chk = QCheckBox("Auto-load newest file after import/receive")
        self.auto_load_chk.setChecked(True)
        left.addWidget(self.auto_load_chk)

        center = QVBoxLayout()
        main_layout.addLayout(center, 3)

        self.viewport3d = SimplePLYViewport()
        center.addWidget(self.viewport3d, 1)

        right = QVBoxLayout()
        main_layout.addLayout(right, 1)

        render_group = QGroupBox("Render Controls")
        render_layout = QVBoxLayout(render_group)

        self.chk_dark_bg = QCheckBox("Dark background")
        self.chk_dark_bg.setChecked(True)

        self.chk_grid = QCheckBox("Show grid")
        self.chk_grid.setChecked(True)

        self.chk_axes = QCheckBox("Show axes")
        self.chk_axes.setChecked(True)

        self.btn_pt_minus = QPushButton("Point Size: -")
        self.btn_pt_plus = QPushButton("Point Size: +")

        self.btn_color_ply = QPushButton("Color: PLY")
        self.btn_color_height = QPushButton("Color: Height")
        self.btn_color_single = QPushButton("Color: Single")

        for w in (
            self.chk_dark_bg, self.chk_grid, self.chk_axes,
            self.btn_pt_minus, self.btn_pt_plus,
            self.btn_color_ply, self.btn_color_height, self.btn_color_single
        ):
            render_layout.addWidget(w)

        right.addWidget(render_group)

        view_group = QGroupBox("View Controls")
        view_layout = QVBoxLayout(view_group)

        self.btn_fit = QPushButton("Center & Fit")
        self.btn_reset = QPushButton("Reset View")

        view_layout.addWidget(self.btn_fit)
        view_layout.addWidget(self.btn_reset)
        right.addWidget(view_group)

        info_group = QGroupBox("File Info")
        info_layout = QVBoxLayout(info_group)

        self.info_lbl = QLabel("No file loaded.")
        self.info_lbl.setWordWrap(True)
        info_layout.addWidget(self.info_lbl)

        right.addWidget(info_group)

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        right.addWidget(self.progress)

        right.addWidget(QLabel("Log"))

        self.log_lbl = QLabel("")
        self.log_lbl.setWordWrap(True)
        self.log_lbl.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.log_lbl.setMinimumHeight(140)
        self.log_lbl.setStyleSheet("border: 1px solid #999; padding: 8px;")
        right.addWidget(self.log_lbl, 1)

        self._apply_dark_theme(self.chk_dark_bg.isChecked())

    def _wire_min_signals(self):
        self.btn_import_local.clicked.connect(self._import_local_clicked)
        self.btn_delete.clicked.connect(self._delete_selected_clicked)

        self.scan_list.itemDoubleClicked.connect(self._load_item_into_viewport)
        #Double-click loads selected scan

        try:
            self.chk_dark_bg.toggled.connect(self._apply_dark_theme)
        except Exception:
            pass

        self._apply_dark_theme(self.chk_dark_bg.isChecked())

    def _import_local_clicked(self):
        file_paths, _ = QFileDialog.getOpenFileNames(
            self,
            "Select LiDAR Scan(s)",
            "",
            "PLY Files (*.ply);;All Files (*)"
        )

        if not file_paths:
            self._log("Import canceled.")
            return

        for path in file_paths:
            filename = os.path.basename(path)
            item = QListWidgetItem(filename)
            item.setData(Qt.ItemDataRole.UserRole, path)
            #Store full path on the list item
            self.scan_list.addItem(item)
            self._log(f"Imported local file: {filename}")

        if self.auto_load_chk.isChecked():
            newest_path = file_paths[-1]
            #Loads the most recently selected file
            try:
                self.viewport3d.load_ply(newest_path)
                self.info_lbl.setText(f"Loaded file:\n{os.path.basename(newest_path)}")
                self._log("Loaded newest imported file into 3D viewport.")
            except Exception as e:
                self._log(f"Load failed: {e}")

    def _load_item_into_viewport(self, item: QListWidgetItem):
        path = item.data(Qt.ItemDataRole.UserRole)
        #Pull stored path from list item
        if not path or not os.path.exists(path):
            self._log("Selected item has no valid file path (missing file?).")
            return

        try:
            self.viewport3d.load_ply(path)
            self.info_lbl.setText(f"Loaded file:\n{os.path.basename(path)}")
            self._log(f"Loaded from queue: {os.path.basename(path)}")
        except Exception as e:
            self._log(f"Load failed: {e}")

    def _delete_selected_clicked(self):
        for item in self.scan_list.selectedItems():
            row = self.scan_list.row(item)
            self.scan_list.takeItem(row)
        #Removes from UI queue only

        self._log("Deleted selected scan(s) from queue.")

    def _log(self, msg: str):
        from time import strftime
        ts = strftime("%H:%M:%S")
        current = self.log_lbl.text()
        self.log_lbl.setText((f"[{ts}] {msg}\n" + current)[:4000])
        #Keeps log bounded to avoid UI slowdown

    def _apply_dark_theme(self, enabled: bool):
        app = QApplication.instance()
        if not app:
            return
        app.setStyleSheet(DARK_STYLESHEET if enabled else "")
        #Applies global stylesheet to the app

    def closeEvent(self, event):
        try:
            if hasattr(self, "receiver") and self.receiver.isRunning():
                self._log("Stopping receiver...")
                self.receiver.stop()
                self.receiver.wait(1500)
                #Stops thread before closing UI
        except Exception:
            pass
        super().closeEvent(event)


def main():
    lidar_app = QApplication(sys.argv)
    lidar_main = LidarWindow()
    lidar_main.show()
    sys.exit(lidar_app.exec())


if __name__ == "__main__":
    main()
