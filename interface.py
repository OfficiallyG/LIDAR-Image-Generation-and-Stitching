#===== SECTION 1: IMPORTS START POINT =====
# #import sys for argv and clean app exit
import sys
#import os for safe filename handling and file existence checks
import os
#import socket for tcp server and local ip discovery
import socket
#import struct for packing/unpacking fixed-size integers in the transfer protocol
import struct
#import pathlib for safe cross-platform path building
from pathlib import Path
#import typing for clearer intent in function signatures
from typing import Optional, List, Tuple

#import numpy for fast math and array operations for point clouds
import numpy as np
#import pyqtgraph.opengl for 3d rendering widgets/items
import pyqtgraph.opengl as gl
#import qt core for ui constants, background thread, signals, and timers
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
#import qt widgets for the main window and all ui controls
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
#===== SECTION 1: IMPORTS END POINT =====

#===== SECTION 2: PATHS AND NETWORK CONFIG START POINT =====
def get_desktop_path() -> Path:
    #find a desktop path that works on common windows setups
    home = Path.home()
    candidates = [
        home / "Desktop",
        home / "OneDrive" / "Desktop",
        home / "OneDrive - Personal" / "Desktop",
    ]
    for p in candidates:
        if p.exists():
            return p
    #fallback if desktop not found
    return home


def get_local_ip() -> str:
    #best-effort lan ip discovery by checking the outbound routing interface
    ip = "127.0.0.1"
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            #udp connect is used only to learn which interface would be used
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
    except Exception:
        #keep fallback ip if anything fails
        pass
    return ip


#receiver saves incoming .ply files here
INBOX_DIR = get_desktop_path() / "LiDAR_Inbox"
#ensure inbox exists on startup
INBOX_DIR.mkdir(parents=True, exist_ok=True)

#bind on all network interfaces so other devices can connect
RECV_HOST = "0.0.0.0"
#must match sender port
RECV_PORT = 5001
#===== SECTION 2: PATHS AND NETWORK CONFIG END POINT =====

#===== SECTION 3: TCP TRANSFER HELPERS START POINT =====
def recv_exact(conn: socket.socket, n: int) -> bytes:
    #read exactly n bytes or raise if connection closes
    data = b""
    while len(data) < n:
        chunk = conn.recv(n - len(data))
        if not chunk:
            raise ConnectionError("connection closed while receiving data")
        data += chunk
    return data
#===== SECTION 3: TCP TRANSFER HELPERS END POINT =====

#===== SECTION 4: RECEIVER BACKGROUND THREAD START POINT =====
class ReceiverWorker(QThread):
    #emits the saved file path so ui can update list and optionally auto-load
    file_received = pyqtSignal(str)
    #thread-safe logging to the ui
    log = pyqtSignal(str)
    #thread-safe error reporting to the ui
    error = pyqtSignal(str)
    #true = actively listening, false = stopped/faulted
    status = pyqtSignal(bool)

    def __init__(self, host: str, port: int, inbox_dir: Path, parent=None):
        super().__init__(parent)
        self.host = host
        self.port = port
        self.inbox_dir = inbox_dir
        #running flag polled by the accept loop
        self._running = True
        #server socket stored so stop() can force accept() to exit
        self._server_socket: Optional[socket.socket] = None

    def stop(self):
        #request thread shutdown
        self._running = False
        try:
            if self._server_socket:
                #closing server socket breaks accept() quickly
                self._server_socket.close()
        except Exception:
            pass

    def run(self):
        #ensure inbox exists before binding receiver
        inbox = self.inbox_dir
        inbox.mkdir(parents=True, exist_ok=True)

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                self._server_socket = s
                #allow quick restart without "address already in use"
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind((self.host, self.port))
                s.listen(5)
                #timeout lets us periodically check the stop flag
                s.settimeout(1.0)

                #receiver is now listening
                self.status.emit(True)
                self.log.emit(f"[receiver] listening on {self.host}:{self.port}")
                self.log.emit(f"[receiver] saving incoming files to: {inbox.resolve()}")

                while self._running:
                    try:
                        conn, addr = s.accept()
                    except socket.timeout:
                        continue
                    except OSError:
                        #most likely stop() closed the socket
                        break

                    with conn:
                        try:
                            self.log.emit(f"[receiver] connection from {addr}")

                            #protocol frame: 4 bytes filename length, filename bytes, 8 bytes filesize, file bytes
                            name_len = struct.unpack("!I", recv_exact(conn, 4))[0]
                            filename = recv_exact(conn, name_len).decode("utf-8")
                            file_size = struct.unpack("!Q", recv_exact(conn, 8))[0]

                            #only accept .ply files
                            if not filename.lower().endswith(".ply"):
                                self.log.emit(f"[receiver] rejected non-.ply file: {filename}")
                                continue

                            #prevent directory traversal from sender-provided paths
                            safe_name = os.path.basename(filename)
                            out_path = inbox / safe_name

                            #de-dupe filename if it already exists
                            base = out_path.stem
                            suffix = out_path.suffix
                            i = 1
                            while out_path.exists():
                                out_path = inbox / f"{base}_{i}{suffix}"
                                i += 1

                            self.log.emit(f"[receiver] receiving: {safe_name} ({file_size} bytes)")

                            received = 0
                            with open(out_path, "wb") as f:
                                while received < file_size:
                                    chunk = conn.recv(min(65536, file_size - received))
                                    if not chunk:
                                        raise ConnectionError("connection closed mid-transfer")
                                    f.write(chunk)
                                    received += len(chunk)

                            self.log.emit(f"[receiver] saved {received} bytes -> {out_path}")
                            self.file_received.emit(str(out_path))

                        except Exception as e:
                            self.error.emit(f"[receiver] error: {e}")

        except Exception as e:
            self.error.emit(f"[receiver] fatal server error: {e}")

        finally:
            #receiver ended normally or due to crash
            self.status.emit(False)
#===== SECTION 4: RECEIVER BACKGROUND THREAD END POINT =====

#===== SECTION 5: PLY LOADING HELPERS START POINT =====
PLY_DTYPE_MAP = {
    "char": "i1",
    "int8": "i1",
    "uchar": "u1",
    "uint8": "u1",
    "short": "i2",
    "int16": "i2",
    "ushort": "u2",
    "uint16": "u2",
    "int": "i4",
    "int32": "i4",
    "uint": "u4",
    "uint32": "u4",
    "float": "f4",
    "float32": "f4",
    "double": "f8",
    "float64": "f8",
}


def read_ply_xyz(path: str) -> np.ndarray:
    #read xyz vertices from ascii or binary little-endian ply files
    with open(path, "rb") as f:
        fmt = None
        vertex_count = None
        vertex_props: List[Tuple[str, str]] = []
        in_vertex = False

        while True:
            line = f.readline()
            if not line:
                raise ValueError("invalid ply: missing end_header")

            try:
                s = line.decode("ascii").strip()
            except UnicodeDecodeError:
                raise ValueError("invalid ply header")

            if s.startswith("format "):
                parts = s.split()
                if len(parts) >= 2:
                    fmt = parts[1]
            elif s.startswith("element "):
                parts = s.split()
                if len(parts) >= 3 and parts[1] == "vertex":
                    vertex_count = int(parts[2])
                    in_vertex = True
                    vertex_props = []
                else:
                    in_vertex = False
            elif s.startswith("property ") and in_vertex:
                parts = s.split()
                if len(parts) >= 3 and parts[1] != "list":
                    vertex_props.append((parts[2], parts[1]))
            elif s == "end_header":
                break

        if fmt is None or vertex_count is None or not vertex_props:
            raise ValueError("unsupported ply header")

        prop_names = [name for name, _ in vertex_props]
        if not {"x", "y", "z"}.issubset(prop_names):
            raise ValueError("ply does not contain x y z columns")

        if fmt == "ascii":
            data = np.loadtxt(f, dtype=np.float64, usecols=[prop_names.index("x"), prop_names.index("y"), prop_names.index("z")], max_rows=vertex_count)
            if data.ndim == 1:
                data = data.reshape(1, 3)
            return data.astype(np.float32)

        if fmt != "binary_little_endian":
            raise ValueError(f"unsupported ply format: {fmt}")

        dtype_fields = []
        for name, typ in vertex_props:
            if typ not in PLY_DTYPE_MAP:
                raise ValueError(f"unsupported ply property type: {typ}")
            dtype_fields.append((name, "<" + PLY_DTYPE_MAP[typ]))

        vertex_dtype = np.dtype(dtype_fields)
        data = np.fromfile(f, dtype=vertex_dtype, count=vertex_count)
        xyz = np.column_stack((data["x"], data["y"], data["z"]))
        return xyz.astype(np.float32)
#===== SECTION 5: PLY LOADING HELPERS END POINT =====

#===== SECTION 6: 3D VIEWPORT (SINGLE SCAN) START POINT =====
class SinglePLYViewport(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        #ui layout: viewport fills this widget
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        #single 3d viewer for one point cloud at a time
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=85, elevation=18, azimuth=45)

        #make background white for better contrast in screenshots
        try:
            self.view.setBackgroundColor("k")
        except Exception:
            self.view.opts["bgcolor"] = (255, 255, 255, 255)

        layout.addWidget(self.view)

        #one scatter item only
        self.point_cloud_item = gl.GLScatterPlotItem(size=3.5, pxMode=True)
        self.point_cloud_item.setGLOptions("opaque")
        self.view.addItem(self.point_cloud_item)

        #track if anything is loaded
        self.file_loaded = False

    def clear_view(self):
        #wipe the current point cloud from the viewer
        self.point_cloud_item.setData(pos=np.zeros((0, 3), dtype=np.float32))
        self.file_loaded = False

    def _robust_normalize(self, z: np.ndarray) -> np.ndarray:
        #normalize heights using percentiles so outliers do not dominate color mapping
        if z.size == 0:
            return z
        lo = float(np.percentile(z, 2))
        hi = float(np.percentile(z, 98))
        if (hi - lo) < 1e-6:
            lo = float(z.min())
            hi = float(z.max())
            if (hi - lo) < 1e-6:
                return np.zeros_like(z, dtype=np.float32)
        t = (z - lo) / (hi - lo)
        return np.clip(t, 0.0, 1.0).astype(np.float32)

    def _height_colors_smooth(self, t: np.ndarray) -> np.ndarray:
        #map normalized heights to a smooth gradient rgba color
        anchors = np.array([
            [1.00, 0.92, 0.10],
            [1.00, 0.55, 0.10],
            [0.95, 0.15, 0.15],
            [0.15, 0.45, 0.95],
            [0.60, 0.25, 0.85],
            [0.15, 0.80, 0.30],
        ], dtype=np.float32)

        t = np.clip(t, 0.0, 1.0)
        nseg = anchors.shape[0] - 1
        u = t * nseg
        i0 = np.floor(u).astype(np.int32)
        i0 = np.clip(i0, 0, nseg - 1)
        f = (u - i0).astype(np.float32)

        c0 = anchors[i0]
        c1 = anchors[i0 + 1]
        rgb = (1.0 - f[:, None]) * c0 + f[:, None] * c1
        return np.hstack((rgb, np.ones((rgb.shape[0], 1), dtype=np.float32)))

    def load_ply(self, path: str):
        #load one ply, fit it into view, color it by height, and replace the current scan
        pos = read_ply_xyz(path)
        if pos.size == 0:
            raise ValueError("ply contains no vertices")

        mins = pos.min(axis=0)
        maxs = pos.max(axis=0)
        center = (mins + maxs) / 2.0
        span = maxs - mins
        max_span = float(np.max(span)) if float(np.max(span)) > 0 else 1.0
        fit_scale = 25.0 / max_span
        pos_local = (pos - center) * fit_scale

        z = pos_local[:, 2]
        t = self._robust_normalize(z)
        color = self._height_colors_smooth(t)

        self.point_cloud_item.setData(
            pos=pos_local.astype(np.float32),
            color=color.astype(np.float32),
            size=3.5,
            pxMode=True
        )
        self.point_cloud_item.setGLOptions("opaque")
        self.file_loaded = True

        distance = max(25.0, max_span * fit_scale * 2.5)
        self.view.setCameraPosition(distance=distance, elevation=18, azimuth=45)
#===== SECTION 6: 3D VIEWPORT (SINGLE SCAN) END POINT =====

#===== SECTION 7: MAIN WINDOW (UI + APP LOGIC) START POINT =====
class LidarWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("LIDAR Image Generation & Stitching")
        self.setGeometry(250, 250, 1100, 650)

        self._closing = False

        #===== SECTION 8: UI CREATION START POINT =====
        self._build_ui()
        #===== SECTION 8: UI CREATION END POINT =====

        #===== SECTION 9: UI SIGNAL WIRING START POINT =====
        self._wire_min_signals()
        #===== SECTION 9: UI SIGNAL WIRING END POINT =====

        #===== SECTION 10: RECEIVER STARTUP START POINT =====
        self._start_receiver()
        self._log("ui initialized. receiver is running.")
        #===== SECTION 10: RECEIVER STARTUP END POINT =====

        #===== SECTION 11: IP REFRESH TIMER START POINT =====
        self._ip_timer = QTimer(self)
        self._ip_timer.setInterval(2500)
        self._ip_timer.timeout.connect(self._refresh_ip_label)
        self._ip_timer.start()
        self._refresh_ip_label()
        #===== SECTION 11: IP REFRESH TIMER END POINT =====

    def _start_receiver(self):
        #start tcp receiver in a background thread so ui stays responsive
        self.receiver = ReceiverWorker(RECV_HOST, RECV_PORT, INBOX_DIR, parent=self)
        self.receiver.log.connect(self._log)
        self.receiver.error.connect(self._on_receiver_error)
        self.receiver.file_received.connect(self._on_file_received)
        self.receiver.finished.connect(self._on_receiver_finished)
        self.receiver.status.connect(self._on_receiver_status)
        self.receiver.start()

    def _on_receiver_status(self, running: bool):
        #update ui status label and dot indicator
        if running:
            self.receiver_status_lbl.setText("Receiver: RUNNING")
            self._set_status_dot(True)
        else:
            self.receiver_status_lbl.setText("Receiver: NOT RUNNING")
            self._set_status_dot(False)

    def _set_status_dot(self, running: bool):
        #glowing dot indicator is driven purely by stylesheet
        if running:
            self.receiver_dot.setStyleSheet(
                "width: 14px; height: 14px; border-radius: 7px;"
                "background: #28d14c;"
                "border: 1px solid #0a7a1c;"
                "box-shadow: 0 0 10px rgba(40, 209, 76, 0.9);"
            )
        else:
            self.receiver_dot.setStyleSheet(
                "width: 14px; height: 14px; border-radius: 7px;"
                "background: #e03a3a;"
                "border: 1px solid #8a1111;"
                "box-shadow: 0 0 10px rgba(224, 58, 58, 0.9);"
            )

    def _refresh_ip_label(self):
        #shows the laptop ip so the pi operator can type it into sender config
        ip = get_local_ip()
        self.ip_value_lbl.setText(f"{ip}:{RECV_PORT}")

    def _on_receiver_error(self, msg: str):
        #surface receiver errors in the ui log
        self._log(msg)
        if "fatal" in msg.lower():
            self._set_status_dot(False)
            self.receiver_status_lbl.setText("Receiver: NOT RUNNING")

    def _on_receiver_finished(self):
        #receiver thread ended (stop or crash)
        self._log("[receiver] receiver thread finished.")
        self._set_status_dot(False)
        self.receiver_status_lbl.setText("Receiver: NOT RUNNING")

    def _load_path_into_viewer(self, path: str):
        #load a file into the single viewer and replace the current scan
        try:
            self.viewer3d.load_ply(path)
            self._log(f"loaded scan: {os.path.basename(path)}")
            self.info_lbl.setText(f"Loaded:\n{os.path.basename(path)}")
        except Exception as e:
            self._log(f"load failed: {e}")

    def _on_file_received(self, saved_path: str):
        #add the received file to the queue list and optionally auto-load it
        filename = os.path.basename(saved_path)
        item = QListWidgetItem(filename)
        item.setData(Qt.ItemDataRole.UserRole, saved_path)
        self.scan_list.addItem(item)
        self._log(f"received -> Desktop/LiDAR_Inbox: {filename}")

        if self.auto_load_chk.isChecked():
            self._load_path_into_viewer(saved_path)

    def _build_ui(self):
        #===== SECTION 12: UI LAYOUT START POINT =====
        root = QWidget()
        self.setCentralWidget(root)
        main_layout = QHBoxLayout(root)

        #===== SECTION 13: LEFT COLUMN (QUEUE + STATUS) START POINT =====
        left = QVBoxLayout()
        main_layout.addLayout(left, 1)

        status_row = QHBoxLayout()
        self.receiver_dot = QLabel("")
        self.receiver_dot.setFixedSize(14, 14)
        self.receiver_status_lbl = QLabel("Receiver: NOT RUNNING")
        self.receiver_status_lbl.setStyleSheet("font-weight: bold;")
        status_row.addWidget(self.receiver_dot)
        status_row.addWidget(self.receiver_status_lbl)
        status_row.addStretch(1)
        left.addLayout(status_row)
        self._set_status_dot(False)

        ip_row = QHBoxLayout()
        ip_title = QLabel("Laptop IP:")
        ip_title.setStyleSheet("font-weight: bold;")
        self.ip_value_lbl = QLabel("0.0.0.0:----")
        self.ip_value_lbl.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        ip_row.addWidget(ip_title)
        ip_row.addWidget(self.ip_value_lbl, 1)
        left.addLayout(ip_row)

        left.addSpacing(8)

        left.addWidget(QLabel("Scans Queue"))
        self.scan_list = QListWidget()
        left.addWidget(self.scan_list, 1)

        file_btn_row = QHBoxLayout()
        self.btn_import_local = QPushButton("Import Local")
        self.btn_delete = QPushButton("Delete Selected")
        file_btn_row.addWidget(self.btn_import_local)
        file_btn_row.addWidget(self.btn_delete)
        left.addLayout(file_btn_row)

        self.auto_load_chk = QCheckBox("Auto-load after import/receive")
        self.auto_load_chk.setChecked(True)
        left.addWidget(self.auto_load_chk)
        #===== SECTION 13: LEFT COLUMN (QUEUE + STATUS) END POINT =====

        #===== SECTION 14: CENTER COLUMN (3D VIEW) START POINT =====
        center = QVBoxLayout()
        main_layout.addLayout(center, 3)

        self.viewer3d = SinglePLYViewport()
        center.addWidget(self.viewer3d, 1)

        hint = QLabel("Viewer shows one .ply scan at a time. Loading another file replaces the current scan.")
        hint.setWordWrap(True)
        center.addWidget(hint)
        #===== SECTION 14: CENTER COLUMN (3D VIEW) END POINT =====

        #===== SECTION 15: RIGHT COLUMN (CONTROLS + LOG) START POINT =====
        right = QVBoxLayout()
        main_layout.addLayout(right, 1)

        render_group = QGroupBox("Render Controls")
        render_layout = QVBoxLayout(render_group)

        self.btn_clear_view = QPushButton("Clear Viewer")
        render_layout.addWidget(self.btn_clear_view)
        right.addWidget(render_group)

        info_group = QGroupBox("File Info")
        info_layout = QVBoxLayout(info_group)
        self.info_lbl = QLabel("No file loaded.")
        self.info_lbl.setFixedSize(250, 30)
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
        self.log_lbl.setFixedWidth(280)
        self.log_lbl.setStyleSheet("border: 1px solid #999; padding: 8px;")
        right.addWidget(self.log_lbl, 1)
        #===== SECTION 15: RIGHT COLUMN (CONTROLS + LOG) END POINT =====
        #===== SECTION 12: UI LAYOUT END POINT =====

    def _wire_min_signals(self):
        #===== SECTION 16: UI EVENTS START POINT =====
        self.btn_import_local.clicked.connect(self._import_local_clicked)
        self.btn_delete.clicked.connect(self._delete_selected_clicked)
        self.scan_list.itemDoubleClicked.connect(self._load_item_into_viewer)
        self.btn_clear_view.clicked.connect(self._clear_view_clicked)

    def _clear_view_clicked(self):
        self.viewer3d.clear_view()
        self.info_lbl.setText("No file loaded.")
        self._log("cleared viewer.")

    def _import_local_clicked(self):
        #choose one or more ply files and add them to the queue
        file_paths, _ = QFileDialog.getOpenFileNames(
            self,
            "Select LiDAR Scan(s)",
            "",
            "PLY Files (*.ply);;All Files (*)"
        )
        if not file_paths:
            self._log("import canceled.")
            return

        for path in file_paths:
            filename = os.path.basename(path)
            item = QListWidgetItem(filename)
            item.setData(Qt.ItemDataRole.UserRole, path)
            self.scan_list.addItem(item)
            self._log(f"imported local file: {filename}")

        if self.auto_load_chk.isChecked() and file_paths:
            self._load_path_into_viewer(file_paths[-1])

    def _load_item_into_viewer(self, item: QListWidgetItem):
        #double-clicking a queued file loads it into the viewer
        path = item.data(Qt.ItemDataRole.UserRole)
        if not path or not os.path.exists(path):
            self._log("selected item has no valid file path (missing file?).")
            return
        self._load_path_into_viewer(path)

    def _delete_selected_clicked(self):
        #removes from ui list only (does not delete file on disk)
        for item in self.scan_list.selectedItems():
            row = self.scan_list.row(item)
            self.scan_list.takeItem(row)
        self._log("deleted selected scan(s) from queue.")

    def _log(self, msg: str):
        #prepend logs with a time stamp and cap total size for responsiveness
        from time import strftime
        ts = strftime("%H:%M:%S")
        current = self.log_lbl.text()
        self.log_lbl.setText((f"[{ts}] {msg}\n" + current)[:4000])

    def closeEvent(self, event):
        #stop receiver thread during window close
        self._closing = True
        try:
            if hasattr(self, "receiver") and self.receiver.isRunning():
                self._log("stopping receiver...")
                self.receiver.stop()
                self.receiver.wait(1500)
        except Exception:
            pass
        super().closeEvent(event)
#===== SECTION 16: UI EVENTS END POINT =====

#===== SECTION 7: MAIN WINDOW (UI + APP LOGIC) END POINT =====

#===== SECTION 17: APP ENTRYPOINT START POINT =====
def main():
    lidar_app = QApplication(sys.argv)
    lidar_main = LidarWindow()
    lidar_main.show()
    sys.exit(lidar_app.exec())
#===== SECTION 17: APP ENTRYPOINT END POINT =====
if __name__ == "__main__":
    main()
