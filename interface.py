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

#import numpy for fast math and array operations for point clouds and camera picking
import numpy as np
#import pyqtgraph.opengl for 3d rendering widgets/items (scatter + line cubes)
import pyqtgraph.opengl as gl
#import vector helper used by glviewwidget camera center
from pyqtgraph import Vector
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

#===== SECTION 5: CAMERA AND PICKING MATH START POINT =====
def _normalize(v: np.ndarray) -> np.ndarray:
    #safe unit-vector helper used in camera ray math
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return v
    return v / n


class SlotGLView(gl.GLViewWidget):
    #gl view: double-click selects a slot by raycasting to the board plane (z=0)
    slot_double_clicked = pyqtSignal(int)

    def __init__(self, get_selected_center_fn, pick_slot_from_world_fn, parent=None):
        super().__init__(parent)
        #callback to find current selected cube center (for zoom targeting)
        self._get_selected_center_fn = get_selected_center_fn
        #callback to map a world hit point to a slot index
        self._pick_slot_from_world_fn = pick_slot_from_world_fn

    def _raycast_to_plane_z0(self, x_px: float, y_px: float) -> Optional[Tuple[float, float, float]]:
        #convert screen pixel to a world hit point on the z=0 plane
        w = max(1, self.width())
        h = max(1, self.height())
        aspect = w / h

        #normalize pixel coordinates to [-1..1] (ndc space)
        x_ndc = (2.0 * (x_px / w)) - 1.0
        y_ndc = 1.0 - (2.0 * (y_px / h))

        #read camera parameters from glviewwidget
        dist = float(self.opts.get("distance", 50.0))
        elev = float(self.opts.get("elevation", 0.0))
        azim = float(self.opts.get("azimuth", 0.0))
        fov = float(self.opts.get("fov", 60.0))
        center = self.opts.get("center", Vector(0, 0, 0))
        cx, cy, cz = float(center.x()), float(center.y()), float(center.z())

        #convert degrees to radians
        el = np.deg2rad(elev)
        az = np.deg2rad(azim)

        #compute camera position from spherical coordinates around center
        cam_pos = np.array([
            cx + dist * np.cos(el) * np.cos(az),
            cy + dist * np.cos(el) * np.sin(az),
            cz + dist * np.sin(el),
        ], dtype=np.float64)

        target = np.array([cx, cy, cz], dtype=np.float64)
        forward = _normalize(target - cam_pos)

        #build a stable camera basis (right, up, forward)
        up_world = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        right = _normalize(np.cross(forward, up_world))
        if float(np.linalg.norm(right)) < 1e-6:
            #fallback if forward is nearly parallel to world up
            up_world = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            right = _normalize(np.cross(forward, up_world))
        up = _normalize(np.cross(right, forward))

        #project ndc into a camera ray direction using fov
        tan_half = np.tan(np.deg2rad(fov) / 2.0)
        dx = x_ndc * aspect * tan_half
        dy = y_ndc * tan_half
        ray_dir = _normalize((right * dx) + (up * dy) + (forward * 1.0))

        #ray-plane intersection with z=0 plane
        dz = ray_dir[2]
        if abs(dz) < 1e-9:
            return None
        t = -cam_pos[2] / dz
        if t < 0.0:
            return None

        hit = cam_pos + t * ray_dir
        return (float(hit[0]), float(hit[1]), 0.0)

    def mouseDoubleClickEvent(self, ev):
        #double-click selects a cube slot without breaking orbit controls
        if ev.button() == Qt.MouseButton.LeftButton:
            pos = ev.position()
            hit = self._raycast_to_plane_z0(pos.x(), pos.y())
            if hit is not None:
                idx = self._pick_slot_from_world_fn(hit[0], hit[1])
                if idx is not None:
                    self.slot_double_clicked.emit(idx)
                    ev.accept()
                    return
        super().mouseDoubleClickEvent(ev)

    def wheelEvent(self, ev):
        #zoom toward selected slot by shifting the camera center
        try:
            center = self._get_selected_center_fn()
            if center is not None:
                self.opts["center"] = Vector(center[0], center[1], center[2])
        except Exception:
            pass
        super().wheelEvent(ev)
#===== SECTION 5: CAMERA AND PICKING MATH END POINT =====

#===== SECTION 6: 3D VIEWPORT (9 SLOT GRID) START POINT =====
class MultiSlotPLYViewport(QWidget):
    slot_selected = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)

        #grid and cube layout settings
        self.max_slots = 9
        self.cube_size = 22.0
        self.slot_spacing = self.cube_size

        #precompute cube centers in world space
        self.slot_centers = self._build_slot_centers()

        #render items per slot
        self.slot_points: List[gl.GLScatterPlotItem] = []
        self.slot_cubes: List[gl.GLLinePlotItem] = []
        self.slot_cube_base_pts: List[np.ndarray] = []

        #slot state tracking
        self.slot_filled: List[bool] = [False] * self.max_slots
        self.selected_slot: Optional[int] = None

        #ui layout: viewport fills this widget
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        #use custom gl view so we can pick cubes by double-click
        self.view = SlotGLView(
            self.get_selected_center,
            self._pick_slot_from_world_xy
        )
        self.view.setCameraPosition(distance=85, elevation=18, azimuth=45)
        self.view.opts["center"] = Vector(0, 0, 0)

        #make background white for better contrast in screenshots
        try:
            self.view.setBackgroundColor("w")
        except Exception:
            self.view.opts["bgcolor"] = (255, 255, 255, 255)

        layout.addWidget(self.view)

        #create cubes + empty scatters
        self._build_slots()

        #wire picking into selection logic
        self.view.slot_double_clicked.connect(self.select_slot)
        self._refresh_cube_highlights()

    def _build_slot_centers(self) -> List[Tuple[float, float, float]]:
        #lay out 3x3 grid centered at origin on z=0
        centers = []
        for r in range(3):
            for c in range(3):
                x = (c - 1) * self.slot_spacing
                y = (1 - r) * self.slot_spacing
                centers.append((x, y, 0.0))
        return centers

    def get_selected_center(self) -> Optional[Tuple[float, float, float]]:
        #used by zoom to keep scroll focused on the selected cube
        if self.selected_slot is None:
            return None
        return self.slot_centers[self.selected_slot]

    def _pick_slot_from_world_xy(self, x: float, y: float) -> Optional[int]:
        #pick nearest cube center, then require click to be within the cube footprint
        centers_xy = np.array([(cx, cy) for (cx, cy, _) in self.slot_centers], dtype=np.float64)
        p = np.array([x, y], dtype=np.float64)
        d2 = np.sum((centers_xy - p) ** 2, axis=1)
        idx = int(np.argmin(d2))

        cx, cy, _ = self.slot_centers[idx]
        half = float(self.cube_size) / 2.0
        if abs(x - cx) <= half and abs(y - cy) <= half:
            return idx
        return None

    def _cube_edges(self, size: float) -> List[Tuple[np.ndarray, np.ndarray]]:
        #build 12 edges for a wireframe cube centered at origin
        s = size / 2.0
        p000 = np.array([-s, -s, -s], dtype=np.float32)
        p001 = np.array([-s, -s,  s], dtype=np.float32)
        p010 = np.array([-s,  s, -s], dtype=np.float32)
        p011 = np.array([-s,  s,  s], dtype=np.float32)
        p100 = np.array([ s, -s, -s], dtype=np.float32)
        p101 = np.array([ s, -s,  s], dtype=np.float32)
        p110 = np.array([ s,  s, -s], dtype=np.float32)
        p111 = np.array([ s,  s,  s], dtype=np.float32)
        return [
            (p000, p001), (p001, p011), (p011, p010), (p010, p000),
            (p100, p101), (p101, p111), (p111, p110), (p110, p100),
            (p000, p100), (p001, p101), (p011, p111), (p010, p110),
        ]

    def _wire_cube_dotted(self, size: float, dash_count: int = 18) -> np.ndarray:
        #create a dotted effect by alternating short line segments along each edge
        pts = []
        edges = self._cube_edges(size)
        n = max(6, dash_count)
        for a, b in edges:
            for i in range(n):
                t0 = i / n
                t1 = (i + 1) / n
                if i % 2 == 0:
                    p0 = a * (1.0 - t0) + b * t0
                    p1 = a * (1.0 - t1) + b * t1
                    pts.append(p0)
                    pts.append(p1)
        return np.vstack(pts).astype(np.float32)

    def _build_slots(self):
        #create one scatter renderer per slot
        for _ in range(self.max_slots):
            scatter = gl.GLScatterPlotItem(size=3.5, pxMode=True)
            scatter.setGLOptions("opaque")
            self.view.addItem(scatter)
            self.slot_points.append(scatter)

        #create one dotted cube outline per slot
        for i in range(self.max_slots):
            cx, cy, cz = self.slot_centers[i]
            base = self._wire_cube_dotted(self.cube_size, dash_count=18)
            base = base + np.array([cx, cy, cz], dtype=np.float32)
            self.slot_cube_base_pts.append(base)

            cube = gl.GLLinePlotItem(
                pos=base,
                mode="lines",
                width=2,
                color=(0.40, 0.40, 0.40, 0.45)
            )
            cube.setGLOptions("translucent")
            self.view.addItem(cube)
            self.slot_cubes.append(cube)

    def _refresh_cube_highlights(self):
        #visual cue: selected cube is darker/thicker
        for i in range(self.max_slots):
            if self.selected_slot == i:
                self.slot_cubes[i].setData(
                    pos=self.slot_cube_base_pts[i],
                    color=(0.25, 0.25, 0.25, 0.70),
                    width=3
                )
            else:
                self.slot_cubes[i].setData(
                    pos=self.slot_cube_base_pts[i],
                    color=(0.40, 0.40, 0.40, 0.45),
                    width=2
                )

        #keep camera center aligned to selection to make zoom feel intentional
        sel = self.get_selected_center()
        if sel is not None:
            self.view.opts["center"] = Vector(sel[0], sel[1], sel[2])

    def select_slot(self, slot_index: int):
        #set selection and notify the rest of the ui
        if slot_index < 0 or slot_index >= self.max_slots:
            return
        self.selected_slot = slot_index
        self._refresh_cube_highlights()
        self.slot_selected.emit(slot_index)

    def clear_slots(self):
        #wipe all slot point data and reset state
        for i in range(self.max_slots):
            self.slot_points[i].setData(pos=np.zeros((0, 3), dtype=np.float32))
            self.slot_filled[i] = False
        self.selected_slot = None
        self._refresh_cube_highlights()

    def next_empty_slot(self) -> Optional[int]:
        #find the first empty slot (used for auto-loading)
        for i in range(self.max_slots):
            if not self.slot_filled[i]:
                return i
        return None

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

    def load_ply_into_slot(self, path: str, slot_index: int):
        #load an ascii ply, fit it into the cube, color it by height, and render it
        if slot_index < 0 or slot_index >= self.max_slots:
            raise ValueError("slot_index out of range (0..8)")

        #scan header to know how many lines to skip before numeric data
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            header_lines = 0
            for line in f:
                header_lines += 1
                if line.strip() == "end_header":
                    break

        data = np.loadtxt(path, skiprows=header_lines, dtype=np.float32)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.shape[1] < 3:
            raise ValueError("ply does not contain x y z columns")

        pos = data[:, :3].astype(np.float32)

        #fit the point cloud into the slot cube
        mins = pos.min(axis=0)
        maxs = pos.max(axis=0)
        center = (mins + maxs) / 2.0
        span = (maxs - mins)
        max_span = float(np.max(span)) if float(np.max(span)) > 0 else 1.0
        fit_scale = (self.cube_size * 0.90) / max_span
        pos_local = (pos - center) * fit_scale

        #place the fitted cloud at the chosen slot center
        cx, cy, cz = self.slot_centers[slot_index]
        pos_world = pos_local + np.array([cx, cy, cz], dtype=np.float32)

        #height-based coloring uses local z (so colors are consistent across slots)
        z = pos_local[:, 2]
        t = self._robust_normalize(z)
        color = self._height_colors_smooth(t)

        self.slot_points[slot_index].setData(
            pos=pos_world.astype(np.float32),
            color=color.astype(np.float32),
            size=3.5,
            pxMode=True
        )
        self.slot_points[slot_index].setGLOptions("opaque")
        self.slot_filled[slot_index] = True
#===== SECTION 6: 3D VIEWPORT (9 SLOT GRID) END POINT =====

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
        if "Fatal" in msg:
            self._set_status_dot(False)
            self.receiver_status_lbl.setText("Receiver: NOT RUNNING")

    def _on_receiver_finished(self):
        #receiver thread ended (stop or crash)
        self._log("[receiver] receiver thread finished.")
        self._set_status_dot(False)
        self.receiver_status_lbl.setText("Receiver: NOT RUNNING")

    def _on_slot_selected(self, slot_index: int):
        #sync right-panel labels with the selection state
        self._log(f"selected slot: {slot_index + 1}")
        self.info_lbl.setText(f"Selected slot {slot_index + 1} (load a file here)")
        self.selected_slot_lbl.setText(f"Selected Slot: {slot_index + 1}")

    def _choose_target_slot(self) -> Optional[int]:
        #decide where to load next: selected slot or next empty depending on lock and occupancy
        lock = self.lock_selection_chk.isChecked()
        if self.viewport3d.selected_slot is None:
            return self.viewport3d.next_empty_slot()

        s = self.viewport3d.selected_slot
        if lock:
            return s
        if not self.viewport3d.slot_filled[s]:
            return s
        return self.viewport3d.next_empty_slot()

    def _post_load_selection_update(self, used_slot: int):
        #auto-advance selection after filling a slot unless lock is enabled
        if self.lock_selection_chk.isChecked():
            return
        if self.viewport3d.selected_slot is not None and self.viewport3d.selected_slot == used_slot:
            nxt = self.viewport3d.next_empty_slot()
            if nxt is None:
                return
            self.viewport3d.select_slot(nxt)

    def _load_path_into_target_slot(self, path: str):
        #load a file into the chosen slot and update ui state
        slot = self._choose_target_slot()
        if slot is None:
            self._log("viewer is full (9/9). clear slots to load more.")
            return

        try:
            self.viewport3d.load_ply_into_slot(path, slot)
            self._log(f"loaded into slot {slot + 1}: {os.path.basename(path)}")
            self.info_lbl.setText(f"Loaded into slot {slot + 1}:\n{os.path.basename(path)}")
            self.selected_slot_lbl.setText(f"Selected Slot: {slot + 1}")
            self._post_load_selection_update(slot)
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
            self._load_path_into_target_slot(saved_path)

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

        self.auto_load_chk = QCheckBox("Auto-load into selected slot (or next empty) after import/receive")
        self.auto_load_chk.setChecked(True)
        left.addWidget(self.auto_load_chk)
        #===== SECTION 13: LEFT COLUMN (QUEUE + STATUS) END POINT =====

        #===== SECTION 14: CENTER COLUMN (3D VIEW) START POINT =====
        center = QVBoxLayout()
        main_layout.addLayout(center, 3)

        self.viewport3d = MultiSlotPLYViewport()
        center.addWidget(self.viewport3d, 1)

        hint = QLabel("Double-click INSIDE a cube footprint to select it. Drag to orbit. Mouse wheel zooms toward selected cube.")
        hint.setWordWrap(True)
        center.addWidget(hint)
        #===== SECTION 14: CENTER COLUMN (3D VIEW) END POINT =====

        #===== SECTION 15: RIGHT COLUMN (CONTROLS + LOG) START POINT =====
        right = QVBoxLayout()
        main_layout.addLayout(right, 1)

        render_group = QGroupBox("Render Controls")
        render_layout = QVBoxLayout(render_group)

        self.lock_selection_chk = QCheckBox("Lock slot selection (replace when full)")
        self.lock_selection_chk.setChecked(False)
        self.btn_clear_slots = QPushButton("Clear 3D Slots")
        self.selected_slot_lbl = QLabel("Selected Slot: None")
        self.selected_slot_lbl.setWordWrap(True)

        for w in (self.lock_selection_chk, self.btn_clear_slots, self.selected_slot_lbl):
            render_layout.addWidget(w)

        right.addWidget(render_group)

        info_group = QGroupBox("File Info")
        info_layout = QVBoxLayout(info_group)
        self.info_lbl = QLabel("No file loaded.")
        self.info_lbl.setFixedSize(250,30)
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
        self.scan_list.itemDoubleClicked.connect(self._load_item_into_target_slot)
        self.btn_clear_slots.clicked.connect(self._clear_slots_clicked)
        self.viewport3d.slot_selected.connect(self._on_slot_selected)

    def _clear_slots_clicked(self):
        self.viewport3d.clear_slots()
        self.info_lbl.setText("Cleared all 3D slots.")
        self.selected_slot_lbl.setText("Selected Slot: None")
        self._log("cleared all 3d slots.")

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

        if self.auto_load_chk.isChecked():
            for p in file_paths:
                self._load_path_into_target_slot(p)

    def _load_item_into_target_slot(self, item: QListWidgetItem):
        #double-clicking a queued file loads it into the target slot
        path = item.data(Qt.ItemDataRole.UserRole)
        if not path or not os.path.exists(path):
            self._log("selected item has no valid file path (missing file?).")
            return
        self._load_path_into_target_slot(path)

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
