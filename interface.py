import sys
import os
import socket
import struct
from pathlib import Path
from typing import Optional, List, Tuple

import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph import Vector
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
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


def get_local_ip() -> str:
    #Best-effort: finds the LAN IP used for outbound traffic (works on most networks)
    #Does NOT send data; UDP connect is just used to learn routing interface
    ip = "127.0.0.1"
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
    except Exception:
        pass
    return ip


INBOX_DIR = get_desktop_path() / "LiDAR_Inbox"
#Receiver saves incoming .ply files here
INBOX_DIR.mkdir(parents=True, exist_ok=True)
#Create inbox once on startup
RECV_HOST = "0.0.0.0"
#Bind on all NICs so LAN devices can connect
RECV_PORT = 5001
#Must match sender port


def recv_exact(conn: socket.socket, n: int) -> bytes:
    #Reliable fixed-length read for framed protocol
    data = b""
    while len(data) < n:
        chunk = conn.recv(n - len(data))
        if not chunk:
            raise ConnectionError("Connection closed while receiving data")
        data += chunk
    return data


class ReceiverWorker(QThread):
    file_received = pyqtSignal(str)
    #Pushes saved file path to UI thread
    log = pyqtSignal(str)
    #Thread-safe logging to UI
    error = pyqtSignal(str)
    #Thread-safe error reporting
    status = pyqtSignal(bool)
    #True = running/listening, False = stopped/faulted

    def __init__(self, host: str, port: int, inbox_dir: Path, parent=None):
        super().__init__(parent)
        self.host = host
        self.port = port
        self.inbox_dir = inbox_dir
        self._running = True
        #Stop flag checked by accept loop
        self._server_socket: Optional[socket.socket] = None
        #Closed to break accept() during stop

    def stop(self):
        self._running = False
        #Request thread shutdown
        try:
            if self._server_socket:
                self._server_socket.close()
                #Force accept() to exit quickly
        except Exception:
            pass

    def run(self):
        inbox = self.inbox_dir
        inbox.mkdir(parents=True, exist_ok=True)
        #Ensure target folder exists before bind

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                self._server_socket = s
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                #Avoid "address already in use" on quick restarts
                s.bind((self.host, self.port))
                s.listen(5)
                s.settimeout(1.0)
                #Wake periodically to check _running

                self.status.emit(True)
                #Receiver successfully bound/listening

                self.log.emit(f"[Receiver] Listening on {self.host}:{self.port}")
                self.log.emit(f"[Receiver] Saving incoming files to: {inbox.resolve()}")

                while self._running:
                    try:
                        conn, addr = s.accept()
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                        #Socket likely closed by stop()

                    with conn:
                        try:
                            self.log.emit(f"[Receiver] Connection from {addr}")

                            name_len = struct.unpack("!I", recv_exact(conn, 4))[0]
                            #Read filename length (4 bytes)
                            filename = recv_exact(conn, name_len).decode("utf-8")
                            #Read filename bytes
                            file_size = struct.unpack("!Q", recv_exact(conn, 8))[0]
                            #Read file size (8 bytes)

                            if not filename.lower().endswith(".ply"):
                                self.log.emit(f"[Receiver] Rejected non-.ply file: {filename}")
                                continue
                                #Only accept expected file type

                            safe_name = os.path.basename(filename)
                            #Strip any sender-side directories
                            out_path = inbox / safe_name
                            base = out_path.stem
                            suffix = out_path.suffix
                            i = 1
                            while out_path.exists():
                                out_path = inbox / f"{base}_{i}{suffix}"
                                i += 1
                                #De-dupe if same name already exists

                            self.log.emit(f"[Receiver] Receiving: {safe_name} ({file_size} bytes)")
                            received = 0
                            with open(out_path, "wb") as f:
                                while received < file_size:
                                    chunk = conn.recv(min(65536, file_size - received))
                                    if not chunk:
                                        raise ConnectionError("Connection closed mid-transfer")
                                    f.write(chunk)
                                    received += len(chunk)
                                    #Stream bytes until expected length reached

                            self.log.emit(f"[Receiver] Saved {received} bytes -> {out_path}")
                            self.file_received.emit(str(out_path))
                            #Notify UI that a new file is ready

                        except Exception as e:
                            self.error.emit(f"[Receiver] Error: {e}")

        except Exception as e:
            self.error.emit(f"[Receiver] Fatal server error: {e}")

        finally:
            self.status.emit(False)
            #Receiver stopped (normal stop or crash)


def _normalize(v: np.ndarray) -> np.ndarray:
    #Safe unit-vector helper for camera math
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return v
    return v / n


class SlotGLView(gl.GLViewWidget):
    #GL view: double-click selects by raycast to board plane (z=0) using camera opts (no matrices)
    slot_double_clicked = pyqtSignal(int)

    def __init__(self, get_selected_center_fn, pick_slot_from_world_fn, parent=None):
        super().__init__(parent)
        self._get_selected_center_fn = get_selected_center_fn
        #Provides current selection center for zoom targeting
        self._pick_slot_from_world_fn = pick_slot_from_world_fn
        #Maps world hit point to a slot index

    def _raycast_to_plane_z0(self, x_px: float, y_px: float) -> Optional[Tuple[float, float, float]]:
        #Convert screen pixel to world hit on z=0 plane
        w = max(1, self.width())
        h = max(1, self.height())
        aspect = w / h
        #Aspect affects horizontal FOV scaling

        x_ndc = (2.0 * (x_px / w)) - 1.0
        y_ndc = 1.0 - (2.0 * (y_px / h))
        #Normalize pixel to [-1..1] clip space

        dist = float(self.opts.get("distance", 50.0))
        elev = float(self.opts.get("elevation", 0.0))
        azim = float(self.opts.get("azimuth", 0.0))
        fov = float(self.opts.get("fov", 60.0))
        center = self.opts.get("center", Vector(0, 0, 0))
        cx, cy, cz = float(center.x()), float(center.y()), float(center.z())
        #Use GLViewWidget camera parameters (no matrices)

        el = np.deg2rad(elev)
        az = np.deg2rad(azim)
        #Degrees -> radians

        cam_pos = np.array([
            cx + dist * np.cos(el) * np.cos(az),
            cy + dist * np.cos(el) * np.sin(az),
            cz + dist * np.sin(el),
        ], dtype=np.float64)
        #Camera position around center (spherical)

        target = np.array([cx, cy, cz], dtype=np.float64)
        #Camera looks toward center

        forward = _normalize(target - cam_pos)
        up_world = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        #World up axis for basis build

        right = _normalize(np.cross(forward, up_world))
        #Right vector from forward and world up
        if float(np.linalg.norm(right)) < 1e-6:
            up_world = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            right = _normalize(np.cross(forward, up_world))
            #Fallback if forward ~ parallel to up

        up = _normalize(np.cross(right, forward))
        #Orthogonal up vector in camera basis

        tan_half = np.tan(np.deg2rad(fov) / 2.0)
        dx = x_ndc * aspect * tan_half
        dy = y_ndc * tan_half
        #Project NDC into camera ray direction

        ray_dir = _normalize((right * dx) + (up * dy) + (forward * 1.0))
        #World-space ray direction

        dz = ray_dir[2]
        if abs(dz) < 1e-9:
            return None
            #No intersection if ray is parallel to plane

        t = -cam_pos[2] / dz
        if t < 0.0:
            return None
            #Ignore hits behind camera

        hit = cam_pos + t * ray_dir
        return (float(hit[0]), float(hit[1]), 0.0)
        #Return world point on z=0 plane

    def mouseDoubleClickEvent(self, ev):
        #Double-click performs slot picking without breaking orbit controls
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
        #Zoom toward selected slot by shifting camera center
        try:
            center = self._get_selected_center_fn()
            if center is not None:
                self.opts["center"] = Vector(center[0], center[1], center[2])
                #Make scroll zoom focus on selected cube
        except Exception:
            pass
        super().wheelEvent(ev)


class MultiSlotPLYViewport(QWidget):
    slot_selected = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.max_slots = 9
        #3x3 grid capacity
        self.cube_size = 22.0
        #Visual bounding box size per slot
        self.slot_spacing = self.cube_size
        #Grid spacing matches cube size

        self.slot_centers = self._build_slot_centers()
        #Precompute cube centers in world space

        self.slot_points: List[gl.GLScatterPlotItem] = []
        self.slot_cubes: List[gl.GLLinePlotItem] = []
        self.slot_cube_base_pts: List[np.ndarray] = []
        #Per-slot render objects + cached cube geometry

        self.slot_filled: List[bool] = [False] * self.max_slots
        #Tracks which slots contain data
        self.selected_slot: Optional[int] = None
        #Current selected slot index

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.view = SlotGLView(
            self.get_selected_center,
            self._pick_slot_from_world_xy
        )
        #Custom GL view with picking support
        self.view.setCameraPosition(distance=85, elevation=18, azimuth=45)
        self.view.opts["center"] = Vector(0, 0, 0)
        #Default camera framing for 3x3 grid

        try:
            self.view.setBackgroundColor("w")
        except Exception:
            self.view.opts["bgcolor"] = (255, 255, 255, 255)
            #Fallback if setBackgroundColor not available

        layout.addWidget(self.view)

        self._build_slots()
        #Create empty slot renderers
        self.view.slot_double_clicked.connect(self.select_slot)
        #Connect pick event to selection logic
        self._refresh_cube_highlights()
        #Apply initial highlight state

    def _build_slot_centers(self) -> List[Tuple[float, float, float]]:
        centers = []
        for r in range(3):
            for c in range(3):
                x = (c - 1) * self.slot_spacing
                y = (1 - r) * self.slot_spacing
                centers.append((x, y, 0.0))
        return centers
        #Centers laid out as 3x3 grid on z=0

    def get_selected_center(self) -> Optional[Tuple[float, float, float]]:
        if self.selected_slot is None:
            return None
        return self.slot_centers[self.selected_slot]
        #Provides selected cube center for zoom targeting

    def _pick_slot_from_world_xy(self, x: float, y: float) -> Optional[int]:
        #Nearest-center pick with footprint containment check
        centers_xy = np.array([(cx, cy) for (cx, cy, _) in self.slot_centers], dtype=np.float64)
        p = np.array([x, y], dtype=np.float64)

        d2 = np.sum((centers_xy - p) ** 2, axis=1)
        idx = int(np.argmin(d2))
        #Choose closest cube center

        cx, cy, _ = self.slot_centers[idx]
        half = float(self.cube_size) / 2.0

        if abs(x - cx) <= half and abs(y - cy) <= half:
            return idx
            #Only accept click inside cube footprint
        return None

    def _cube_edges(self, size: float) -> List[Tuple[np.ndarray, np.ndarray]]:
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
        #12 cube edges as endpoint pairs

    def _wire_cube_dotted(self, size: float, dash_count: int = 18) -> np.ndarray:
        pts = []
        edges = self._cube_edges(size)
        n = max(6, dash_count)
        #Ensure enough dashes to look dotted

        for a, b in edges:
            for i in range(n):
                t0 = i / n
                t1 = (i + 1) / n
                if i % 2 == 0:
                    p0 = a * (1.0 - t0) + b * t0
                    p1 = a * (1.0 - t1) + b * t1
                    pts.append(p0)
                    pts.append(p1)
                    #Alternate segments to create dash gaps

        return np.vstack(pts).astype(np.float32)
        #Line segments formatted for GLLinePlotItem

    def _build_slots(self):
        for _ in range(self.max_slots):
            scatter = gl.GLScatterPlotItem(size=3.5, pxMode=True)
            scatter.setGLOptions("opaque")
            self.view.addItem(scatter)
            self.slot_points.append(scatter)
            #One scatter renderer per slot

        for i in range(self.max_slots):
            cx, cy, cz = self.slot_centers[i]
            base = self._wire_cube_dotted(self.cube_size, dash_count=18)
            base = base + np.array([cx, cy, cz], dtype=np.float32)
            self.slot_cube_base_pts.append(base)
            #Cache cube geometry per slot

            cube = gl.GLLinePlotItem(
                pos=base,
                mode="lines",
                width=2,
                color=(0.40, 0.40, 0.40, 0.45)
            )
            cube.setGLOptions("translucent")
            self.view.addItem(cube)
            self.slot_cubes.append(cube)
            #Dotted cube outline for each slot

    def _refresh_cube_highlights(self):
        for i in range(self.max_slots):
            if self.selected_slot == i:
                self.slot_cubes[i].setData(
                    pos=self.slot_cube_base_pts[i],
                    color=(0.25, 0.25, 0.25, 0.70),
                    width=3
                )
                #Selected cube appears darker/thicker
            else:
                self.slot_cubes[i].setData(
                    pos=self.slot_cube_base_pts[i],
                    color=(0.40, 0.40, 0.40, 0.45),
                    width=2
                )
                #Unselected cubes use default style

        sel = self.get_selected_center()
        if sel is not None:
            self.view.opts["center"] = Vector(sel[0], sel[1], sel[2])
            #Keep camera centered on selection

    def select_slot(self, slot_index: int):
        if slot_index < 0 or slot_index >= self.max_slots:
            return
            #Ignore invalid index
        self.selected_slot = slot_index
        self._refresh_cube_highlights()
        self.slot_selected.emit(slot_index)
        #Broadcast selection to rest of UI

    def clear_slots(self):
        for i in range(self.max_slots):
            self.slot_points[i].setData(pos=np.zeros((0, 3), dtype=np.float32))
            self.slot_filled[i] = False
            #Clear points and mark empty
        self.selected_slot = None
        self._refresh_cube_highlights()
        #Reset selection visuals

    def next_empty_slot(self) -> Optional[int]:
        for i in range(self.max_slots):
            if not self.slot_filled[i]:
                return i
                #First available slot
        return None

    def _robust_normalize(self, z: np.ndarray) -> np.ndarray:
        #Percentile-based normalization to reduce outlier impact
        if z.size == 0:
            return z
        lo = float(np.percentile(z, 2))
        hi = float(np.percentile(z, 98))
        if (hi - lo) < 1e-6:
            lo = float(z.min())
            hi = float(z.max())
            if (hi - lo) < 1e-6:
                return np.zeros_like(z, dtype=np.float32)
                #All heights identical -> flat colors
        t = (z - lo) / (hi - lo)
        return np.clip(t, 0.0, 1.0).astype(np.float32)

    def _height_colors_smooth(self, t: np.ndarray) -> np.ndarray:
        #Smooth gradient mapping from normalized height -> RGBA
        anchors = np.array([
            [1.00, 0.92, 0.10],  #yellow
            [1.00, 0.55, 0.10],  #orange
            [0.95, 0.15, 0.15],  #red
            [0.15, 0.45, 0.95],  #blue
            [0.60, 0.25, 0.85],  #purple
            [0.15, 0.80, 0.30],  #green
        ], dtype=np.float32)
        #Color stops for interpolation

        t = np.clip(t, 0.0, 1.0)
        nseg = anchors.shape[0] - 1
        u = t * nseg
        i0 = np.floor(u).astype(np.int32)
        i0 = np.clip(i0, 0, nseg - 1)
        f = (u - i0).astype(np.float32)
        #Segment index + within-segment fraction

        c0 = anchors[i0]
        c1 = anchors[i0 + 1]
        rgb = (1.0 - f[:, None]) * c0 + f[:, None] * c1
        #Linear interpolation per point
        return np.hstack((rgb, np.ones((rgb.shape[0], 1), dtype=np.float32)))
        #Add alpha=1 for GLScatterPlotItem

    def load_ply_into_slot(self, path: str, slot_index: int):
        if slot_index < 0 or slot_index >= self.max_slots:
            raise ValueError("slot_index out of range (0..8)")
            #Guard against bad slot index

        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            header_lines = 0
            for line in f:
                header_lines += 1
                if line.strip() == "end_header":
                    break
            #Count header to skip into numeric section

        data = np.loadtxt(path, skiprows=header_lines, dtype=np.float32)
        #Loads ASCII PLY numeric body
        if data.ndim == 1:
            data = data.reshape(1, -1)
            #Ensure 2D for single-point files
        if data.shape[1] < 3:
            raise ValueError("PLY does not contain x y z columns")
            #Requires at least XYZ columns

        pos = data[:, :3].astype(np.float32)
        #Extract XYZ only

        mins = pos.min(axis=0)
        maxs = pos.max(axis=0)
        center = (mins + maxs) / 2.0
        span = (maxs - mins)
        #Bounding-box fit inputs
        max_span = float(np.max(span)) if float(np.max(span)) > 0 else 1.0
        #Avoid divide by zero

        fit_scale = (self.cube_size * 0.90) / max_span
        #Scale to fit inside cube with margin
        pos_local = (pos - center) * fit_scale
        #Center + scale around origin

        cx, cy, cz = self.slot_centers[slot_index]
        pos_world = pos_local + np.array([cx, cy, cz], dtype=np.float32)
        #Place centered cloud into chosen slot

        z = pos_local[:, 2]
        t = self._robust_normalize(z)
        color = self._height_colors_smooth(t)
        #Height-based coloring in local coordinates

        self.slot_points[slot_index].setData(
            pos=pos_world.astype(np.float32),
            color=color.astype(np.float32),
            size=3.5,
            pxMode=True
        )
        #Upload point cloud to GPU renderer
        self.slot_points[slot_index].setGLOptions("opaque")
        self.slot_filled[slot_index] = True
        #Mark slot as occupied


class LidarWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("LIDAR Image Generation & Stitching")
        self.setGeometry(250, 250, 1100, 650)

        self._closing = False
        #Used to avoid extra work during shutdown

        self._build_ui()
        self._wire_min_signals()

        self._start_receiver()
        self._log("UI initialized. Receiver is running.")

        #Refresh local IP periodically (handles Wi-Fi changes)
        self._ip_timer = QTimer(self)
        self._ip_timer.setInterval(2500)
        self._ip_timer.timeout.connect(self._refresh_ip_label)
        self._ip_timer.start()
        self._refresh_ip_label()

    def _start_receiver(self):
        self.receiver = ReceiverWorker(RECV_HOST, RECV_PORT, INBOX_DIR, parent=self)
        self.receiver.log.connect(self._log)
        self.receiver.error.connect(self._on_receiver_error)
        self.receiver.file_received.connect(self._on_file_received)
        self.receiver.finished.connect(self._on_receiver_finished)
        self.receiver.status.connect(self._on_receiver_status)
        self.receiver.start()
        #Run TCP receiver in background thread

    def _on_receiver_status(self, running: bool):
        #Green if receiver is actively listening, red if stopped/faulted
        if running:
            self.receiver_status_lbl.setText("Receiver: RUNNING")
            self._set_status_dot(True)
        else:
            self.receiver_status_lbl.setText("Receiver: NOT RUNNING")
            self._set_status_dot(False)

    def _set_status_dot(self, running: bool):
        #Glowing dot indicator via stylesheet
        if running:
            #Glowing green
            self.receiver_dot.setStyleSheet(
                "width: 14px; height: 14px; border-radius: 7px;"
                "background: #28d14c;"
                "border: 1px solid #0a7a1c;"
                "box-shadow: 0 0 10px rgba(40, 209, 76, 0.9);"
            )
        else:
            #Glowing red
            self.receiver_dot.setStyleSheet(
                "width: 14px; height: 14px; border-radius: 7px;"
                "background: #e03a3a;"
                "border: 1px solid #8a1111;"
                "box-shadow: 0 0 10px rgba(224, 58, 58, 0.9);"
            )

    def _refresh_ip_label(self):
        #Shows the laptop IP so the Pi operator can type it into the sender config
        ip = get_local_ip()
        self.ip_value_lbl.setText(f"{ip}:{RECV_PORT}")

    def _on_receiver_error(self, msg: str):
        self._log(msg)
        #Surface receiver errors in UI log
        #If there is an error that stops the receiver, ensure UI reflects "not running"
        if "Fatal" in msg:
            self._set_status_dot(False)
            self.receiver_status_lbl.setText("Receiver: NOT RUNNING")

    def _on_receiver_finished(self):
        self._log("[Receiver] Receiver thread finished.")
        self._set_status_dot(False)
        self.receiver_status_lbl.setText("Receiver: NOT RUNNING")
        #Lets user know receiver stopped

    def _on_slot_selected(self, slot_index: int):
        self._log(f"Selected slot: {slot_index + 1}")
        self.info_lbl.setText(f"Selected slot {slot_index + 1} (load a file here)")
        self.selected_slot_lbl.setText(f"Selected Slot: {slot_index + 1}")
        #UI state sync when selection changes

    def _choose_target_slot(self) -> Optional[int]:
        lock = self.lock_selection_chk.isChecked()
        #If locked, always load into selected slot

        if self.viewport3d.selected_slot is None:
            return self.viewport3d.next_empty_slot()
            #No selection -> pick next empty

        s = self.viewport3d.selected_slot
        if lock:
            return s
        if not self.viewport3d.slot_filled[s]:
            return s
            #Selected slot empty -> use it
        return self.viewport3d.next_empty_slot()
        #Selected slot full -> fallback to next empty

    def _post_load_selection_update(self, used_slot: int):
        if self.lock_selection_chk.isChecked():
            return
            #Keep selection fixed when locked
        if self.viewport3d.selected_slot is not None and self.viewport3d.selected_slot == used_slot:
            nxt = self.viewport3d.next_empty_slot()
            if nxt is None:
                return
            self.viewport3d.select_slot(nxt)
            #Auto-advance selection after filling a slot

    def _load_path_into_target_slot(self, path: str):
        slot = self._choose_target_slot()
        if slot is None:
            self._log("Viewer is full (9/9). Clear slots to load more.")
            return

        try:
            self.viewport3d.load_ply_into_slot(path, slot)
            self._log(f"Loaded into slot {slot + 1}: {os.path.basename(path)}")

            self.info_lbl.setText(f"Loaded into slot {slot + 1}:\n{os.path.basename(path)}")
            self.selected_slot_lbl.setText(f"Selected Slot: {slot + 1}")
            self._post_load_selection_update(slot)
            #Keep UI labels + selection consistent

        except Exception as e:
            self._log(f"Load failed: {e}")

    def _on_file_received(self, saved_path: str):
        filename = os.path.basename(saved_path)

        item = QListWidgetItem(filename)
        item.setData(Qt.ItemDataRole.UserRole, saved_path)
        self.scan_list.addItem(item)
        #Queue received file in UI list

        self._log(f"Received -> Desktop/LiDAR_Inbox: {filename}")

        if self.auto_load_chk.isChecked():
            self._load_path_into_target_slot(saved_path)
            #Auto-load newest receive into a slot

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)

        main_layout = QHBoxLayout(root)

        left = QVBoxLayout()
        main_layout.addLayout(left, 1)

        # --- Receiver status + IP display (TOP OF LEFT COLUMN) ---
        status_row = QHBoxLayout()

        self.receiver_dot = QLabel("")
        #Dot is styled via stylesheet to glow
        self.receiver_dot.setFixedSize(14, 14)

        self.receiver_status_lbl = QLabel("Receiver: NOT RUNNING")
        self.receiver_status_lbl.setStyleSheet("font-weight: bold;")

        status_row.addWidget(self.receiver_dot)
        status_row.addWidget(self.receiver_status_lbl)
        status_row.addStretch(1)

        left.addLayout(status_row)
        self._set_status_dot(False)
        #Default to red until worker reports running

        ip_row = QHBoxLayout()
        ip_title = QLabel("Laptop IP:")
        ip_title.setStyleSheet("font-weight: bold;")
        self.ip_value_lbl = QLabel("0.0.0.0:----")
        self.ip_value_lbl.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        #Selectable so you can copy/paste if needed
        ip_row.addWidget(ip_title)
        ip_row.addWidget(self.ip_value_lbl, 1)
        left.addLayout(ip_row)

        left.addSpacing(8)
        # --- End status/IP section ---

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

        center = QVBoxLayout()
        main_layout.addLayout(center, 3)

        self.viewport3d = MultiSlotPLYViewport()
        center.addWidget(self.viewport3d, 1)

        hint = QLabel("Double-click INSIDE a cube footprint to select it. Drag to orbit. Mouse wheel zooms toward selected cube.")
        hint.setWordWrap(True)
        center.addWidget(hint)
        #User instruction for selection + navigation

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

    def _wire_min_signals(self):
        self.btn_import_local.clicked.connect(self._import_local_clicked)
        self.btn_delete.clicked.connect(self._delete_selected_clicked)
        self.scan_list.itemDoubleClicked.connect(self._load_item_into_target_slot)
        self.btn_clear_slots.clicked.connect(self._clear_slots_clicked)
        self.viewport3d.slot_selected.connect(self._on_slot_selected)
        #Minimal UI wiring for queue + slot selection

    def _clear_slots_clicked(self):
        self.viewport3d.clear_slots()
        self.info_lbl.setText("Cleared all 3D slots.")
        self.selected_slot_lbl.setText("Selected Slot: None")
        self._log("Cleared all 3D slots.")
        #Reset UI state after clearing

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
            self.scan_list.addItem(item)
            self._log(f"Imported local file: {filename}")
            #Queue imports for later load

        if self.auto_load_chk.isChecked():
            for p in file_paths:
                self._load_path_into_target_slot(p)
                #Bulk auto-load imported scans

    def _load_item_into_target_slot(self, item: QListWidgetItem):
        path = item.data(Qt.ItemDataRole.UserRole)
        if not path or not os.path.exists(path):
            self._log("Selected item has no valid file path (missing file?).")
            return
        self._load_path_into_target_slot(path)
        #Load chosen queued scan into viewer

    def _delete_selected_clicked(self):
        for item in self.scan_list.selectedItems():
            row = self.scan_list.row(item)
            self.scan_list.takeItem(row)
            #Removes from UI list only
        self._log("Deleted selected scan(s) from queue.")

    def _log(self, msg: str):
        from time import strftime
        ts = strftime("%H:%M:%S")
        current = self.log_lbl.text()
        self.log_lbl.setText((f"[{ts}] {msg}\n" + current)[:4000])
        #Prepend newest log and cap size

    def closeEvent(self, event):
        self._closing = True
        #Signals shutdown path
        try:
            if hasattr(self, "receiver") and self.receiver.isRunning():
                self._log("Stopping receiver...")
                self.receiver.stop()
                self.receiver.wait(1500)
                #Stop thread before window closes
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
