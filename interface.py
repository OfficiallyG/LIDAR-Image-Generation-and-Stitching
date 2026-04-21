#===== SECTION 1: IMPORTS START POINT =====
# #import sys for argv and clean app exit
import sys
#import os for safe filename handling and file existence checks
import os
#import re for compact 3-digit file naming with tags
import re
#import socket for tcp server and local ip discovery
import socket
#import struct for packing/unpacking fixed-size integers in the transfer protocol
import struct
#import tempfile for safe temporary stitched file names during multi-file merges
import tempfile
#import copy for open3d point cloud duplication during stitching
import copy
#import pathlib for safe cross-platform path building
from pathlib import Path
#import typing for clearer intent in function signatures
from typing import Optional, List, Tuple, Dict, Any

#import numpy for fast math and array operations for point clouds
import numpy as np
#import open3d lazily inside stitch functions so the app can still open even if stitching deps are missing
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
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QListWidget,
    QListWidgetItem,
    QFileDialog,
    QMessageBox,
    QAbstractItemView,
    QCheckBox,
    QSizePolicy
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

class NullProgress:
    #no-op progress helper so long operations can keep calling progress methods without showing a bar
    def setRange(self, *args, **kwargs):
        pass

    def setValue(self, *args, **kwargs):
        pass

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
                            out_path = build_short_raw_receive_path(inbox)

                            self.log.emit(f"[receiver] receiving: {safe_name} -> {out_path.name} ({file_size} bytes)")

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


def _get_structured_channel(data: np.ndarray, names: List[str]) -> Optional[np.ndarray]:
    #return the first available named channel from a structured array
    for name in names:
        if name in data.dtype.names:
            return np.asarray(data[name])
    return None


def read_ply_data(path: str) -> Dict[str, Any]:
    #read xyz vertices and optional rgb/intensity from ascii or binary little-endian ply files
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
            n_cols = len(prop_names)
            rows = []
            for _ in range(vertex_count):
                raw = f.readline()
                if not raw:
                    break
                try:
                    vals = [float(v) for v in raw.split()]
                except ValueError:
                    continue
                if len(vals) == n_cols:
                    rows.append(vals)
            if not rows:
                raise ValueError(f"no valid vertex rows found in ascii ply: {os.path.basename(path)}")
            data = np.array(rows, dtype=np.float64)
            if data.ndim == 1:
                data = data.reshape(1, -1)
            xyz = data[:, [prop_names.index("x"), prop_names.index("y"), prop_names.index("z")]].astype(np.float32)

            rgb = None
            rgb_names = ["red", "green", "blue"]
            if all(name in prop_names for name in rgb_names):
                rgb = data[:, [prop_names.index("red"), prop_names.index("green"), prop_names.index("blue")]]
                rgb = np.clip(rgb, 0, 255).astype(np.uint8)

            intensity = None
            for name in ["intensity", "reflectivity"]:
                if name in prop_names:
                    intensity = data[:, prop_names.index(name)].astype(np.float32)
                    break

            return {"xyz": xyz, "rgb": rgb, "intensity": intensity}

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

        rgb = None
        rgb_channels = [
            _get_structured_channel(data, ["red", "r"]),
            _get_structured_channel(data, ["green", "g"]),
            _get_structured_channel(data, ["blue", "b"]),
        ]
        if all(ch is not None for ch in rgb_channels):
            rgb = np.column_stack(rgb_channels)
            rgb = np.clip(rgb, 0, 255).astype(np.uint8)

        intensity = _get_structured_channel(data, ["intensity", "reflectivity"])
        if intensity is not None:
            intensity = intensity.astype(np.float32)

        return {"xyz": xyz.astype(np.float32), "rgb": rgb, "intensity": intensity}


def read_ply_xyz(path: str) -> np.ndarray:
    #backward-compatible xyz-only loader
    return read_ply_data(path)["xyz"]


def write_ply_xyzrgb_ascii(path: str, xyz: np.ndarray, rgb: Optional[np.ndarray] = None):
    #save an edited point cloud as ascii ply with optional rgb colors
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("xyz must be an Nx3 array")

    with open(path, "w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {xyz.shape[0]}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        use_rgb = rgb is not None and len(rgb) == len(xyz)
        if use_rgb:
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
        f.write("end_header\n")

        if use_rgb:
            rgb = np.clip(np.asarray(rgb), 0, 255).astype(np.uint8)
            for p, c in zip(xyz, rgb):
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {int(c[0])} {int(c[1])} {int(c[2])}\n")
        else:
            for p in xyz:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def _normalize_unit(values: np.ndarray) -> np.ndarray:
    #normalize a 1d array into the 0..1 range
    values = np.asarray(values, dtype=np.float32)
    if values.size == 0:
        return values
    lo = float(np.percentile(values, 2))
    hi = float(np.percentile(values, 98))
    if (hi - lo) < 1e-6:
        lo = float(values.min())
        hi = float(values.max())
        if (hi - lo) < 1e-6:
            return np.zeros_like(values, dtype=np.float32)
    return np.clip((values - lo) / (hi - lo), 0.0, 1.0).astype(np.float32)


def _smooth_gradient_colors(t: np.ndarray) -> np.ndarray:
    #use a perceptually smoother palette so rooms are easier to read on a dark background
    #this is a viridis-style ramp: dark purple -> blue -> teal -> green -> yellow
    anchors = np.array([
        [0.267, 0.005, 0.329],
        [0.283, 0.141, 0.458],
        [0.254, 0.265, 0.530],
        [0.207, 0.372, 0.553],
        [0.164, 0.471, 0.558],
        [0.128, 0.567, 0.551],
        [0.135, 0.659, 0.518],
        [0.267, 0.749, 0.441],
        [0.478, 0.821, 0.318],
        [0.741, 0.873, 0.150],
        [0.993, 0.906, 0.144],
    ], dtype=np.float32)
    t = np.clip(np.asarray(t, dtype=np.float32), 0.0, 1.0)
    #slightly brighten the midrange so floor, desks, and walls separate more clearly
    t = np.power(t, 0.85, dtype=np.float32)
    nseg = anchors.shape[0] - 1
    u = t * nseg
    i0 = np.floor(u).astype(np.int32)
    i0 = np.clip(i0, 0, nseg - 1)
    frac = (u - i0).astype(np.float32)
    rgb = (1.0 - frac[:, None]) * anchors[i0] + frac[:, None] * anchors[i0 + 1]
    return np.clip(rgb * 255.0, 0, 255).astype(np.uint8)


def voxel_downsample_numpy(
    xyz: np.ndarray,
    rgb: Optional[np.ndarray] = None,
    intensity: Optional[np.ndarray] = None,
    voxel_size: float = 0.05,
) -> Tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
    #merge nearby points into voxel centroids so scans render cleaner and faster
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.size == 0:
        return xyz, rgb, intensity
    voxel_size = max(1e-4, float(voxel_size))
    mins = xyz.min(axis=0)
    keys = np.floor((xyz - mins) / voxel_size).astype(np.int32)
    uniq, inverse = np.unique(keys, axis=0, return_inverse=True)

    counts = np.bincount(inverse).astype(np.float32)
    out_xyz = np.zeros((len(uniq), 3), dtype=np.float32)
    for dim in range(3):
        out_xyz[:, dim] = np.bincount(inverse, weights=xyz[:, dim], minlength=len(uniq)) / counts

    out_rgb = None
    if rgb is not None and len(rgb) == len(xyz):
        rgbf = rgb.astype(np.float32)
        out_rgb = np.zeros((len(uniq), 3), dtype=np.float32)
        for dim in range(3):
            out_rgb[:, dim] = np.bincount(inverse, weights=rgbf[:, dim], minlength=len(uniq)) / counts
        out_rgb = np.clip(out_rgb, 0, 255).astype(np.uint8)

    out_intensity = None
    if intensity is not None and len(intensity) == len(xyz):
        intensityf = np.asarray(intensity, dtype=np.float32)
        out_intensity = np.bincount(inverse, weights=intensityf, minlength=len(uniq)) / counts
        out_intensity = out_intensity.astype(np.float32)

    return out_xyz, out_rgb, out_intensity


def statistical_outlier_filter_numpy(
    xyz: np.ndarray,
    rgb: Optional[np.ndarray] = None,
    intensity: Optional[np.ndarray] = None,
    k: int = 12,
    z_thresh: float = 1.2,
) -> Tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray]]:
    #remove stray points using local neighbor distances without external deps
    xyz = np.asarray(xyz, dtype=np.float32)
    if len(xyz) <= max(8, k + 1):
        return xyz, rgb, intensity

    sample_cap = 2500
    if len(xyz) > sample_cap:
        idx = np.linspace(0, len(xyz) - 1, sample_cap, dtype=np.int32)
        ref = xyz[idx]
    else:
        ref = xyz

    d2 = np.sum((xyz[:, None, :] - ref[None, :, :]) ** 2, axis=2)
    kk = min(k + 1, d2.shape[1])
    nearest = np.partition(d2, kk - 1, axis=1)[:, :kk]
    mean_neighbor_dist = np.sqrt(np.maximum(nearest[:, 1:], 0.0)).mean(axis=1)
    mu = float(mean_neighbor_dist.mean())
    sigma = float(mean_neighbor_dist.std())
    limit = mu + max(1e-6, z_thresh * sigma)
    keep = mean_neighbor_dist <= limit

    out_xyz = xyz[keep]
    out_rgb = rgb[keep] if rgb is not None and len(rgb) == len(xyz) else None
    out_intensity = intensity[keep] if intensity is not None and len(intensity) == len(xyz) else None
    return out_xyz, out_rgb, out_intensity


def _safe_take_optional(values: Optional[np.ndarray], keep: np.ndarray) -> Optional[np.ndarray]:
    #slice an optional 1d channel only when it matches the point count
    if values is None:
        return None
    values = np.asarray(values)
    if len(values) != len(keep):
        return None
    return values[keep]


def _normalize_vector(vec: np.ndarray) -> np.ndarray:
    #return a unit-length copy of a vector
    vec = np.asarray(vec, dtype=np.float64)
    norm = float(np.linalg.norm(vec))
    if norm < 1e-12:
        raise ValueError("cannot normalize a near-zero vector")
    return (vec / norm).astype(np.float64)


def _rotation_matrix_from_vectors(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    #build the shortest-arc rotation that maps one direction vector onto another
    a = _normalize_vector(source)
    b = _normalize_vector(target)
    v = np.cross(a, b)
    c = float(np.clip(np.dot(a, b), -1.0, 1.0))
    s = float(np.linalg.norm(v))

    if s < 1e-10:
        if c > 0.0:
            return np.eye(3, dtype=np.float64)

        axis_guess = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(a[0]) > 0.9:
            axis_guess = np.array([0.0, 1.0, 0.0], dtype=np.float64)

        axis = _normalize_vector(np.cross(a, axis_guess))
        outer = np.outer(axis, axis)
        return (-np.eye(3, dtype=np.float64) + 2.0 * outer).astype(np.float64)

    vx = np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ], dtype=np.float64)
    return (np.eye(3, dtype=np.float64) + vx + (vx @ vx) * ((1.0 - c) / (s * s))).astype(np.float64)


def _rotation_matrix_about_z(theta_rad: float) -> np.ndarray:
    #build a standard yaw rotation matrix around the z-axis
    c = float(np.cos(theta_rad))
    s = float(np.sin(theta_rad))
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


def _apply_rotation(xyz: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    #rotate Nx3 points using a 3x3 rotation matrix
    xyz = np.asarray(xyz, dtype=np.float32)
    rotation = np.asarray(rotation, dtype=np.float64)
    return (xyz.astype(np.float64) @ rotation.T).astype(np.float32)


def _fit_plane_from_points(points: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
    #solve ax+by+cz+d=0 for three sample points
    if len(points) != 3:
        return None
    p0, p1, p2 = points
    normal = np.cross(p1 - p0, p2 - p0)
    norm = float(np.linalg.norm(normal))
    if norm < 1e-8:
        return None
    normal = normal / norm
    d = -float(np.dot(normal, p0))
    return normal.astype(np.float64), d


def _sample_for_ransac(xyz: np.ndarray, cap: int = 7000) -> np.ndarray:
    #downsample the candidate pool for faster plane scoring while preserving broad coverage
    xyz = np.asarray(xyz, dtype=np.float32)
    if len(xyz) <= cap:
        return xyz
    rng = np.random.default_rng(12345)
    idx = rng.choice(len(xyz), size=cap, replace=False)
    return xyz[np.sort(idx)]


def _robust_axis_span(values: np.ndarray) -> float:
    #measure a stable extent while ignoring extreme outliers
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return 0.0
    lo = float(np.percentile(values, 5))
    hi = float(np.percentile(values, 95))
    return max(hi - lo, 0.0)


def _score_plane_candidate(all_xyz: np.ndarray, normal: np.ndarray, d: float, threshold: float) -> Optional[Dict[str, Any]]:
    #score a plane using full-cloud support, one-sided occupancy, and 2d floor-like spread
    signed = all_xyz @ normal + d
    distances = np.abs(signed)
    inliers = distances <= threshold
    inlier_count = int(inliers.sum())
    if inlier_count < 80:
        return None

    inlier_xyz = all_xyz[inliers]
    rotation = _rotation_matrix_from_vectors(normal, np.array([0.0, 0.0, 1.0], dtype=np.float64))
    inlier_leveled = _apply_rotation(inlier_xyz, rotation)
    span_x = _robust_axis_span(inlier_leveled[:, 0])
    span_y = _robust_axis_span(inlier_leveled[:, 1])
    plane_area = float(span_x * span_y)
    if plane_area < 0.10:
        return None

    pos_ratio = float((signed > threshold).mean())
    neg_ratio = float((signed < -threshold).mean())

    oriented_normal = np.asarray(normal, dtype=np.float64)
    oriented_d = float(d)
    if neg_ratio > pos_ratio:
        oriented_normal = -oriented_normal
        oriented_d = -oriented_d
        signed = -signed
        pos_ratio, neg_ratio = neg_ratio, pos_ratio

    one_sidedness = max(pos_ratio - neg_ratio, 0.0)

    #weak tie-breaker only; does not drive the result by itself
    z = all_xyz[:, 2]
    raw_low = float(np.percentile(z, 10))
    raw_high = float(np.percentile(z, 90))
    raw_span = max(raw_high - raw_low, 1e-6)
    inlier_z = float(np.median(inlier_xyz[:, 2]))
    low_bias = 1.0 - np.clip((inlier_z - raw_low) / raw_span, 0.0, 1.0)

    score = (inlier_count * 10.0) + (plane_area * 260.0) + (one_sidedness * 900.0) + (low_bias * 40.0)
    return {
        "normal": oriented_normal.astype(np.float64),
        "d": oriented_d,
        "threshold": threshold,
        "subset_inlier_mask": inliers,
        "subset_inlier_count": inlier_count,
        "score": float(score),
        "subset_xyz": all_xyz,
        "plane_area": plane_area,
        "one_sidedness": one_sidedness,
    }


def detect_floor_plane_ransac_numpy(xyz: np.ndarray, iterations: int = 1200, distance_threshold: Optional[float] = None) -> Dict[str, Any]:
    #find the dominant room-base plane using full-cloud ransac instead of raw-z cropping
    xyz = np.asarray(xyz, dtype=np.float32)
    if len(xyz) < 50:
        raise ValueError("not enough points for floor detection")

    sampled_xyz = _sample_for_ransac(xyz, cap=7000)
    spans = np.percentile(sampled_xyz, 95, axis=0) - np.percentile(sampled_xyz, 5, axis=0)
    span = float(max(np.max(spans), 1e-6))
    threshold = float(distance_threshold) if distance_threshold is not None else float(np.clip(span / 120.0, 0.010, 0.045))

    best = None
    best_score = -1.0
    rng = np.random.default_rng()

    for _ in range(iterations):
        sample_idx = rng.choice(len(sampled_xyz), size=3, replace=False)
        fit = _fit_plane_from_points(sampled_xyz[sample_idx])
        if fit is None:
            continue
        normal, d = fit
        scored = _score_plane_candidate(sampled_xyz, normal, d, threshold)
        if scored is None:
            continue
        if scored["score"] > best_score:
            best_score = scored["score"]
            best = scored

    if best is None:
        centered = sampled_xyz - sampled_xyz.mean(axis=0, keepdims=True)
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        normal = _normalize_vector(vh[-1])
        d = -float(np.dot(normal, sampled_xyz.mean(axis=0)))
        best = {
            "normal": normal,
            "d": d,
            "threshold": threshold,
            "subset_inlier_mask": np.ones(len(sampled_xyz), dtype=bool),
            "subset_inlier_count": len(sampled_xyz),
            "subset_xyz": sampled_xyz,
            "score": 0.0,
        }

    return best


def remove_ceiling_only_numpy(xyz: np.ndarray, rgb: Optional[np.ndarray] = None, intensity: Optional[np.ndarray] = None) -> Tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
    #keep the room and floor intact, but trim only the highest ceiling band
    xyz = np.asarray(xyz, dtype=np.float32)
    if len(xyz) < 100:
        floor_ref = float(np.percentile(xyz[:, 2], 3)) if len(xyz) else 0.0
        height = xyz[:, 2] - floor_ref if len(xyz) else None
        return xyz, rgb, intensity, height

    z = xyz[:, 2]
    low_ref = float(np.percentile(z, 3))
    high_p = float(np.percentile(z, 96))
    room_span = max(high_p - low_ref, 1e-6)

    ceiling_margin = float(np.clip(room_span * 0.06, 0.05, 0.24))
    high_cut = high_p - ceiling_margin
    keep = z <= high_cut

    if keep.sum() < max(50, int(len(xyz) * 0.50)):
        high_cut = float(np.percentile(z, 98))
        keep = z <= high_cut

    if keep.sum() < max(50, int(len(xyz) * 0.50)):
        return xyz, rgb, intensity, z - low_ref

    out_xyz = xyz[keep]
    out_rgb = rgb[keep] if rgb is not None and len(rgb) == len(xyz) else None
    out_intensity = intensity[keep] if intensity is not None and len(intensity) == len(xyz) else None
    floor_ref = float(np.percentile(out_xyz[:, 2], 3))
    return out_xyz, out_rgb, out_intensity, out_xyz[:, 2] - floor_ref


def detect_dominant_wall_yaw_numpy(xyz: np.ndarray) -> float:
    #estimate the strongest room-wall heading from a leveled scan
    xyz = np.asarray(xyz, dtype=np.float32)
    if len(xyz) < 50:
        return 0.0

    z = xyz[:, 2]
    z_low = float(np.percentile(z, 20))
    z_high = float(np.percentile(z, 80))
    wall_band = xyz[(z >= z_low) & (z <= z_high)]
    if len(wall_band) < 50:
        wall_band = xyz

    xy = wall_band[:, :2].astype(np.float64)
    xy_centered = xy - xy.mean(axis=0, keepdims=True)

    cov = np.cov(xy_centered.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    main_vec = eigvecs[:, int(np.argmax(eigvals))]
    theta = float(np.arctan2(main_vec[1], main_vec[0]))

    candidates = [theta, theta + np.pi / 2.0]
    normalized_candidates = [((angle + np.pi / 4.0) % (np.pi / 2.0)) - np.pi / 4.0 for angle in candidates]
    best_angle = min(normalized_candidates, key=lambda angle: abs(angle))
    return float(best_angle)


def _slice_plane_strength(xyz: np.ndarray, low_q: float, high_q: float) -> float:
    #estimate how much of a broad horizontal plane exists inside a thin height slice
    xyz = np.asarray(xyz, dtype=np.float32)
    if len(xyz) < 50:
        return 0.0
    z = xyz[:, 2]
    z_low = float(np.percentile(z, low_q))
    z_high = float(np.percentile(z, high_q))
    slab = xyz[(z >= z_low) & (z <= z_high)]
    if len(slab) < 25:
        return 0.0
    span_x = _robust_axis_span(slab[:, 0])
    span_y = _robust_axis_span(slab[:, 1])
    area = float(span_x * span_y)
    density = float(len(slab))
    return area * max(density, 1.0)


def _vertical_density_balance(xyz: np.ndarray) -> Dict[str, float]:
    #compare lower-room density to upper-room density so upside-down results can be detected from real room content
    xyz = np.asarray(xyz, dtype=np.float32)
    if len(xyz) == 0:
        return {"low": 0.0, "high": 0.0, "very_low": 0.0, "very_high": 0.0, "metric": 0.0}

    z = xyz[:, 2]
    z_lo = float(np.percentile(z, 2))
    z_hi = float(np.percentile(z, 98))
    span = max(z_hi - z_lo, 1e-6)
    zn = np.clip((z - z_lo) / span, 0.0, 1.0)

    low = float(np.sum((zn >= 0.00) & (zn <= 0.30)))
    high = float(np.sum((zn >= 0.70) & (zn <= 1.00)))
    very_low = float(np.sum(zn <= 0.12))
    very_high = float(np.sum(zn >= 0.88))
    metric = (low - high) + 0.5 * (very_low - very_high)
    return {
        "low": low,
        "high": high,
        "very_low": very_low,
        "very_high": very_high,
        "metric": float(metric),
    }


def should_flip_stabilized_scan_180(xyz: np.ndarray) -> bool:
    #flip automatically when the stabilized room has more structure near the top than near the bottom
    xyz = np.asarray(xyz, dtype=np.float32)
    if len(xyz) < 100:
        return False

    balance = _vertical_density_balance(xyz)
    return bool(balance["metric"] < 0.0)


def stabilize_scan_to_floor_frame_numpy(
    xyz: np.ndarray,
    rgb: Optional[np.ndarray] = None,
    intensity: Optional[np.ndarray] = None,
) -> Dict[str, Any]:
    #anchor the scan to the detected floor, square it to the room, then force a 180-degree flip so the floor lands down
    xyz = np.asarray(xyz, dtype=np.float32)
    if len(xyz) < 50:
        floor_ref = float(np.percentile(xyz[:, 2], 3)) if len(xyz) else 0.0
        return {
            "xyz": xyz,
            "rgb": rgb,
            "intensity": intensity,
            "height_from_floor": xyz[:, 2] - floor_ref if len(xyz) else np.zeros((0,), dtype=np.float32),
            "floor_normal": np.array([0.0, 0.0, 1.0], dtype=np.float32),
            "rotation_level": np.eye(3, dtype=np.float32),
            "rotation_yaw": np.eye(3, dtype=np.float32),
            "rotation_flip": np.eye(3, dtype=np.float32),
            "floor_shift": 0.0,
            "wall_yaw_rad": 0.0,
            "auto_flipped": False,
        }

    floor_fit = detect_floor_plane_ransac_numpy(xyz)
    floor_normal = floor_fit["normal"]
    rotation_level = _rotation_matrix_from_vectors(floor_normal, np.array([0.0, 0.0, 1.0], dtype=np.float64))
    leveled_xyz = _apply_rotation(xyz, rotation_level)

    floor_subset_rotated = _apply_rotation(floor_fit["subset_xyz"], rotation_level)
    subset_mask = floor_fit.get("subset_inlier_mask")
    if subset_mask is not None and len(subset_mask) == len(floor_subset_rotated) and subset_mask.any():
        floor_z = float(np.median(floor_subset_rotated[subset_mask, 2]))
    else:
        floor_z = float(np.percentile(leveled_xyz[:, 2], 3))
    leveled_xyz[:, 2] -= floor_z

    yaw_angle = detect_dominant_wall_yaw_numpy(leveled_xyz)
    rotation_yaw = _rotation_matrix_about_z(-yaw_angle)
    stabilized_xyz = _apply_rotation(leveled_xyz, rotation_yaw)

    #always apply a 180-degree flip after leveling/yaw alignment so the result comes back floor-down
    rotation_flip = np.array([
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ], dtype=np.float64)
    stabilized_xyz = _apply_rotation(stabilized_xyz, rotation_flip)
    auto_flipped = True

    floor_ref = float(np.percentile(stabilized_xyz[:, 2], 1))
    stabilized_xyz[:, 2] -= floor_ref

    xy_center = np.median(stabilized_xyz[:, :2], axis=0)
    stabilized_xyz[:, 0] -= float(xy_center[0])
    stabilized_xyz[:, 1] -= float(xy_center[1])

    height_from_floor = stabilized_xyz[:, 2].astype(np.float32)
    return {
        "xyz": stabilized_xyz,
        "rgb": rgb,
        "intensity": intensity,
        "height_from_floor": height_from_floor,
        "floor_normal": floor_normal.astype(np.float32),
        "rotation_level": rotation_level.astype(np.float32),
        "rotation_yaw": rotation_yaw.astype(np.float32),
        "rotation_flip": rotation_flip.astype(np.float32),
        "floor_shift": float(floor_z),
        "wall_yaw_rad": float(yaw_angle),
        "auto_flipped": bool(auto_flipped),
    }


def build_edited_colors(xyz: np.ndarray, intensity: Optional[np.ndarray] = None, height_from_floor: Optional[np.ndarray] = None) -> np.ndarray:
    #create a strong tactical-style color pass for the edited scan
    xyz = np.asarray(xyz, dtype=np.float32)
    if height_from_floor is None or len(height_from_floor) != len(xyz):
        height_from_floor = xyz[:, 2]

    height_t = _normalize_unit(height_from_floor)
    dist = np.linalg.norm(xyz[:, :2], axis=1)
    dist_t = _normalize_unit(dist)

    if intensity is not None and len(intensity) == len(xyz):
        intensity_t = _normalize_unit(intensity)
        blend = (0.55 * height_t) + (0.20 * (1.0 - dist_t)) + (0.25 * intensity_t)
    else:
        blend = (0.70 * height_t) + (0.30 * (1.0 - dist_t))

    return _smooth_gradient_colors(blend)


def estimate_ceiling_cut_height(z_values: np.ndarray) -> Optional[float]:
    #estimate the lower boundary of the densest high-z slab so ceiling points can be hidden or removed
    z_values = np.asarray(z_values, dtype=np.float32)
    if z_values.size < 100:
        return None

    z_low = float(np.percentile(z_values, 2))
    z_high = float(np.percentile(z_values, 98))
    span = max(z_high - z_low, 1e-6)
    if span < 0.20:
        return None

    focus = z_values[z_values >= (z_low + 0.55 * span)]
    if focus.size < 50:
        return None

    bins = int(np.clip(span / 0.05, 12, 40))
    hist, edges = np.histogram(focus, bins=bins, range=(z_low + 0.55 * span, z_high))
    if hist.size == 0 or int(hist.max()) < 10:
        return None

    peak = int(np.argmax(hist))
    peak_height = int(hist[peak])
    run_start = peak
    run_end = peak
    threshold = max(4, int(peak_height * 0.45))

    while run_start > 0 and hist[run_start - 1] >= threshold:
        run_start -= 1
    while run_end < (len(hist) - 1) and hist[run_end + 1] >= threshold:
        run_end += 1

    cut_height = float(edges[run_start])
    if cut_height <= (z_low + 0.45 * span):
        cut_height = float(z_low + 0.82 * span)
    return cut_height


def remove_detected_ceiling_numpy(
    xyz: np.ndarray,
    rgb: Optional[np.ndarray] = None,
    intensity: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, Optional[np.ndarray], Optional[np.ndarray], Optional[float]]:
    #remove only the detected ceiling slab while keeping walls and room contents intact
    xyz = np.asarray(xyz, dtype=np.float32)
    if len(xyz) == 0:
        return xyz, rgb, intensity, None

    cut_height = estimate_ceiling_cut_height(xyz[:, 2])
    if cut_height is None:
        return xyz, rgb, intensity, None

    keep = xyz[:, 2] < cut_height
    if int(np.count_nonzero(keep)) < max(100, int(len(xyz) * 0.55)):
        return xyz, rgb, intensity, None

    out_xyz = xyz[keep]
    out_rgb = rgb[keep] if rgb is not None and len(rgb) == len(xyz) else None
    out_intensity = intensity[keep] if intensity is not None and len(intensity) == len(xyz) else None
    return out_xyz, out_rgb, out_intensity, float(cut_height)



def _extract_short_base_and_tags(stem: str) -> Tuple[Optional[str], List[str]]:
    #read names like 001, 001-f, 001-f-c, 001-x, etc.
    m = re.fullmatch(r"(\d{3})(?:-([a-z](?:-[a-z])*))?", stem.lower())
    if not m:
        return None, []

    base = m.group(1)
    tag_str = m.group(2)
    tags = tag_str.split("-") if tag_str else []
    return base, tags


def _next_short_base(parent: Path) -> str:
    #find next available 3-digit id in this folder
    used = set()

    if parent.exists():
        for p in parent.glob("*.ply"):
            m = re.match(r"^(\d{3})(?:-[a-z](?:-[a-z])*)?$", p.stem.lower())
            if m:
                used.add(int(m.group(1)))

    for i in range(1, 1000):
        if i not in used:
            return f"{i:03d}"

    raise ValueError("no free 3-digit filenames left in this folder")


def build_short_output_path(input_path: str, add_tags: List[str]) -> str:
    #create compact names like 001.ply, 001-f.ply, 001-c.ply, 001-f-c.ply
    src = Path(input_path)
    parent = src.parent if src.parent.exists() else INBOX_DIR

    base, existing_tags = _extract_short_base_and_tags(src.stem)
    if base is None:
        base = _next_short_base(parent)

    tags = list(existing_tags)
    for tag in add_tags:
        tag = tag.lower().strip()
        if tag and tag not in tags:
            tags.append(tag)

    stem = base if not tags else f"{base}-{'-'.join(tags)}"
    output_path = parent / f"{stem}{src.suffix}"

    #if exact name already exists, move to a fresh 3-digit base
    if output_path.exists():
        base = _next_short_base(parent)
        stem = base if not tags else f"{base}-{'-'.join(tags)}"
        output_path = parent / f"{stem}{src.suffix}"

    return str(output_path)


def build_short_raw_receive_path(inbox_dir: Path) -> Path:
    #assign the next raw incoming scan name like 001.ply, 002.ply, etc.
    base = _next_short_base(inbox_dir)
    return inbox_dir / f"{base}.ply"


def cleanup_scan_to_new_file(input_path: str) -> str:
    #clean a scan gently so room detail is preserved while obvious floaters are removed
    data = read_ply_data(input_path)
    xyz = data["xyz"]
    rgb = data.get("rgb")
    intensity = data.get("intensity")

    if xyz.size == 0:
        raise ValueError("scan contains no points")

    mins = xyz.min(axis=0)
    maxs = xyz.max(axis=0)
    span = np.maximum(maxs - mins, 1e-6)
    max_span = float(np.max(span))

    #use a lighter downsample than before so desks, corners, and wall detail survive better
    voxel_size = float(np.clip(max_span / 420.0, 0.010, 0.035))
    intensity = None if intensity is None or len(intensity) != len(xyz) else np.asarray(intensity, dtype=np.float32)

    if len(xyz) > 120000:
        xyz, rgb, intensity = voxel_downsample_numpy(xyz, rgb, intensity=intensity, voxel_size=voxel_size)

    #single lighter outlier pass so noise is reduced without chewing through good geometry
    xyz, rgb, intensity = statistical_outlier_filter_numpy(xyz, rgb, intensity=intensity, k=10, z_thresh=1.8)

    edited_rgb = build_edited_colors(xyz, intensity=intensity, height_from_floor=xyz[:, 2] if len(xyz) else None)

    output_path = build_short_output_path(input_path, ["c"])

    write_ply_xyzrgb_ascii(str(output_path), xyz, edited_rgb)
    return str(output_path)


def remove_ceiling_to_new_file(input_path: str) -> str:
    #save a new scan file with the detected ceiling slab removed
    data = read_ply_data(input_path)
    xyz = data["xyz"]
    intensity = data.get("intensity")

    if xyz.size == 0:
        raise ValueError("scan contains no points")

    out_xyz, _, out_intensity, cut_height = remove_detected_ceiling_numpy(xyz, intensity=intensity)
    if cut_height is None:
        raise ValueError("could not detect a strong ceiling slab in this scan")

    out_rgb = build_edited_colors(out_xyz, intensity=out_intensity, height_from_floor=out_xyz[:, 2] if len(out_xyz) else None)

    output_path = build_short_output_path(input_path, ["r"])

    write_ply_xyzrgb_ascii(str(output_path), out_xyz, out_rgb)
    return str(output_path)


def stabilize_floor_to_new_file(input_path: str) -> str:
    #detect the floor plane on a lightly cleaned copy, then apply that stabilized transform back to the full scan
    data = read_ply_data(input_path)
    xyz = data["xyz"]
    rgb = data.get("rgb")
    intensity = data.get("intensity")

    if xyz.size == 0:
        raise ValueError("scan contains no points")

    mins = xyz.min(axis=0)
    maxs = xyz.max(axis=0)
    span = np.maximum(maxs - mins, 1e-6)
    max_span = float(np.max(span))
    floor_voxel = float(np.clip(max_span / 220.0, 0.030, 0.120))

    work_xyz, work_rgb, work_intensity = voxel_downsample_numpy(xyz, rgb, intensity=intensity, voxel_size=floor_voxel)
    work_xyz, work_rgb, work_intensity = statistical_outlier_filter_numpy(work_xyz, work_rgb, intensity=work_intensity, k=14, z_thresh=1.1)

    stabilized = stabilize_scan_to_floor_frame_numpy(work_xyz, rgb=work_rgb, intensity=work_intensity)
    level_rotation = np.asarray(stabilized["rotation_level"], dtype=np.float64)
    yaw_rotation = np.asarray(stabilized["rotation_yaw"], dtype=np.float64)
    flip_rotation = np.asarray(stabilized["rotation_flip"], dtype=np.float64)
    combined_rotation = flip_rotation @ yaw_rotation @ level_rotation

    out_xyz = _apply_rotation(xyz, combined_rotation)
    if len(out_xyz) > 0:
        floor_ref = float(np.percentile(out_xyz[:, 2], 1))
        out_xyz[:, 2] -= floor_ref
        xy_center = np.median(out_xyz[:, :2], axis=0)
        out_xyz[:, 0] -= float(xy_center[0])
        out_xyz[:, 1] -= float(xy_center[1])

    out_rgb = build_edited_colors(
        out_xyz,
        intensity=intensity,
        height_from_floor=out_xyz[:, 2] if len(out_xyz) else None,
    )

    output_path = build_short_output_path(input_path, ["f"])

    write_ply_xyzrgb_ascii(str(output_path), out_xyz, out_rgb)
    return str(output_path)


def flip_scan_180_to_new_file(input_path: str) -> str:
    #flip a leveled scan upside down by rotating 180 degrees about the x-axis
    data = read_ply_data(input_path)
    xyz = data["xyz"]
    intensity = data.get("intensity")

    if xyz.size == 0:
        raise ValueError("scan contains no points")

    rotation_flip = np.array([
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ], dtype=np.float64)
    out_xyz = _apply_rotation(xyz, rotation_flip)
    if len(out_xyz) > 0:
        floor_ref = float(np.percentile(out_xyz[:, 2], 3))
        out_xyz[:, 2] -= floor_ref
        xy_center = np.median(out_xyz[:, :2], axis=0)
        out_xyz[:, 0] -= float(xy_center[0])
        out_xyz[:, 1] -= float(xy_center[1])

    out_rgb = build_edited_colors(out_xyz, intensity=intensity, height_from_floor=out_xyz[:, 2] if len(out_xyz) else None)

    output_path = build_short_output_path(input_path, ["x"])

    write_ply_xyzrgb_ascii(str(output_path), out_xyz, out_rgb)
    return str(output_path)
#===== SECTION 5: PLY LOADING HELPERS END POINT =====



#===== SECTION 5B: STITCHING HELPERS START POINT =====


def build_stitched_output_path(input_paths: List[str]) -> str:
    #save stitched result with short 3-digit naming
    if not input_paths:
        raise ValueError("no input paths were provided for stitching")

    first_path = Path(input_paths[0])
    parent = first_path.parent if first_path.parent.exists() else INBOX_DIR
    seed_input = str(parent / f"{_next_short_base(parent)}.ply")
    return build_short_output_path(seed_input, ["s"])


def build_height_colors_rgb(xyz: np.ndarray) -> np.ndarray:
    #apply one consistent height-based color scheme for regular, cleaned, and stitched scans
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.size == 0:
        return np.zeros((0, 3), dtype=np.uint8)
    floor_ref = float(np.percentile(xyz[:, 2], 3))
    height_from_floor = xyz[:, 2] - floor_ref
    return _smooth_gradient_colors(_normalize_unit(height_from_floor))


def build_height_colors_rgba(xyz: np.ndarray) -> np.ndarray:
    #same color mapping as the saved edited scans, just converted for the gl viewer
    rgb = build_height_colors_rgb(xyz).astype(np.float32) / 255.0
    if len(rgb) == 0:
        return np.zeros((0, 4), dtype=np.float32)
    return np.hstack((rgb, np.ones((len(rgb), 1), dtype=np.float32)))


def _import_open3d():
    #import open3d only when stitching is actually used so startup does not fail on machines missing the package
    try:
        import open3d as o3d
        return o3d
    except Exception as e:
        raise ImportError(
            "Open3D is required for stitching. Install it with: pip install open3d"
        ) from e


def stitch_two_ply_files(source_path: str, target_path: str, output_path: str, distance_threshold: float = 0.02) -> str:
    #fpfh+ransac global registration followed by two-stage point-to-plane icp
    #all distance parameters auto-scale to the point cloud coordinate units (m, cm, or mm)
    o3d = _import_open3d()

    source_raw = o3d.io.read_point_cloud(source_path)
    target_raw = o3d.io.read_point_cloud(target_path)

    if source_raw.is_empty():
        raise ValueError(f"source cloud is empty: {os.path.basename(source_path)}")
    if target_raw.is_empty():
        raise ValueError(f"target cloud is empty: {os.path.basename(target_path)}")

    nb_neighbors = 30
    std_ratio = 2.0

    def _level_floor(cloud):
        #level the floor to Z=0 and center XY — no yaw normalization so ransac handles in-plane rotation
        #forcing normal[2]>0 before rotating ensures both scans rotate the same direction
        xyz = np.asarray(cloud.points, dtype=np.float32)
        if len(xyz) < 50:
            return cloud
        floor_fit = detect_floor_plane_ransac_numpy(xyz)
        normal = np.asarray(floor_fit["normal"], dtype=np.float64)
        if normal[2] < 0:
            normal = -normal
        rot = _rotation_matrix_from_vectors(normal, np.array([0.0, 0.0, 1.0], dtype=np.float64))
        xyz_lev = _apply_rotation(xyz, rot)
        xyz_lev[:, 2] -= float(np.percentile(xyz_lev[:, 2], 3))
        xy_ctr = np.median(xyz_lev[:, :2], axis=0)
        xyz_lev[:, 0] -= float(xy_ctr[0])
        xyz_lev[:, 1] -= float(xy_ctr[1])
        out = o3d.geometry.PointCloud()
        out.points = o3d.utility.Vector3dVector(xyz_lev)
        if cloud.has_colors():
            out.colors = cloud.colors
        return out

    def _clean(cloud):
        cleaned, _ = cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        return cleaned

    print("[stitch] leveling floors and removing outliers...")
    source = _clean(_level_floor(source_raw))
    target = _clean(_level_floor(target_raw))

    # auto-detect coordinate scale from the actual bounding box — makes every subsequent
    # distance parameter unit-agnostic whether the PLY is in metres, cm, or mm
    all_pts = np.vstack([np.asarray(source.points), np.asarray(target.points)])
    extents = np.max(all_pts, axis=0) - np.min(all_pts, axis=0)
    max_extent = float(np.max(extents))
    fpfh_voxel  = max_extent * 0.05   # 5% of scene — room-scale feature context
    merge_voxel = max_extent * 0.008  # 0.8% of scene — fine output density
    norm_radius = fpfh_voxel / 3.0
    corr_dist   = fpfh_voxel * 1.5
    icp_coarse_dist = fpfh_voxel * 0.5
    icp_fine_dist   = max_extent * 0.004  # ~2 cm equivalent for a 5 m room
    print(f"[stitch] scene extent={max_extent:.1f} units  fpfh_voxel={fpfh_voxel:.2f}  merge_voxel={merge_voxel:.3f}")

    def _fpfh(cloud):
        down = cloud.voxel_down_sample(fpfh_voxel)
        down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=fpfh_voxel * 2, max_nn=30))
        features = o3d.pipelines.registration.compute_fpfh_feature(
            down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=fpfh_voxel * 5, max_nn=100),
        )
        return down, features

    print("[stitch] extracting fpfh features...")
    source_down, source_fpfh = _fpfh(source)
    target_down, target_fpfh = _fpfh(target)
    print(f"[stitch] {len(source_down.points)} source keypoints, {len(target_down.points)} target keypoints")

    print("[stitch] running ransac global registration...")
    ransac_result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        mutual_filter=False,
        max_correspondence_distance=corr_dist,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(corr_dist),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 0.999),
    )
    print(f"[stitch] ransac fitness={ransac_result.fitness:.3f}  inlier_rmse={ransac_result.inlier_rmse:.3f}")

    if ransac_result.fitness < 0.05:
        print(
            "[stitch] warning: ransac fitness is very low — "
            "scans may not have enough overlapping geometry; result may be incorrect"
        )

    print("[stitch] refining alignment with two-stage icp...")
    source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=norm_radius, max_nn=30))
    target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=norm_radius, max_nn=30))

    icp_c = o3d.pipelines.registration.registration_icp(
        source, target,
        icp_coarse_dist,
        ransac_result.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )
    icp_f = o3d.pipelines.registration.registration_icp(
        source, target,
        icp_fine_dist,
        icp_c.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )
    print(f"[stitch] icp fitness={icp_f.fitness:.3f}  inlier_rmse={icp_f.inlier_rmse:.4f}")

    source_aligned = copy.deepcopy(source)
    source_aligned.transform(icp_f.transformation)

    merged = source_aligned + target
    merged = merged.voxel_down_sample(merge_voxel)

    if not o3d.io.write_point_cloud(output_path, merged):
        raise IOError(f"failed to save stitched model to {output_path}")
    return output_path


def stitch_multiple_ply_files(input_paths: List[str], output_path: str, distance_threshold: float = 0.02) -> str:
    #run the two-file icp stitch repeatedly so any number of selected scans can be merged
    if len(input_paths) < 2:
        raise ValueError("select at least 2 .ply scans to stitch")

    ordered_paths = [str(Path(p)) for p in input_paths]
    with tempfile.TemporaryDirectory(prefix="lidar_stitch_") as temp_dir:
        running_path = ordered_paths[0]

        for index, next_path in enumerate(ordered_paths[1:], start=1):
            is_last_merge = index == (len(ordered_paths) - 1)
            current_output = output_path if is_last_merge else str(Path(temp_dir) / f"partial_merge_{index}.ply")
            stitch_two_ply_files(
                source_path=next_path,
                target_path=running_path,
                output_path=current_output,
                distance_threshold=distance_threshold,
            )
            running_path = current_output

    return output_path


class StitchWorker(QThread):
    #emits the final stitched file path
    finished_success = pyqtSignal(str)
    #thread-safe logs to the ui
    log = pyqtSignal(str)
    #thread-safe stitch failure reporting
    error = pyqtSignal(str)

    def __init__(self, input_paths: List[str], output_path: str, parent=None):
        super().__init__(parent)
        self.input_paths = input_paths
        self.output_path = output_path

    def run(self):
        #run the built-in multi-file stitcher using the selected queue items
        try:
            self.log.emit(f"[stitch] stitching {len(self.input_paths)} selected file(s)...")
            for idx, path in enumerate(self.input_paths, start=1):
                self.log.emit(f"[stitch] {idx}. {os.path.basename(path)}")

            final_output = stitch_multiple_ply_files(self.input_paths, self.output_path)

            if not os.path.exists(final_output):
                raise FileNotFoundError(
                    "stitching finished but no output .ply file was found on disk."
                )

            self.log.emit(f"[stitch] stitched file created: {os.path.basename(final_output)}")
            self.finished_success.emit(final_output)

        except Exception as e:
            self.error.emit(f"[stitch] failed: {e}")
#===== SECTION 5B: STITCHING HELPERS END POINT =====

def build_dark_app_stylesheet() -> str:
    #dark ui theme so controls match the black viewer background
    return """
    QWidget {
        background-color: #121212;
        color: #e8e8e8;
        font-size: 10pt;
    }
    QMainWindow, QWidget#centralWidget {
        background-color: #121212;
    }
    QGroupBox {
        border: 1px solid #3a3a3a;
        border-radius: 6px;
        margin-top: 10px;
        padding-top: 10px;
        font-weight: bold;
        background-color: #171717;
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        left: 10px;
        padding: 0 4px;
        color: #f0f0f0;
    }
    QPushButton {
        background-color: #232323;
        border: 1px solid #444444;
        border-radius: 5px;
        padding: 6px 10px;
        color: #f0f0f0;
    }
    QPushButton:hover {
        background-color: #2d2d2d;
        border: 1px solid #5a5a5a;
    }
    QPushButton:pressed {
        background-color: #1b1b1b;
    }
    QPushButton:disabled {
        color: #8c8c8c;
        background-color: #1a1a1a;
        border: 1px solid #303030;
    }
    QListWidget, QLabel {
        background-color: transparent;
    }
    QListWidget {
        background-color: #171717;
        border: 1px solid #383838;
        border-radius: 6px;
    }
    QListWidget::item {
        padding: 4px;
    }
    QListWidget::item:selected {
        background-color: #21436b;
        color: #ffffff;
        border-radius: 4px;
    }
    QProgressBar {
        background-color: #171717;
        border: 1px solid #383838;
        border-radius: 5px;
        text-align: center;
        color: #f0f0f0;
    }
    QProgressBar::chunk {
        background-color: #3d8bfd;
        border-radius: 4px;
    }
    QCheckBox {
        spacing: 8px;
    }
    QCheckBox::indicator {
        width: 14px;
        height: 14px;
    }
    QCheckBox::indicator:unchecked {
        border: 1px solid #5a5a5a;
        background: #171717;
    }
    QCheckBox::indicator:checked {
        border: 1px solid #3d8bfd;
        background: #3d8bfd;
    }
    QMessageBox {
        background-color: #121212;
    }
    """


#===== SECTION 6: 3D VIEWPORT (SINGLE SCAN) START POINT =====
class SinglePLYViewport(QWidget):
    point_count_changed = pyqtSignal(int)

    def __init__(self, parent=None):
        super().__init__(parent)

        #ui layout: viewport fills this widget
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        #single 3d viewer for one point cloud at a time
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=85, elevation=18, azimuth=45)

        #keep a dark viewport background for stronger point-cloud contrast
        try:
            self.view.setBackgroundColor("k")
        except Exception:
            self.view.opts["bgcolor"] = (0, 0, 0, 255)

        layout.addWidget(self.view)

        #one scatter item only
        self.point_cloud_item = gl.GLScatterPlotItem(size=3.5, pxMode=True)
        self.point_cloud_item.setGLOptions("opaque")
        self.view.addItem(self.point_cloud_item)

        #floor plane that sits under the visible cloud
        self.floor_plane_item = gl.GLGridItem()
        self.floor_plane_item.setSpacing(x=1.0, y=1.0)
        self.view.addItem(self.floor_plane_item)

        #track loaded point-cloud state so viewer-only filters can be toggled on and off
        self.file_loaded = False
        self._raw_pos = np.zeros((0, 3), dtype=np.float32)
        self._raw_color = np.zeros((0, 4), dtype=np.float32)
        self._fit_scale = 1.0
        self._display_distance = 25.0
        self.hide_ceiling_enabled = False
        self._ceiling_cut_height = None

    def _update_floor_plane(self, pos_local: np.ndarray):
        #resize and place the floor plane so the cloud appears to sit on top of it
        if len(pos_local) == 0:
            self.floor_plane_item.resetTransform()
            self.floor_plane_item.setSize(x=1.0, y=1.0)
            return

        mins = pos_local.min(axis=0)
        maxs = pos_local.max(axis=0)

        span_x = max(float(maxs[0] - mins[0]), 4.0)
        span_y = max(float(maxs[1] - mins[1]), 4.0)
        floor_z = float(mins[2]) - 0.02

        center_x = float((mins[0] + maxs[0]) / 2.0)
        center_y = float((mins[1] + maxs[1]) / 2.0)

        self.floor_plane_item.resetTransform()
        self.floor_plane_item.setSize(x=span_x * 1.15, y=span_y * 1.15)

        spacing = max(min(span_x, span_y) / 20.0, 0.5)
        self.floor_plane_item.setSpacing(x=spacing, y=spacing)
        self.floor_plane_item.translate(center_x, center_y, floor_z)

    def _refresh_display(self):
        #redraw the viewer using the current hide-ceiling toggle without touching the underlying file
        if len(self._raw_pos) == 0:
            self.point_cloud_item.setData(pos=np.zeros((0, 3), dtype=np.float32))
            self.floor_plane_item.resetTransform()
            self.floor_plane_item.setSize(x=1.0, y=1.0)
            self.file_loaded = False
            self.point_count_changed.emit(0)
            return

        pos = self._raw_pos
        color = self._raw_color
        if self.hide_ceiling_enabled and self._ceiling_cut_height is not None:
            keep = pos[:, 2] < float(self._ceiling_cut_height)
            if int(np.count_nonzero(keep)) >= max(100, int(len(pos) * 0.55)):
                pos = pos[keep]
                color = color[keep]

        mins = pos.min(axis=0)
        maxs = pos.max(axis=0)
        center = (mins + maxs) / 2.0
        pos_local = (pos - center) * self._fit_scale

        self.point_cloud_item.setData(
            pos=pos_local.astype(np.float32),
            color=color.astype(np.float32),
            size=3.5,
            pxMode=True
        )
        self.point_cloud_item.setGLOptions("opaque")
        self._update_floor_plane(pos_local)

        self.file_loaded = True
        self.view.setCameraPosition(distance=self._display_distance, elevation=18, azimuth=45)
        self.point_count_changed.emit(int(len(pos)))

    def set_hide_ceiling(self, enabled: bool):
        #toggle viewer-only ceiling hiding for the currently loaded scan
        self.hide_ceiling_enabled = bool(enabled)
        self._refresh_display()

    def set_top_view(self):
        #snap camera to a straight top-down view
        self.view.setCameraPosition(distance=self._display_distance, elevation=90, azimuth=0)

    def set_left_view(self):
        #snap camera to the left side of the scan
        self.view.setCameraPosition(distance=self._display_distance, elevation=0, azimuth=180)

    def set_right_view(self):
        #snap camera to the right side of the scan
        self.view.setCameraPosition(distance=self._display_distance, elevation=0, azimuth=0)

    def set_bottom_view(self):
        #snap camera to a straight bottom-up view
        self.view.setCameraPosition(distance=self._display_distance, elevation=-90, azimuth=0)

    def clear_view(self):
        #wipe the current point cloud from the viewer
        self._raw_pos = np.zeros((0, 3), dtype=np.float32)
        self._raw_color = np.zeros((0, 4), dtype=np.float32)
        self._ceiling_cut_height = None
        self.hide_ceiling_enabled = False
        self.point_cloud_item.setData(pos=np.zeros((0, 3), dtype=np.float32))
        self.floor_plane_item.resetTransform()
        self.floor_plane_item.setSize(x=1.0, y=1.0)
        self.file_loaded = False
        self.point_count_changed.emit(0)

    def load_ply(self, path: str):
        #load one ply, fit it into view, and replace the current scan
        data = read_ply_data(path)
        pos = data["xyz"]
        if pos.size == 0:
            raise ValueError("ply contains no vertices")

        span = pos.max(axis=0) - pos.min(axis=0)
        max_span = float(np.max(span)) if float(np.max(span)) > 0 else 1.0
        self._fit_scale = 25.0 / max_span
        self._display_distance = max(25.0, max_span * self._fit_scale * 2.5)
        self._raw_pos = pos.astype(np.float32)
        self._raw_color = build_height_colors_rgba(pos).astype(np.float32)
        self._ceiling_cut_height = estimate_ceiling_cut_height(pos[:, 2])
        self._refresh_display()

#===== SECTION 6: 3D VIEWPORT (SINGLE SCAN) END POINT =====

#===== SECTION 7: MAIN WINDOW (UI + APP LOGIC) START POINT =====
class LidarWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("LIDAR Image Generation & Stitching")
        self.setGeometry(250, 250, 1100, 650)

        self._closing = False
        self.current_loaded_path: Optional[str] = None
        self.stitch_worker: Optional[StitchWorker] = None
        self.hide_ceiling_enabled = False

        #===== SECTION 8: UI CREATION START POINT =====
        self._build_ui()
        self.setStyleSheet(build_dark_app_stylesheet())
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

    def _update_point_count_label(self, count: int):
        #show the current displayed point count in the lower-right corner
        self.point_count_lbl.setText(f"Point Counter: {int(count):,}")

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
        try:
            self.viewer3d.load_ply(path)
            self.current_loaded_path = path
            self._log(f"loaded scan: {os.path.basename(path)}")
            self.info_lbl.setText(f"Loaded:\n{os.path.basename(path)}")
        except Exception as e:
            self._log(f"load failed: {e}")

    def _on_file_received(self, saved_path: str):
        #add the received file to the queue list and optionally auto-load it
        filename = os.path.basename(saved_path)
        self._add_queue_item(filename, saved_path)
        self._log(f"received -> Desktop/LiDAR_Inbox: {filename}")



    def _add_queue_item(self, display_name: str, actual_path: str):
        #add a file to the queue with a display label that can differ from the disk filename
        item = QListWidgetItem(display_name)
        item.setData(Qt.ItemDataRole.UserRole, actual_path)
        self.scan_list.addItem(item)

    def _cleanup_scan_clicked(self):
        #create an edited version of the currently loaded scan and add it back to the queue
        if not self.current_loaded_path or not os.path.exists(self.current_loaded_path):
            self._log("cleanup failed: load a scan first.")
            return

        try:
            self.progress.setRange(0, 0)
            self._log(f"cleaning scan: {os.path.basename(self.current_loaded_path)}")
            edited_path = cleanup_scan_to_new_file(self.current_loaded_path)
            display_name = os.path.basename(edited_path)
            self._add_queue_item(display_name, edited_path)
            self._log(f"edited scan saved: {os.path.basename(edited_path)}")
            self._load_path_into_viewer(edited_path)
        except Exception as e:
            self._log(f"cleanup failed: {e}")
        finally:
            self.progress.setRange(0, 100)
            self.progress.setValue(0)

    def _stabilize_floor_clicked(self):
        #create a new leveled file with the detected floor placed at the bottom
        if not self.current_loaded_path or not os.path.exists(self.current_loaded_path):
            self._log("flooring failed: load a scan first.")
            return

        try:
            self.progress.setRange(0, 0)
            self._log(f"stabilizing floor: {os.path.basename(self.current_loaded_path)}")
            floored_path = stabilize_floor_to_new_file(self.current_loaded_path)
            display_name = os.path.basename(floored_path)
            self._add_queue_item(display_name, floored_path)
            self._log(f"floored scan saved: {os.path.basename(floored_path)}")
            self._load_path_into_viewer(floored_path)
        except Exception as e:
            self._log(f"flooring failed: {e}")
        finally:
            self.progress.setRange(0, 100)
            self.progress.setValue(0)

    def _flip_180_clicked(self):
        #flip the currently loaded scan so a ceiling-picked floor result can be inverted quickly
        if not self.current_loaded_path or not os.path.exists(self.current_loaded_path):
            self._log("flip failed: load a scan first.")
            return

        try:
            self.progress.setRange(0, 0)
            self._log(f"flipping scan 180°: {os.path.basename(self.current_loaded_path)}")
            flipped_path = flip_scan_180_to_new_file(self.current_loaded_path)
            display_name = os.path.basename(flipped_path)
            self._add_queue_item(display_name, flipped_path)
            self._log(f"flipped scan saved: {os.path.basename(flipped_path)}")
            self._load_path_into_viewer(flipped_path)
        except Exception as e:
            self._log(f"flip failed: {e}")
        finally:
            self.progress.setRange(0, 100)
            self.progress.setValue(0)

    def _hide_ceiling_toggled(self, checked: bool):
        #toggle viewer-only hiding of the detected ceiling slab
        self.viewer3d.set_hide_ceiling(bool(checked))
        self._log("viewer ceiling hidden." if checked else "viewer ceiling shown.")

    def _top_view_clicked(self):
        #switch the viewer camera to the top view
        self.viewer3d.set_top_view()
        self._log("camera set to top view.")

    def _left_view_clicked(self):
        #switch the viewer camera to the left side view
        self.viewer3d.set_left_view()
        self._log("camera set to left side view.")

    def _right_view_clicked(self):
        #switch the viewer camera to the right side view
        self.viewer3d.set_right_view()
        self._log("camera set to right side view.")

    def _bottom_view_clicked(self):
        #switch the viewer camera to the bottom view
        self.viewer3d.set_bottom_view()
        self._log("camera set to bottom view.")

    def _build_ui(self):
        #===== SECTION 12: UI LAYOUT START POINT =====
        root = QWidget()
        self.setCentralWidget(root)
        main_layout = QHBoxLayout(root)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        #===== SECTION 13: LEFT COLUMN (QUEUE + STATUS) START POINT =====
        left = QVBoxLayout()
        left.setSpacing(8)
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
        self.scan_list.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.scan_list.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        left.addWidget(self.scan_list, 1)

        file_btn_row = QHBoxLayout()
        self.btn_import_local = QPushButton("Import Local")
        self.btn_delete = QPushButton("Delete Selected")
        file_btn_row.addWidget(self.btn_import_local)
        file_btn_row.addWidget(self.btn_delete)
        left.addLayout(file_btn_row)

        self.btn_stitch = QPushButton("Stitch")
        left.addWidget(self.btn_stitch)

        self.btn_toggle_right = QPushButton("Hide Side Panel")
        left.addWidget(self.btn_toggle_right)
        #===== SECTION 13: LEFT COLUMN (QUEUE + STATUS) END POINT =====

        #===== SECTION 14: CENTER COLUMN (3D VIEW) START POINT =====
        center = QVBoxLayout()
        center.setSpacing(8)
        main_layout.addLayout(center, 4)

        self.viewer3d = SinglePLYViewport()
        center.addWidget(self.viewer3d, 1)

        #===== SECTION 14: CENTER COLUMN (3D VIEW) END POINT =====

        #===== SECTION 15: RIGHT COLUMN (CONTROLS + LOG) START POINT =====
        self.right_panel = QWidget()
        self.right_layout = QVBoxLayout(self.right_panel)
        self.right_layout.setContentsMargins(0, 0, 0, 0)
        self.right_layout.setSpacing(10)
        main_layout.addWidget(self.right_panel, 2)

        render_group = QGroupBox("Render Controls")
        render_group.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Maximum)
        render_layout = QGridLayout(render_group)
        render_layout.setHorizontalSpacing(8)
        render_layout.setVerticalSpacing(8)

        self.btn_clear_view = QPushButton("Clear Viewer")
        self.btn_floor_scan = QPushButton("Stabilize Floor")
        self.btn_flip_180 = QPushButton("Flip 180°")
        self.btn_cleanup_scan = QPushButton("Clean Up Scan")
        self.btn_top_view = QPushButton("Top View")
        self.btn_left_view = QPushButton("Left Side View")
        self.btn_right_view = QPushButton("Right Side View")
        self.btn_bottom_view = QPushButton("Bottom View")
        self.hide_ceiling_chk = QCheckBox("Hide ceiling in viewer")

        right_buttons = [
            self.btn_clear_view,
            self.btn_floor_scan,
            self.btn_flip_180,
            self.btn_cleanup_scan,
            self.btn_top_view,
            self.btn_left_view,
            self.btn_right_view,
            self.btn_bottom_view,
        ]
        for btn in right_buttons:
            btn.setMinimumHeight(36)
            btn.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        render_layout.addWidget(self.btn_clear_view, 0, 0)
        render_layout.addWidget(self.btn_floor_scan, 0, 1)
        render_layout.addWidget(self.btn_flip_180, 1, 0)
        render_layout.addWidget(self.btn_cleanup_scan, 1, 1)
        render_layout.addWidget(self.hide_ceiling_chk, 2, 0, 1, 2)
        render_layout.setColumnStretch(0, 1)
        render_layout.setColumnStretch(1, 1)
        self.right_layout.addWidget(render_group, 0)

        info_group = QGroupBox("File Info")
        info_group.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Maximum)
        info_layout = QVBoxLayout(info_group)
        self.info_lbl = QLabel("No file loaded.")
        self.info_lbl.setMinimumHeight(42)
        self.info_lbl.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.info_lbl.setWordWrap(True)
        info_layout.addWidget(self.info_lbl)
        self.right_layout.addWidget(info_group, 0)

        self.progress = NullProgress()

        log_title = QLabel("Log")
        log_title.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Maximum)
        self.right_layout.addWidget(log_title, 0)

        self.log_lbl = QLabel("")
        self.log_lbl.setWordWrap(True)
        self.log_lbl.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.log_lbl.setMinimumHeight(180)
        self.log_lbl.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.log_lbl.setStyleSheet("border: 1px solid #3a3a3a; background: #171717; padding: 8px; border-radius: 6px;")
        self.right_layout.addWidget(self.log_lbl, 1)

        viewer_group = QGroupBox("Viewer Buttons")
        viewer_group.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Maximum)
        viewer_layout = QGridLayout(viewer_group)
        viewer_layout.setHorizontalSpacing(8)
        viewer_layout.setVerticalSpacing(8)
        viewer_layout.addWidget(self.btn_top_view, 0, 0)
        viewer_layout.addWidget(self.btn_left_view, 0, 1)
        viewer_layout.addWidget(self.btn_right_view, 1, 0)
        viewer_layout.addWidget(self.btn_bottom_view, 1, 1)
        viewer_layout.setColumnStretch(0, 1)
        viewer_layout.setColumnStretch(1, 1)
        self.right_layout.addWidget(viewer_group, 0)

        bottom_row = QHBoxLayout()
        bottom_row.setContentsMargins(0, 0, 0, 0)
        bottom_row.addStretch(1)
        self.point_count_lbl = QLabel("Point Counter: 0")
        self.point_count_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.point_count_lbl.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Maximum)
        self.point_count_lbl.setStyleSheet("font-weight: bold; padding: 6px 2px;")
        bottom_row.addWidget(self.point_count_lbl)
        bottom_row.addStretch(1)
        self.right_layout.addLayout(bottom_row)

        main_layout.setStretch(0, 1)
        main_layout.setStretch(1, 4)
        main_layout.setStretch(2, 2)
        #===== SECTION 15: RIGHT COLUMN (CONTROLS + LOG) END POINT =====
        #===== SECTION 12: UI LAYOUT END POINT =====

    def _wire_min_signals(self):
        #===== SECTION 16: UI EVENTS START POINT =====
        self.btn_import_local.clicked.connect(self._import_local_clicked)
        self.btn_delete.clicked.connect(self._delete_selected_clicked)
        self.scan_list.itemDoubleClicked.connect(self._load_item_into_viewer)
        self.btn_clear_view.clicked.connect(self._clear_view_clicked)
        self.btn_floor_scan.clicked.connect(self._stabilize_floor_clicked)
        self.btn_flip_180.clicked.connect(self._flip_180_clicked)
        self.btn_cleanup_scan.clicked.connect(self._cleanup_scan_clicked)
        self.btn_top_view.clicked.connect(self._top_view_clicked)
        self.btn_left_view.clicked.connect(self._left_view_clicked)
        self.btn_right_view.clicked.connect(self._right_view_clicked)
        self.btn_bottom_view.clicked.connect(self._bottom_view_clicked)
        self.hide_ceiling_chk.toggled.connect(self._hide_ceiling_toggled)
        self.btn_stitch.clicked.connect(self._stitch_clicked)
        self.btn_toggle_right.clicked.connect(self._toggle_right_panel)
        self.viewer3d.point_count_changed.connect(self._update_point_count_label)
        #===== SECTION 16: UI EVENTS END POINT =====

    def _get_selected_queue_paths(self) -> List[str]:
        #return valid file paths for all currently selected queue items
        selected_paths: List[str] = []
        for item in self.scan_list.selectedItems():
            path = item.data(Qt.ItemDataRole.UserRole)
            if path and os.path.exists(path):
                selected_paths.append(path)
        return selected_paths

    def _stitch_clicked(self):
        #run the built-in stitching algorithm on multiple selected scans
        selected_paths = self._get_selected_queue_paths()
        if len(selected_paths) < 2:
            self._log("[stitch] select at least 2 scans in the queue first.")
            QMessageBox.warning(self, "Stitch", "Select at least 2 scans in the queue before stitching.")
            return

        try:
            output_path = build_stitched_output_path(selected_paths)
            self._log(f"[stitch] output will be saved as: {os.path.basename(output_path)}")
            self.progress.setRange(0, 0)
            self.btn_stitch.setEnabled(False)

            self.stitch_worker = StitchWorker(selected_paths, output_path, parent=self)
            self.stitch_worker.log.connect(self._log)
            self.stitch_worker.error.connect(self._on_stitch_error)
            self.stitch_worker.finished_success.connect(self._on_stitch_success)
            self.stitch_worker.finished.connect(self._on_stitch_finished)
            self.stitch_worker.start()
        except Exception as e:
            self._log(f"[stitch] failed before worker start: {e}")
            self.progress.setRange(0, 100)
            self.progress.setValue(0)
            self.btn_stitch.setEnabled(True)
            QMessageBox.critical(self, "Stitch Failed", str(e))

    def _on_stitch_success(self, stitched_path: str):
        #add the stitched scan to the queue and load it into the viewer
        display_name = os.path.basename(stitched_path)
        self._add_queue_item(display_name, stitched_path)
        self._load_path_into_viewer(stitched_path)
        self._log(f"[stitch] success. added stitched scan: {display_name}")

    def _on_stitch_error(self, msg: str):
        #surface stitch errors to the user and the log
        self._log(msg)
        QMessageBox.critical(self, "Stitch Failed", msg.splitlines()[0])

    def _on_stitch_finished(self):
        #restore ui state after stitch worker exits
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.btn_stitch.setEnabled(True)

    def _toggle_right_panel(self):
        if self.right_panel.isVisible():
            self.right_panel.hide()
            self.btn_toggle_right.setText("Show Side Panel")
        else:
            self.right_panel.show()
            self.btn_toggle_right.setText("Hide Side Panel")

    def _clear_view_clicked(self):
        self.viewer3d.clear_view()
        self.hide_ceiling_chk.blockSignals(True)
        self.hide_ceiling_chk.setChecked(False)
        self.hide_ceiling_chk.blockSignals(False)
        self.current_loaded_path = None
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
            self._add_queue_item(filename, path)
            self._log(f"imported local file: {filename}")


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

        try:
            if hasattr(self, "stitch_worker") and self.stitch_worker and self.stitch_worker.isRunning():
                self._log("waiting for stitch worker to finish...")
                self.stitch_worker.wait(1500)
        except Exception:
            pass
        super().closeEvent(event)
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
