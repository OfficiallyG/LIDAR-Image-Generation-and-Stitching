import sys
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QApplication,        # Required for PyQt application
    QMainWindow,         # Main window container
    QWidget,             # Base widget used as central container
    QHBoxLayout,         # Horizontal layout (left | center | right)
    QVBoxLayout,         # Vertical layout (stacked widgets)
    QGroupBox,           # Titled UI sections
    QLabel,              # Text / display widget
    QPushButton,         # Clickable button
    QListWidget,         # List widget for scan queue
    QProgressBar,        # Progress indicator (future use for file upload etc)
    QCheckBox,           # Toggle options
    QFileDialog          # Native OS file picker dialog
)

# Dark background checkbox in the Render Controls group.
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

# NEXT STEPS!
#
# Task 1 (UI Designer)
#   - Works on layout consistency, labels, grouping, UX polish
#
# Task 2 (3D viewport)
#   - Replace the CENTER "viewport" QLabel with pyqtgraph.opengl
#   - Hook render + view controls to the viewport
#
# Task 3 (File Picker & Queue Data):
#   - Expand import logic (metadata, duplicate handling, previews)
#   - Store full file paths using QListWidgetItem.setData(Qt.UserRole, path)
#
# Task 4 (Raspberry Pi / Networking):
#   - Add "Receive from Pi" button + background TCP thread
#
# Task 5 (Stitching):
#   - Consume file paths from scan queue and output stitched result


class LidarWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # Window title shown in OS title bar
        self.setWindowTitle("LIDAR Image Generation & Stitching")

        # Initial window position (x, y) and size (width, height)
        self.setGeometry(250, 250, 1100, 650)

        # Build all UI widgets and layouts
        self._build_ui()

        # Wire only essential signals to avoid breaking anything
        self._wire_min_signals()

        # Initial log message
        self._log("UI initialized. Ready.")

    # UI Construction
    def _build_ui(self):
        # Root widget is required to apply layouts inside QMainWindow
        root = QWidget()
        self.setCentralWidget(root)

        # Main layout splits window into 3 vertical sections
        main_layout = QHBoxLayout(root)

        # LEFT PANEL — Scan Queue + Import Controls
        left = QVBoxLayout()
        main_layout.addLayout(left, 1)  # smaller width than center

        left.addWidget(QLabel("Scans Queue"))

        # List widget showing imported scan filenames
        self.scan_list = QListWidget()
        left.addWidget(self.scan_list, 1)

        # Row for import/delete buttons
        file_btn_row = QHBoxLayout()
        self.btn_import_local = QPushButton("Import Local")
        self.btn_delete = QPushButton("Delete Selected")
        file_btn_row.addWidget(self.btn_import_local)
        file_btn_row.addWidget(self.btn_delete)
        left.addLayout(file_btn_row)

        # Auto-load toggle (used later to auto-preview files)
        self.auto_load_chk = QCheckBox("Auto-load newest file after import")
        self.auto_load_chk.setChecked(True)
        left.addWidget(self.auto_load_chk)

        # CENTER PANEL — Viewport Placeholder
        center = QVBoxLayout()
        main_layout.addLayout(center, 3)  # Center will be the largest section

        # 3D Viewport Placeholder
        self.viewport = QLabel(
            "3D Viewport Placeholder\n\n"
            "• Rotate / Pan / Zoom\n"
            "• Display .PLY point clouds"
        )
        self.viewport.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.viewport.setStyleSheet("border: 2px dashed #666; padding: 18px;")
        center.addWidget(self.viewport, 1)

        # RIGHT PANEL — Controls, Info, Status
        right = QVBoxLayout()
        main_layout.addLayout(right, 1)

        # Render Controls
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

        # View Controls
        view_group = QGroupBox("View Controls")
        view_layout = QVBoxLayout(view_group)

        self.btn_fit = QPushButton("Center & Fit")
        self.btn_reset = QPushButton("Reset View")

        view_layout.addWidget(self.btn_fit)
        view_layout.addWidget(self.btn_reset)
        right.addWidget(view_group)

        # File Info
        info_group = QGroupBox("File Info")
        info_layout = QVBoxLayout(info_group)

        self.info_lbl = QLabel("No file loaded.")
        self.info_lbl.setWordWrap(True)
        info_layout.addWidget(self.info_lbl)

        right.addWidget(info_group)

        # Progress + Log
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

    # Minimal Signal Wiring
    def _wire_min_signals(self):
        # Only two connected actions for now:
        self.btn_import_local.clicked.connect(self._import_local_clicked)
        self.btn_delete.clicked.connect(self._delete_selected_clicked)
        # Wire dark theme toggle (checkbox emits bool)
        try:
            self.chk_dark_bg.toggled.connect(self._apply_dark_theme)
        except Exception:
            # In case UI construction hasn't created the checkbox yet
            pass

        # Apply initial theme based on the checkbox state
        if hasattr(self, 'chk_dark_bg'):
            self._apply_dark_theme(self.chk_dark_bg.isChecked())

    # Import Local — REAL File Picker
    def _import_local_clicked(self):
        # Opens a native OS file picker and allows the user to select
        # one or more .PLY files.
        #
        # Responsibilities of this function:
        # - Open dialog
        # - Validate extension
        # - Add filenames to scan queue
        # - Optionally auto-select newest file

        # Open native file picker dialog
        file_paths, _ = QFileDialog.getOpenFileNames(
            self,
            "Select LiDAR Scan(s)",
            "",
            "PLY Files (*.ply);;All Files (*)"
        )

        # If user cancels dialog, file_paths will be empty
        if not file_paths:
            self._log("Import canceled.")
            return

        # Loop through all selected files
        for path in file_paths:
            # Extract just the filename (UI-friendly)
            filename = path.split("/")[-1]

            # Add filename to scan queue list
            self.scan_list.addItem(filename)

            # Log import event
            self._log(f"Imported local file: {filename}")

            # LEON! NEXT STEP:
            # Store the *full path* instead of just filename.

        # Auto-load behavior: select last imported item
        if self.auto_load_chk.isChecked():
            last_index = self.scan_list.count() - 1
            self.scan_list.setCurrentRow(last_index)

            # Update File Info panel (placeholder)
            self.info_lbl.setText(
                f"File: {self.scan_list.item(last_index).text()}\n"
                f"Points: (TODO)\n"
                f"Has color: (TODO)"
            )

            self._log("Auto-load enabled: selected newest imported file.")

    # Delete Selected Files
    def _delete_selected_clicked(self):
        # Remove each selected item from the scan queue
        for item in self.scan_list.selectedItems():
            row = self.scan_list.row(item)
            self.scan_list.takeItem(row)

        self._log("Deleted selected scan(s) from queue.")

        # Next steps if we think necessary:
        # If full paths are stored in item data,
        # deletion will automatically stay in sync.

    # Simple Logger Utility
    def _log(self, msg: str):
        from time import strftime
        ts = strftime("%H:%M:%S")

        current = self.log_lbl.text()
        self.log_lbl.setText((f"[{ts}] {msg}\n" + current)[:4000])

    # Theme application handler
    def _apply_dark_theme(self, enabled: bool):
        app = QApplication.instance()
        if not app:
            return
        if enabled:
            app.setStyleSheet(DARK_STYLESHEET)
        else:
            app.setStyleSheet("")


def main():
    # Create Qt application
    lidar_app = QApplication(sys.argv)
    # Note: Theme is applied by the window's `Dark background` checkbox.

    # Create and show main window
    lidar_main = LidarWindow()
    lidar_main.show()

    # Start event loop
    sys.exit(lidar_app.exec())


if __name__ == "__main__":
    main()
