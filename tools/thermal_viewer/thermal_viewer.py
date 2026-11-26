#!/usr/bin/env python3
"""Desktop viewer for the thermal cameras this repository supports.

A camera plugged into this computer (Thermal Master P1, Seek Compact Pro,
HTI-301 over USB; MLX90640 on a Linux I2C bus) is read with the same core
code its ESP32-P4 component uses, and its frames go through the repository's
own processing: radiometric (repair, outlier, temporal, smoothing, median,
statistics), ISP (tone mapping, palettes, equalization, detail, mirroring,
scaling, spot meters) and overlay. Nothing is processed in Python: this file
is the window and the controls, native/ is the glue, components/ is the code
under test.

Usage: tools/thermal_viewer/thermal_viewer.py [--camera KEY] [--arg ARG] [--connect]
Needs: PyQt6 (pip install PyQt6), a C++17 compiler, and libusb 1.0 for the
USB cameras. The native library is built on the first start and again
whenever a source changes. No camera is opened until Connect is pressed
(or --connect is given).
"""

import argparse
import ctypes
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
COMP = os.path.join(ROOT, "components")
NATIVE = os.path.join(HERE, "native")
BUILD = os.path.join(ROOT, "build", "thermal_viewer")

CORE_SOURCES = [
    "video_core/frame.cpp",
    "radiometric/radiometric_core.cpp",
    "isp/isp_core.cpp",
    "overlay/overlay_core.cpp",
    "thermal_master_p1/tm_core.cpp",
    "seek_compact_pro/seek_core.cpp",
    "hti301/hti301_core.cpp",
    "mlx90640/subpage_assembler.cpp",
]
LINUX_SOURCES = ["mlx90640/MLX90640_API.cpp"]


# ---------------------------------------------------------------- native library

def _libusb_flags():
    """Compiler and linker flags for libusb 1.0, or None when it is not installed."""
    if shutil.which("pkg-config"):
        r = subprocess.run(["pkg-config", "--cflags", "--libs", "libusb-1.0"], capture_output=True, text=True)
        if r.returncode == 0:
            return r.stdout.split()
    if shutil.which("brew"):
        r = subprocess.run(["brew", "--prefix", "libusb"], capture_output=True, text=True)
        prefix = r.stdout.strip()
        if r.returncode == 0 and os.path.exists(os.path.join(prefix, "include", "libusb-1.0", "libusb.h")):
            return [f"-I{prefix}/include/libusb-1.0", f"-L{prefix}/lib", "-lusb-1.0"]
    for inc in ("/usr/include/libusb-1.0", "/usr/local/include/libusb-1.0"):
        if os.path.exists(os.path.join(inc, "libusb.h")):
            return [f"-I{inc}", "-lusb-1.0"]
    return None


def build_library(force=False):
    """Builds the native library if a source is newer than it; returns its path."""
    lib = os.path.join(BUILD, "libthermalview" + (".dylib" if sys.platform == "darwin" else ".so"))
    sources = [os.path.join(COMP, s) for s in CORE_SOURCES]
    if sys.platform.startswith("linux"):
        sources += [os.path.join(COMP, s) for s in LINUX_SOURCES]
    sources += sorted(os.path.join(NATIVE, f) for f in os.listdir(NATIVE) if f.endswith(".cpp"))
    inputs = list(sources) + [__file__]
    for d in [NATIVE] + [os.path.dirname(s) for s in sources]:
        inputs += [os.path.join(d, f) for f in os.listdir(d) if f.endswith(".h")]
    if not force and os.path.exists(lib) and all(os.path.getmtime(p) <= os.path.getmtime(lib) for p in inputs):
        return lib
    cxx = os.environ.get("CXX") or shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
    if cxx is None:
        sys.exit("thermal_viewer: no C++ compiler found (set CXX)")
    usb = _libusb_flags()
    if usb is None:
        print("thermal_viewer: libusb 1.0 not found; building without the USB cameras", file=sys.stderr)
        usb = ["-DTV_NO_USB"]
    os.makedirs(BUILD, exist_ok=True)
    cmd = [cxx, "-std=gnu++17", "-O2", "-g", "-fPIC", "-shared", "-pthread", "-Wall", "-Wextra",
           f"-I{COMP}", f"-I{NATIVE}", f"-I{os.path.join(NATIVE, 'shim')}"]
    cmd += [f for f in usb if not f.startswith(("-l", "-L"))] + sources
    cmd += [f for f in usb if f.startswith(("-l", "-L"))] + ["-o", lib]
    print("thermal_viewer: building the native library...", file=sys.stderr)
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.stderr.write(r.stderr[-6000:])
        sys.exit("thermal_viewer: the native library did not build")
    return lib


class Options(ctypes.Structure):
    """tv_options in native/tv_pipeline.cpp."""
    _fields_ = [
        ("temporal_frames", ctypes.c_int32), ("motion_threshold", ctypes.c_float),
        ("edge_smoothing", ctypes.c_float), ("median_filter", ctypes.c_int32),
        ("outlier_level", ctypes.c_float), ("valid_min", ctypes.c_float), ("valid_max", ctypes.c_float),
        ("range_min", ctypes.c_float), ("range_max", ctypes.c_float),
        ("mirror_x", ctypes.c_int32), ("mirror_y", ctypes.c_int32),
        ("scale", ctypes.c_int32), ("upscale", ctypes.c_int32),
        ("equalize", ctypes.c_float), ("detail", ctypes.c_float), ("palette", ctypes.c_int32),
        ("overlay_enabled", ctypes.c_int32), ("overlay_kinds", ctypes.c_uint32),
        ("fahrenheit", ctypes.c_int32), ("hold_ms", ctypes.c_uint32),
    ]


class FrameInfo(ctypes.Structure):
    """tv_frame_info in native/tv_pipeline.cpp."""
    _fields_ = [
        ("width", ctypes.c_int32), ("height", ctypes.c_int32),
        ("sensor_width", ctypes.c_int32), ("sensor_height", ctypes.c_int32),
        ("sequence", ctypes.c_uint32), ("stats_valid", ctypes.c_int32),
        ("min", ctypes.c_float), ("max", ctypes.c_float), ("mean", ctypes.c_float),
        ("median", ctypes.c_float), ("center", ctypes.c_float),
        ("fps", ctypes.c_float), ("process_ms", ctypes.c_float),
        ("emissivity", ctypes.c_float), ("reflected", ctypes.c_float),
    ]


MAX_IMAGE_BYTES = 8192 * 4800 * 3  # 20x of the largest sensor is far below this


class Native:
    """The native library: one pipeline, at most one camera."""

    def __init__(self, path):
        L = self.lib = ctypes.CDLL(path)
        c, i, f, p, s = ctypes.c_char_p, ctypes.c_int, ctypes.c_float, ctypes.c_void_p, ctypes.c_char_p
        for name, res, args in [
            ("tv_camera_count", i, []), ("tv_camera_key", c, [i]), ("tv_camera_label", c, [i]),
            ("tv_camera_arg_hint", c, [i]), ("tv_camera_unavailable", c, [i]), ("tv_camera_scale", i, [i]),
            ("tv_camera_outlier_level", f, [i]), ("tv_camera_action_count", i, [i]),
            ("tv_camera_action", c, [i, i]),
            ("tv_palette_count", i, []), ("tv_palette_name", c, [i]),
            ("tv_material_count", i, []), ("tv_material_label", c, [i]), ("tv_material_emissivity", f, [i]),
            ("tv_default_options", None, [ctypes.POINTER(Options)]),
            ("tv_create", p, []), ("tv_destroy", None, [p]),
            ("tv_set_options", None, [p, ctypes.POINTER(Options)]),
            ("tv_open", i, [p, i, s]), ("tv_close", None, [p]), ("tv_running", i, [p]),
            ("tv_status", None, [p, ctypes.c_char_p, i]), ("tv_action", None, [p, s]),
            ("tv_toggle_spot", i, [p, f, f, i]), ("tv_clear_spots", None, [p]),
            ("tv_spots_text", None, [p, ctypes.c_char_p, i]),
            ("tv_get_frame", i, [p, ctypes.c_void_p, i, ctypes.POINTER(FrameInfo)]),
            ("tv_get_temps", i, [p, ctypes.POINTER(ctypes.c_float), i]),
        ]:
            fn = getattr(L, name)
            fn.restype, fn.argtypes = res, args
        self.ctx = L.tv_create()
        self._buf = None
        self._text = ctypes.create_string_buffer(1024)

    def close(self):
        if self.ctx:
            self.lib.tv_destroy(self.ctx)
            self.ctx = None

    def cameras(self):
        L = self.lib
        return [{
            "key": L.tv_camera_key(n).decode(), "label": L.tv_camera_label(n).decode(),
            "arg_hint": L.tv_camera_arg_hint(n).decode(), "unavailable": L.tv_camera_unavailable(n).decode(),
            "scale": L.tv_camera_scale(n), "outlier_level": L.tv_camera_outlier_level(n),
            "actions": [L.tv_camera_action(n, k).decode() for k in range(L.tv_camera_action_count(n))],
        } for n in range(L.tv_camera_count())]

    def palettes(self):
        return [self.lib.tv_palette_name(n).decode() for n in range(self.lib.tv_palette_count())]

    def materials(self):
        return [(self.lib.tv_material_label(n).decode(), self.lib.tv_material_emissivity(n))
                for n in range(self.lib.tv_material_count())]

    def default_options(self):
        o = Options()
        self.lib.tv_default_options(ctypes.byref(o))
        return o

    def set_options(self, o):
        self.lib.tv_set_options(self.ctx, ctypes.byref(o))

    def open(self, index, arg=""):
        return self.lib.tv_open(self.ctx, index, arg.encode()) == 0

    def stop(self):
        self.lib.tv_close(self.ctx)

    def running(self):
        return bool(self.lib.tv_running(self.ctx))

    def status(self):
        self.lib.tv_status(self.ctx, self._text, len(self._text))
        return self._text.value.decode(errors="replace")

    def action(self, name):
        self.lib.tv_action(self.ctx, name.encode())

    def toggle_spot(self, fx, fy, material):
        return bool(self.lib.tv_toggle_spot(self.ctx, fx, fy, material))

    def clear_spots(self):
        self.lib.tv_clear_spots(self.ctx)

    def spots_text(self):
        self.lib.tv_spots_text(self.ctx, self._text, len(self._text))
        return self._text.value.decode(errors="replace")

    def frame(self):
        """(rgb bytes or None when there is no new image, FrameInfo or None before the first frame)."""
        info = FrameInfo()
        if self._buf is None:
            self._buf = ctypes.create_string_buffer(640 * 480 * 3)
        got = self.lib.tv_get_frame(self.ctx, self._buf, len(self._buf), ctypes.byref(info))
        need = info.width * info.height * 3
        if not got and 0 < need <= MAX_IMAGE_BYTES and need > len(self._buf):
            self._buf = ctypes.create_string_buffer(need)  # the image grew: take it now
            got = self.lib.tv_get_frame(self.ctx, self._buf, len(self._buf), ctypes.byref(info))
        if info.width == 0:
            return None, None
        return (self._buf.raw[:need] if got else None), info

    def temps(self, count):
        buf = (ctypes.c_float * count)()
        return buf if self.lib.tv_get_temps(self.ctx, buf, count) == count else None


# ---------------------------------------------------------------- window

def run_ui(native, args):
    from PyQt6 import QtCore, QtGui, QtWidgets
    Qt = QtCore.Qt

    class ImageView(QtWidgets.QWidget):
        """Shows the pipeline's image, whole, at the largest size that fits; clicks place spots."""
        clicked = QtCore.pyqtSignal(float, float)

        def __init__(self):
            super().__init__()
            self.image = None
            self.smooth = False
            self.setMinimumSize(320, 240)
            self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Expanding)
            self.setCursor(Qt.CursorShape.CrossCursor)

        def set_image(self, image):
            self.image = image
            self.update()

        def _target(self):
            if self.image is None:
                return QtCore.QRectF()
            iw, ih = self.image.width(), self.image.height()
            k = min(self.width() / iw, self.height() / ih)
            w, h = iw * k, ih * k
            return QtCore.QRectF((self.width() - w) / 2, (self.height() - h) / 2, w, h)

        def paintEvent(self, _):
            p = QtGui.QPainter(self)
            p.fillRect(self.rect(), QtGui.QColor(24, 24, 24))
            if self.image is None:
                p.setPen(QtGui.QColor(150, 150, 150))
                p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "No image.\nChoose a camera and press Connect.")
                return
            # The pipeline's own scaling is what is under test: the window
            # only enlarges whole pixels unless asked to smooth.
            p.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform, self.smooth)
            p.drawImage(self._target(), self.image)

        def mousePressEvent(self, e):
            r = self._target()
            if e.button() != Qt.MouseButton.LeftButton or r.isEmpty() or not r.contains(e.position()):
                return
            self.clicked.emit((e.position().x() - r.x()) / r.width(), (e.position().y() - r.y()) / r.height())

    class Window(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("Thermal viewer")
            self.cameras = native.cameras()
            self.opt = native.default_options()
            self.rgb = None
            self.info = None
            self.loading = False
            self.frames_seen = 0

            self.view = ImageView()
            self.view.clicked.connect(self.on_click)
            self.setCentralWidget(self.view)
            panel = QtWidgets.QWidget()
            col = QtWidgets.QVBoxLayout(panel)
            col.setContentsMargins(8, 8, 8, 8)
            col.addWidget(self.camera_group())
            col.addWidget(self.readings_group())
            col.addWidget(self.radiometric_group())
            col.addWidget(self.isp_group())
            col.addWidget(self.overlay_group())
            col.addWidget(self.spots_group())
            col.addStretch(1)
            scroll = QtWidgets.QScrollArea()
            scroll.setWidget(panel)
            scroll.setWidgetResizable(True)
            scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
            dock = QtWidgets.QDockWidget("Controls")
            dock.setWidget(scroll)
            dock.setFeatures(QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetMovable |
                             QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetFloatable)
            dock.setMinimumWidth(340)
            self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dock)
            self.status_label = QtWidgets.QLabel()
            self.statusBar().addWidget(self.status_label, 1)
            self.resize(1180, 760)

            self.on_camera_changed()
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.tick)
            self.timer.start(15)

        # ---- control helpers: each writes one field of the options and applies them
        def spin(self, field, lo, hi, step, decimals=1, suffix=""):
            if decimals == 0:
                w = QtWidgets.QSpinBox()
                w.setRange(int(lo), int(hi))
                w.setSingleStep(int(step))
                w.setValue(int(getattr(self.opt, field)))
            else:
                w = QtWidgets.QDoubleSpinBox()
                w.setDecimals(decimals)
                w.setRange(lo, hi)
                w.setSingleStep(step)
                w.setValue(getattr(self.opt, field))
            w.setSuffix(suffix)
            w.setKeyboardTracking(False)
            w.valueChanged.connect(lambda v, f=field: self.set_option(f, v))
            self.widgets[field] = w
            return w

        def check(self, field, text):
            w = QtWidgets.QCheckBox(text)
            w.setChecked(bool(getattr(self.opt, field)))
            w.toggled.connect(lambda v, f=field: self.set_option(f, int(v)))
            self.widgets[field] = w
            return w

        def combo(self, field, items):
            w = QtWidgets.QComboBox()
            w.addItems(items)
            w.setCurrentIndex(int(getattr(self.opt, field)))
            w.currentIndexChanged.connect(lambda v, f=field: self.set_option(f, v))
            self.widgets[field] = w
            return w

        def set_option(self, field, value):
            if self.loading:
                return
            setattr(self.opt, field, value)
            native.set_options(self.opt)

        def load_options(self):
            """Shows self.opt in the controls and applies it."""
            self.loading = True
            for field, w in self.widgets.items():
                v = getattr(self.opt, field)
                if isinstance(w, QtWidgets.QCheckBox):
                    w.setChecked(bool(v))
                elif isinstance(w, QtWidgets.QComboBox):
                    w.setCurrentIndex(int(v))
                else:
                    w.setValue(int(round(v)) if isinstance(w, QtWidgets.QSpinBox) else v)
            for bit, w in self.kind_boxes.items():
                w.setChecked(bool(self.opt.overlay_kinds & (1 << bit)))
            self.loading = False
            native.set_options(self.opt)

        # ---- groups
        widgets = {}

        def camera_group(self):
            g = QtWidgets.QGroupBox("Camera")
            lay = QtWidgets.QVBoxLayout(g)
            self.camera_box = QtWidgets.QComboBox()
            for c in self.cameras:
                self.camera_box.addItem(c["label"])
            self.camera_box.currentIndexChanged.connect(self.on_camera_changed)
            lay.addWidget(self.camera_box)
            self.arg_edit = QtWidgets.QLineEdit()
            lay.addWidget(self.arg_edit)
            self.camera_note = QtWidgets.QLabel()
            self.camera_note.setWordWrap(True)
            lay.addWidget(self.camera_note)
            row = QtWidgets.QHBoxLayout()
            self.connect_button = QtWidgets.QPushButton("Connect")
            self.connect_button.clicked.connect(self.on_connect)
            row.addWidget(self.connect_button)
            save = QtWidgets.QPushButton("Save image...")
            save.clicked.connect(self.on_save)
            row.addWidget(save)
            lay.addLayout(row)
            self.actions_row = QtWidgets.QVBoxLayout()
            lay.addLayout(self.actions_row)
            return g

        def readings_group(self):
            g = QtWidgets.QGroupBox("Readings (radiometric stage)")
            lay = QtWidgets.QVBoxLayout(g)
            self.readings = QtWidgets.QLabel("-")
            self.readings.setTextFormat(Qt.TextFormat.PlainText)
            font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)
            self.readings.setFont(font)
            lay.addWidget(self.readings)
            return g

        def radiometric_group(self):
            g = QtWidgets.QGroupBox("Radiometric")
            f = QtWidgets.QFormLayout(g)
            f.addRow("Temporal filter", self.spin("temporal_frames", 1, 16, 1, 0, " frames"))
            f.addRow("Motion threshold", self.spin("motion_threshold", 0.05, 10, 0.05, 2, " °C"))
            f.addRow("Edge smoothing", self.spin("edge_smoothing", 0, 10, 0.1, 2, " °C"))
            f.addRow("", self.check("median_filter", "3x3 median filter"))
            f.addRow("Outlier level", self.spin("outlier_level", 0, 100, 0.5, 1, " °C"))
            f.addRow("Valid from", self.spin("valid_min", -273, 2000, 5, 0, " °C"))
            f.addRow("Valid to", self.spin("valid_max", -273, 2000, 5, 0, " °C"))
            return g

        def isp_group(self):
            g = QtWidgets.QGroupBox("ISP")
            f = QtWidgets.QFormLayout(g)
            f.addRow("Palette", self.combo("palette", native.palettes()))
            f.addRow("Range from", self.spin("range_min", -273, 2000, 0.5, 1, " °C"))
            f.addRow("Range to", self.spin("range_max", -273, 2000, 0.5, 1, " °C"))
            fit = QtWidgets.QPushButton("Set range to this scene")
            fit.clicked.connect(self.on_fit_range)
            f.addRow("", fit)
            f.addRow("Equalization", self.spin("equalize", 0, 100, 5, 0, " %"))
            f.addRow("Detail", self.spin("detail", 0, 4, 0.1, 2))
            f.addRow("Scale", self.spin("scale", 1, 20, 1, 0, "x"))
            f.addRow("Upscale", self.combo("upscale", ["Nearest", "Bilinear", "Bicubic"]))
            row = QtWidgets.QHBoxLayout()
            row.addWidget(self.check("mirror_x", "Mirror X"))
            row.addWidget(self.check("mirror_y", "Mirror Y"))
            f.addRow("", row)
            smooth = QtWidgets.QCheckBox("Smooth when the window enlarges it")
            smooth.toggled.connect(self.on_smooth)
            f.addRow("", smooth)
            return g

        def overlay_group(self):
            g = QtWidgets.QGroupBox("Overlay")
            f = QtWidgets.QFormLayout(g)
            f.addRow("", self.check("overlay_enabled", "Enabled"))
            row = QtWidgets.QHBoxLayout()
            self.kind_boxes = {}
            for bit, text in ((0, "Min"), (1, "Max"), (2, "Centre"), (4, "Spots")):
                w = QtWidgets.QCheckBox(text)
                w.setChecked(bool(self.opt.overlay_kinds & (1 << bit)))
                w.toggled.connect(lambda v, b=bit: self.on_kind(b, v))
                self.kind_boxes[bit] = w
                row.addWidget(w)
            f.addRow("Marks", row)
            f.addRow("Unit", self.combo("fahrenheit", ["°C", "°F"]))
            f.addRow("Value hold", self.spin("hold_ms", 0, 10000, 100, 0, " ms"))
            return g

        def spots_group(self):
            g = QtWidgets.QGroupBox("Spot meters")
            lay = QtWidgets.QVBoxLayout(g)
            note = QtWidgets.QLabel("Click the image to place a spot; click a spot to remove it.")
            note.setWordWrap(True)
            lay.addWidget(note)
            self.material_box = QtWidgets.QComboBox()
            for label, e in native.materials():
                self.material_box.addItem(f"{label} ({e:.2f})" if e > 0 else label)
            form = QtWidgets.QFormLayout()
            form.addRow("Material", self.material_box)
            lay.addLayout(form)
            self.spots_label = QtWidgets.QLabel("No spots")
            self.spots_label.setWordWrap(True)
            lay.addWidget(self.spots_label)
            clear = QtWidgets.QPushButton("Remove all spots")
            clear.clicked.connect(self.on_clear_spots)
            lay.addWidget(clear)
            return g

        # ---- camera
        def camera(self):
            return self.cameras[self.camera_box.currentIndex()]

        def on_camera_changed(self):
            c = self.camera()
            self.arg_edit.setVisible(bool(c["arg_hint"]))
            self.arg_edit.setPlaceholderText(c["arg_hint"])
            self.camera_note.setVisible(bool(c["unavailable"]))
            self.camera_note.setText("Not available: " + c["unavailable"])
            self.connect_button.setEnabled(not c["unavailable"] or native.running())
            while self.actions_row.count():
                self.actions_row.takeAt(0).widget().deleteLater()
            for name in c["actions"]:
                b = QtWidgets.QPushButton(name)
                b.clicked.connect(lambda _, n=name: native.action(n))
                self.actions_row.addWidget(b)

        def on_connect(self):
            if native.running():
                native.stop()
            else:
                c = self.camera()
                # What suits this camera, as its example configuration has it.
                self.opt.scale = c["scale"]
                self.opt.outlier_level = c["outlier_level"]
                self.load_options()
                self.view.set_image(None)
                self.info = None
                native.open(self.camera_box.currentIndex(), self.arg_edit.text().strip())
            self.sync_connection()

        def sync_connection(self):
            running = native.running()
            self.connect_button.setText("Disconnect" if running else "Connect")
            self.camera_box.setEnabled(not running)
            self.arg_edit.setEnabled(not running)
            self.connect_button.setEnabled(running or not self.camera()["unavailable"])
            for n in range(self.actions_row.count()):
                self.actions_row.itemAt(n).widget().setEnabled(running)

        # ---- controls with a little more to them
        def on_kind(self, bit, on):
            kinds = self.opt.overlay_kinds
            self.set_option("overlay_kinds", (kinds | (1 << bit)) if on else (kinds & ~(1 << bit)))

        def on_smooth(self, on):
            self.view.smooth = on
            self.view.update()

        def on_fit_range(self):
            if self.info is None or not self.info.stats_valid:
                return
            lo, hi = self.info.min, self.info.max
            if hi - lo < 1.0:
                lo, hi = lo - 0.5, hi + 0.5
            self.opt.range_min, self.opt.range_max = round(lo, 1), round(hi, 1)
            self.load_options()

        def on_click(self, fx, fy):
            native.toggle_spot(fx, fy, self.material_box.currentIndex())
            self.show_spots()

        def on_clear_spots(self):
            native.clear_spots()
            self.show_spots()

        def show_spots(self):
            text = native.spots_text()
            if not text:
                self.spots_label.setText("No spots")
                return
            lines = []
            for n, s in enumerate(text.split(";"), 1):
                x, y, material, color = s.split(",")
                lines.append(f"{n}: {x}%, {y}%  {material}  (colour {color})")
            self.spots_label.setText("\n".join(lines))

        def on_save(self):
            if self.view.image is None:
                return
            name = time.strftime("thermal-%Y%m%d-%H%M%S.png")
            path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save image", name, "PNG (*.png)")
            if path:
                self.view.image.save(path, "PNG")

        # ---- frames
        def tick(self):
            rgb, info = native.frame()
            if info is not None:
                self.info = info
            if rgb is not None:
                self.rgb = rgb  # the image's pixels: kept while it is shown
                self.view.set_image(QtGui.QImage(self.rgb, info.width, info.height, info.width * 3,
                                                 QtGui.QImage.Format.Format_RGB888))
                self.frames_seen += 1
            self.status_label.setText(native.status())
            if self.connect_button.text() == "Disconnect" and not native.running():
                self.sync_connection()  # the camera stopped by itself; the status says why
            i = self.info
            if i is None:
                self.readings.setText("-")
                return

            def unit(v):
                return f"{v * 9 / 5 + 32:7.2f} °F" if self.opt.fahrenheit else f"{v:7.2f} °C"

            lines = []
            if i.stats_valid:
                lines += [f"min    {unit(i.min)}", f"max    {unit(i.max)}", f"mean   {unit(i.mean)}",
                          f"median {unit(i.median)}", f"centre {unit(i.center)}"]
            else:
                lines.append("no valid statistics")
            source = (f"emissivity {i.emissivity:.2f}, reflected {i.reflected:.1f} °C"
                      if i.emissivity == i.emissivity else "emissivity: as a black body")
            lines += ["", f"sensor {i.sensor_width} x {i.sensor_height}, image {i.width} x {i.height}",
                      f"{i.fps:4.1f} frames/s, {i.process_ms:.2f} ms processing", source]
            self.readings.setText("\n".join(lines))

        def closeEvent(self, e):
            self.timer.stop()
            native.stop()
            e.accept()

    app = QtWidgets.QApplication(sys.argv[:1])
    w = Window()
    keys = [c["key"] for c in w.cameras]
    if args.camera:
        if args.camera not in keys:
            sys.exit(f"thermal_viewer: no camera type '{args.camera}' (one of: {', '.join(keys)})")
        w.camera_box.setCurrentIndex(keys.index(args.camera))
    if args.arg:
        w.arg_edit.setText(args.arg)
    w.show()
    if args.connect:
        w.on_connect()
    if args.screenshot:
        # For checking the window without a display: a moment of frames, then a picture of it.
        def shoot():
            w.grab().save(args.screenshot)
            w.close()
        QtCore.QTimer.singleShot(int(args.screenshot_after * 1000), shoot)
    code = app.exec()
    native.close()
    return code


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--camera", help="camera type to select (see --list)")
    ap.add_argument("--arg", help="the camera's argument (MLX90640: bus[,address[,rate]])")
    ap.add_argument("--connect", action="store_true", help="connect at once")
    ap.add_argument("--list", action="store_true", help="list the camera types and exit")
    ap.add_argument("--rebuild", action="store_true", help="build the native library again")
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    ap.add_argument("--screenshot-after", type=float, default=2.0, help=argparse.SUPPRESS)
    args = ap.parse_args()
    native = Native(build_library(args.rebuild))
    if args.list:
        for c in native.cameras():
            print(f"{c['key']:20} {c['label']}" + (f"  [not available: {c['unavailable']}]" if c["unavailable"] else ""))
        return 0
    try:
        import PyQt6  # noqa: F401
    except ImportError:
        sys.exit("thermal_viewer: PyQt6 is not installed (pip install PyQt6)")
    return run_ui(native, args)


if __name__ == "__main__":
    sys.exit(main())
