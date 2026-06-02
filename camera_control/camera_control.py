#!/usr/bin/env python3
"""
camera_control_tabs_metadata_v8.py

First-pass ROS2 + PySide6 camera cockpit for cambuffer_recorder_ng, with Startup/Shutdown tab.

What it does now:
  - Discovers camera nodes that expose /<node>/get_status
  - Shows CBRNG status via cambuffer_recorder_ng/srv/GetStatus
  - Applies camera settings via cambuffer_recorder_ng/srv/ApplySettings
  - Tracks clean/dirty settings against camera read-back with amber/green/red fields
  - Starts/stops recording via std_srvs/srv/Trigger
  - Adds a session/trial metadata panel
  - Creates a session folder + session.yaml before recording/apply-with-record
  - Passes node-specific output.dir, output.prefix, and metadata_path to CBRNG

Install/run sketch:
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  source ~/ros2_ws/.venv_gui/bin/activate
  python3 camera_control_with_metadata.py
"""

from __future__ import annotations

import os
import re
import sys
import getpass
import socket
import shlex
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Callable

from PySide6 import QtCore, QtWidgets

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float64
from cambuffer_recorder_ng.srv import ApplySettings, GetStatus


DISCOVERY_INTERVAL_MS = 1500
STATUS_INTERVAL_MS = 1000
SPIN_INTERVAL_MS = 10


# --------------------------
# Small helpers
# --------------------------
def full_node_name(name: str, namespace: str) -> str:
    if namespace in ("", "/"):
        return f"/{name.lstrip('/')}"
    return f"/{namespace.strip('/')}/{name.lstrip('/')}"


def safe_token(text: str, fallback: str = "unset") -> str:
    text = (text or "").strip()
    if not text:
        return fallback
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", text)
    text = text.strip("._-")
    return text or fallback


def yaml_quote(value: str) -> str:
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n') + '"'


def flat_yaml(settings: Dict[str, Any]) -> str:
    lines = []
    for key, value in settings.items():
        if value is None:
            continue
        if isinstance(value, bool):
            rendered = "true" if value else "false"
        elif isinstance(value, (int, float)):
            rendered = str(value)
        else:
            rendered = yaml_quote(str(value))
        lines.append(f"{key}: {rendered}")
    return "\n".join(lines) + "\n"



# --------------------------
# Lightweight YAML helpers for CBRNG status/apply settings
# --------------------------
def _parse_scalar(text: str) -> Any:
    text = str(text).strip()
    if text == "":
        return ""
    if (text.startswith("\"") and text.endswith("\"")) or (text.startswith("'") and text.endswith("'")):
        return text[1:-1]
    low = text.lower()
    if low == "true":
        return True
    if low == "false":
        return False
    if low in ("null", "none", "~"):
        return None
    try:
        if any(ch in text for ch in [".", "e", "E"]):
            return float(text)
        return int(text)
    except ValueError:
        return text


def _flatten_dict(d: Dict[str, Any], prefix: str = "") -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for key, value in d.items():
        full = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, dict):
            out.update(_flatten_dict(value, full))
        else:
            out[full] = value
    return out


def parse_settings_yaml(text: str) -> Dict[str, Any]:
    """Parse CBRNG settings YAML into flat dotted keys.

    Prefer PyYAML if available. Fall back to a tiny parser that handles the
    simple flat YAML we send plus basic one/two-level ROS param dumps.
    """
    text = text or ""
    if not text.strip():
        return {}
    try:
        import yaml  # type: ignore
        data = yaml.safe_load(text)
        if isinstance(data, dict):
            # Some dumps may be /node: ros__parameters: ...; peel that if present.
            if len(data) == 1:
                only = next(iter(data.values()))
                if isinstance(only, dict) and "ros__parameters" in only:
                    data = only["ros__parameters"]
            return _flatten_dict(data)
    except Exception:
        pass

    # Fallback indentation parser, good enough for scalar mapping values.
    out: Dict[str, Any] = {}
    stack: List[Tuple[int, str]] = []
    for raw in text.splitlines():
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        line = raw.strip()
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip().strip("'\"")
        value = value.strip()
        while stack and stack[-1][0] >= indent:
            stack.pop()
        if value == "":
            stack.append((indent, key))
            continue
        prefix = ".".join(k for _, k in stack)
        full = f"{prefix}.{key}" if prefix else key
        out[full] = _parse_scalar(value)
    return out


def values_equal(a: Any, b: Any) -> bool:
    if a is None or b is None:
        return a is b
    if isinstance(a, bool) or isinstance(b, bool):
        return bool(a) == bool(b)
    try:
        return abs(float(a) - float(b)) < 1e-9
    except Exception:
        return str(a) == str(b)


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


@dataclass
class SessionMetadata:
    base_dir: str = str(Path.home() / "camera_sessions")
    experiment_id: str = ""
    animal_id: str = ""
    timepoint: str = ""
    group: str = ""
    operator: str = getpass.getuser()
    trial_id: str = "trial001"
    speed_cm_s: str = ""
    condition: str = ""
    notes: str = ""
    circular_trigger_type: str = "mid"
    pre_trigger_s: str = ""
    post_trigger_s: str = ""

    def required_missing(self) -> List[str]:
        missing = []
        if not self.animal_id.strip():
            missing.append("Animal ID")
        if not self.trial_id.strip():
            missing.append("Trial ID")
        return missing

    def session_label(self) -> str:
        date = datetime.now().strftime("%Y-%m-%d")
        parts = [
            date,
            safe_token(self.animal_id, "animal"),
            safe_token(self.timepoint, "timepoint"),
            safe_token(self.trial_id, "trial"),
        ]
        return "_".join(parts)

    def session_dir(self) -> Path:
        return Path(os.path.expanduser(self.base_dir)).resolve() / self.session_label()

    def to_yaml(self) -> str:
        data = asdict(self)
        data["created_local_time"] = datetime.now().isoformat(timespec="seconds")
        data["session_dir"] = str(self.session_dir())
        lines = ["session_metadata:"]
        for key, value in data.items():
            lines.append(f"  {key}: {yaml_quote(value)}")
        return "\n".join(lines) + "\n"

    def write_session_yaml(self) -> Path:
        path = self.session_dir() / "session.yaml"
        write_text(path, self.to_yaml())
        return path

    def node_prefix(self, node_name: str) -> str:
        return "_".join([
            safe_token(node_name.strip("/"), "cam"),
            safe_token(self.animal_id, "animal"),
            safe_token(self.timepoint, "timepoint"),
            safe_token(self.trial_id, "trial"),
        ])


# --------------------------
# ROS client node
# --------------------------
class CameraControlRos(Node):
    def __init__(self):
        super().__init__("camera_control_gui")
        self._apply_clients: Dict[str, Any] = {}
        self._status_clients: Dict[str, Any] = {}
        self._start_clients: Dict[str, Any] = {}
        self._stop_clients: Dict[str, Any] = {}
        self._event_subs: Dict[str, Any] = {}
        self._event_callback = None

    def set_event_callback(self, callback):
        """Set GUI callback for camera event log lines.

        In this app rclpy.spin_once() is called from the Qt event loop, so these
        callbacks run on the GUI thread and can safely update widgets.
        """
        self._event_callback = callback

    def _emit_event(self, full_name: str, topic_label: str, payload: str):
        if self._event_callback is not None:
            self._event_callback(f"{full_name} {topic_label}: {payload}")
        else:
            self.get_logger().info(f"{full_name} {topic_label}: {payload}")

    def ensure_event_subscriptions(self, full_name: str):
        """Subscribe once to the useful CBRNG event/status topics for a camera."""
        specs = [
            ("settings_event", String, lambda msg: msg.data),
            ("recording_event", String, lambda msg: msg.data),
            ("storage/free_gib", Float64, lambda msg: f"{msg.data:.2f} GiB free"),
        ]
        for suffix, msg_type, formatter in specs:
            topic = f"{full_name}/{suffix}"
            if topic in self._event_subs:
                continue

            def cb(msg, full=full_name, label=suffix, fmt=formatter):
                try:
                    text = fmt(msg)
                except Exception as e:
                    text = f"<format error: {e}>"
                self._emit_event(full, label, text)

            self._event_subs[topic] = self.create_subscription(msg_type, topic, cb, 10)

    def discover_camera_nodes(self) -> List[Tuple[str, str, str]]:
        """Return (name, namespace, full_name) for CBRNG nodes exposing get_status.

        Important: do not use a loose name.startswith("cam") fallback here, because
        this GUI node is named /camera_control_gui and would discover itself.
        """
        service_types = {name: types for name, types in self.get_service_names_and_types()}
        out = []
        own_full = full_node_name(self.get_name(), self.get_namespace())

        for name, ns in self.get_node_names_and_namespaces():
            full = full_node_name(name, ns)
            if full == own_full:
                continue

            status_srv = f"{full}/get_status"
            types = service_types.get(status_srv, [])
            if "cambuffer_recorder_ng/srv/GetStatus" in types:
                out.append((name, ns, full))

        out.sort(key=lambda x: x[2])
        return out

    def _client(self, cache: Dict[str, Any], srv_type: Any, full_name: str, suffix: str):
        key = f"{full_name}/{suffix}"
        if key not in cache:
            cache[key] = self.create_client(srv_type, key)
        return cache[key]

    def get_status_async(self, full_name: str):
        cli = self._client(self._status_clients, GetStatus, full_name, "get_status")
        if not cli.service_is_ready():
            cli.wait_for_service(timeout_sec=0.0)
        return cli.call_async(GetStatus.Request())

    def apply_settings_async(
        self,
        full_name: str,
        settings_yaml: str,
        merge_with_current: bool = True,
        restart_if_active: bool = False,
        activate_after_apply: bool = False,
    ):
        cli = self._client(self._apply_clients, ApplySettings, full_name, "apply_settings")
        req = ApplySettings.Request()
        req.settings_yaml = settings_yaml
        req.merge_with_current = merge_with_current
        req.restart_if_active = restart_if_active
        req.activate_after_apply = activate_after_apply
        return cli.call_async(req)

    def trigger_async(self, full_name: str, start: bool):
        cache = self._start_clients if start else self._stop_clients
        suffix = "start_recording" if start else "stop_recording"
        cli = self._client(cache, Trigger, full_name, suffix)
        return cli.call_async(Trigger.Request())


# --------------------------
# Metadata panel
# --------------------------
class MetadataPanel(QtWidgets.QGroupBox):
    metadata_changed = QtCore.Signal()

    def __init__(self):
        super().__init__("Session / Trial Metadata")
        self.confirmed = False

        self.base_dir = QtWidgets.QLineEdit(str(Path.home() / "camera_sessions"))
        self.experiment_id = QtWidgets.QLineEdit()
        self.animal_id = QtWidgets.QLineEdit()
        self.timepoint = QtWidgets.QLineEdit()
        self.group = QtWidgets.QLineEdit()
        self.operator = QtWidgets.QLineEdit(getpass.getuser())
        self.trial_id = QtWidgets.QLineEdit("trial001")
        self.speed_cm_s = QtWidgets.QLineEdit()
        self.condition = QtWidgets.QLineEdit()
        self.notes = QtWidgets.QPlainTextEdit()
        self.notes.setMaximumHeight(70)

        self.trigger_type = QtWidgets.QComboBox()
        self.trigger_type.addItems(["start", "mid", "end"])
        self.trigger_type.setCurrentText("mid")
        self.pre_trigger_s = QtWidgets.QLineEdit()
        self.post_trigger_s = QtWidgets.QLineEdit()

        self.confirm_btn = QtWidgets.QPushButton("Confirm metadata")
        self.status_label = QtWidgets.QLabel("Not confirmed")
        self.status_label.setStyleSheet("font-weight: bold;")

        form = QtWidgets.QFormLayout()
        form.addRow("Base output dir", self.base_dir)
        form.addRow("Experiment", self.experiment_id)
        form.addRow("Animal ID *", self.animal_id)
        form.addRow("Timepoint", self.timepoint)
        form.addRow("Group", self.group)
        form.addRow("Operator", self.operator)
        form.addRow("Trial ID *", self.trial_id)
        form.addRow("Speed (cm/s)", self.speed_cm_s)
        form.addRow("Condition", self.condition)
        form.addRow("Circular trigger", self.trigger_type)
        form.addRow("Pre-trigger (s)", self.pre_trigger_s)
        form.addRow("Post-trigger (s)", self.post_trigger_s)
        form.addRow("Notes", self.notes)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.confirm_btn)
        row.addWidget(self.status_label)
        row.addStretch(1)

        layout = QtWidgets.QVBoxLayout()
        layout.addLayout(form)
        layout.addLayout(row)
        self.setLayout(layout)

        for widget in [
            self.base_dir, self.experiment_id, self.animal_id, self.timepoint,
            self.group, self.operator, self.trial_id, self.speed_cm_s,
            self.condition, self.pre_trigger_s, self.post_trigger_s,
        ]:
            widget.textChanged.connect(self._mark_dirty)
        self.notes.textChanged.connect(self._mark_dirty)
        self.trigger_type.currentTextChanged.connect(self._mark_dirty)
        self.confirm_btn.clicked.connect(self.confirm)

    def _mark_dirty(self):
        self.confirmed = False
        self.status_label.setText("Not confirmed")
        self.metadata_changed.emit()

    def current_metadata(self) -> SessionMetadata:
        return SessionMetadata(
            base_dir=self.base_dir.text().strip(),
            experiment_id=self.experiment_id.text().strip(),
            animal_id=self.animal_id.text().strip(),
            timepoint=self.timepoint.text().strip(),
            group=self.group.text().strip(),
            operator=self.operator.text().strip(),
            trial_id=self.trial_id.text().strip(),
            speed_cm_s=self.speed_cm_s.text().strip(),
            condition=self.condition.text().strip(),
            notes=self.notes.toPlainText().strip(),
            circular_trigger_type=self.trigger_type.currentText(),
            pre_trigger_s=self.pre_trigger_s.text().strip(),
            post_trigger_s=self.post_trigger_s.text().strip(),
        )

    def confirm(self) -> bool:
        md = self.current_metadata()
        missing = md.required_missing()
        if missing:
            QtWidgets.QMessageBox.warning(self, "Metadata incomplete", "Missing: " + ", ".join(missing))
            return False
        path = md.write_session_yaml()
        self.confirmed = True
        self.status_label.setText(f"Confirmed → {path.parent.name}")
        self.metadata_changed.emit()
        return True

    def ensure_confirmed(self, parent: QtWidgets.QWidget, reason: str) -> Optional[SessionMetadata]:
        md = self.current_metadata()
        missing = md.required_missing()
        if missing:
            QtWidgets.QMessageBox.warning(parent, "Metadata required", f"{reason}\n\nMissing: " + ", ".join(missing))
            return None
        if not self.confirmed:
            msg = QtWidgets.QMessageBox(parent)
            msg.setWindowTitle("Update metadata?")
            msg.setText(f"{reason}\n\nMetadata has not been confirmed for this trial.")
            edit = msg.addButton("Edit metadata", QtWidgets.QMessageBox.RejectRole)
            use = msg.addButton("Use current metadata", QtWidgets.QMessageBox.AcceptRole)
            cancel = msg.addButton("Cancel", QtWidgets.QMessageBox.DestructiveRole)
            msg.exec()
            clicked = msg.clickedButton()
            if clicked is cancel or clicked is edit:
                return None
            if clicked is use:
                md.write_session_yaml()
                self.confirmed = True
                self.status_label.setText(f"Confirmed → {md.session_dir().name}")
                self.metadata_changed.emit()
        return self.current_metadata()


class MetadataSummary(QtWidgets.QFrame):
    edit_requested = QtCore.Signal()

    def __init__(self, metadata_panel: MetadataPanel):
        super().__init__()
        self.metadata_panel = metadata_panel
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.setObjectName("MetadataSummary")

        self.label = QtWidgets.QLabel()
        self.label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.label.setMinimumWidth(300)
        self.edit_btn = QtWidgets.QPushButton("Edit metadata")
        self.confirm_btn = QtWidgets.QPushButton("Confirm")

        layout = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(8, 4, 8, 4)
        layout.addWidget(self.label, stretch=1)
        layout.addWidget(self.confirm_btn)
        layout.addWidget(self.edit_btn)
        self.setLayout(layout)

        self.edit_btn.clicked.connect(self.edit_requested.emit)
        self.confirm_btn.clicked.connect(self.metadata_panel.confirm)
        self.metadata_panel.metadata_changed.connect(self.update_summary)
        self.update_summary()

    def update_summary(self):
        md = self.metadata_panel.current_metadata()
        animal = md.animal_id or "animal?"
        timepoint = md.timepoint or "timepoint?"
        trial = md.trial_id or "trial?"
        speed = f"{md.speed_cm_s} cm/s" if md.speed_cm_s else "speed?"
        condition = md.condition or "condition?"
        status = "confirmed ✅" if self.metadata_panel.confirmed else "not confirmed"
        self.label.setText(
            f"Session: {animal} | {timepoint} | {trial} | {speed} | {condition} | {status}"
        )


# --------------------------
# Camera panel
# --------------------------
class CameraTable(QtWidgets.QTableWidget):
    COL_NAME = 0
    COL_STATE = 1
    COL_CONFIGURED = 2
    COL_RECORDING = 3
    COL_MODE = 4
    COL_OUTPUT = 5

    def __init__(self):
        super().__init__(0, 6)
        self.setHorizontalHeaderLabels(["Camera", "State", "Cfg", "Rec", "Mode", "Output"])
        header = self.horizontalHeader()
        header.setStretchLastSection(False)
        header.setSectionResizeMode(QtWidgets.QHeaderView.Interactive)
        header.setSectionResizeMode(self.COL_OUTPUT, QtWidgets.QHeaderView.Stretch)
        self.setColumnWidth(self.COL_NAME, 110)
        self.setColumnWidth(self.COL_STATE, 90)
        self.setColumnWidth(self.COL_CONFIGURED, 45)
        self.setColumnWidth(self.COL_RECORDING, 45)
        self.setColumnWidth(self.COL_MODE, 150)
        self.setMinimumWidth(650)
        self.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
        self.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.verticalHeader().setVisible(False)
        self._row_nodes: List[Tuple[str, str, str]] = []

    def set_nodes(self, nodes: List[Tuple[str, str, str]]):
        selected = set(self.selected_full_names())
        self.setRowCount(0)
        self._row_nodes = nodes
        for name, ns, full in nodes:
            row = self.rowCount()
            self.insertRow(row)
            for col, text in enumerate([full, "unknown", "?", "?", "", ""]):
                self.setItem(row, col, QtWidgets.QTableWidgetItem(text))
            if full in selected:
                self.selectRow(row)

    def selected_full_names(self) -> List[str]:
        out = []
        for idx in self.selectionModel().selectedRows():
            row = idx.row()
            if 0 <= row < len(self._row_nodes):
                out.append(self._row_nodes[row][2])
        return out

    def all_full_names(self) -> List[str]:
        return [x[2] for x in self._row_nodes]

    def update_status(self, full: str, status: Any):
        for row, (_, _, f) in enumerate(self._row_nodes):
            if f != full:
                continue
            self.item(row, self.COL_STATE).setText(status.state)
            self.item(row, self.COL_CONFIGURED).setText("yes" if status.configured else "no")
            self.item(row, self.COL_RECORDING).setText("yes" if status.recording else "no")
            self.item(row, self.COL_MODE).setText(status.mode)
            out = status.rolling_path_prefix or status.output_path or status.metadata_path
            self.item(row, self.COL_OUTPUT).setText(out)
            break


class CameraPanel(QtWidgets.QGroupBox):
    def __init__(self, ros: CameraControlRos, metadata_panel: MetadataPanel):
        super().__init__("Cameras")
        self.ros = ros
        self.metadata_panel = metadata_panel
        self.baseline_settings: Dict[str, Any] = {}
        self.settings_state = "unknown"  # unknown, synced, dirty, mixed
        self._populating_settings = False
        self._apply_batch_counter = 0
        self._pending_apply_batches: Dict[int, Dict[str, Any]] = {}

        self.discover_btn = QtWidgets.QPushButton("Discover cameras")
        self.refresh_btn = QtWidgets.QPushButton("Refresh status")
        self.table = CameraTable()

        # Settings widgets
        self.width = QtWidgets.QSpinBox(); self.width.setRange(1, 10000); self.width.setValue(2048)
        self.height = QtWidgets.QSpinBox(); self.height.setRange(1, 10000); self.height.setValue(700)
        self.offset_x = QtWidgets.QSpinBox(); self.offset_x.setRange(0, 10000); self.offset_x.setValue(0)
        self.offset_y = QtWidgets.QSpinBox(); self.offset_y.setRange(0, 10000); self.offset_y.setValue(194)
        self.exposure_us = QtWidgets.QDoubleSpinBox(); self.exposure_us.setRange(0, 1e8); self.exposure_us.setDecimals(1); self.exposure_us.setValue(2000.0)
        self.fps = QtWidgets.QDoubleSpinBox(); self.fps.setRange(0, 10000); self.fps.setDecimals(3); self.fps.setValue(100.0)
        self.gain_db = QtWidgets.QDoubleSpinBox(); self.gain_db.setRange(0, 1000); self.gain_db.setDecimals(2); self.gain_db.setValue(0.0)
        self.hw_trigger = QtWidgets.QCheckBox("Hardware trigger")
        self.expected_hw_fps = QtWidgets.QCheckBox("Set expected hardware FPS = FPS")
        self.expected_hw_fps.setChecked(True)

        self.mode = QtWidgets.QComboBox()
        self.mode.addItem("Keep current", None)
        self.mode.addItem("raw8mono_rolling", "raw8mono_rolling")
        self.mode.addItem("raw8bayerGBRG_rolling", "raw8bayerGBRG_rolling")
        self.mode.addItem("video_rgb24", "video_rgb24")

        self.pixel_format = QtWidgets.QComboBox()
        self.pixel_format.addItem("Keep current", None)
        self.pixel_format.addItem("mono8", "mono8")
        self.pixel_format.addItem("bayer_gbrg8", "bayer_gbrg8")
        self.pixel_format.addItem("rgb24", "rgb24")

        self.bayer_pattern = QtWidgets.QComboBox()
        self.bayer_pattern.addItem("Keep current", None)
        self.bayer_pattern.addItem("GBRG", "GBRG")
        self.bayer_pattern.addItem("none", "")

        self.output_kind = QtWidgets.QComboBox()
        self.output_kind.addItem("Auto from mode", "auto")
        self.output_kind.addItem("rolling_raw_binary", "rolling_raw_binary")
        self.output_kind.addItem("video_mp4", "video_mp4")

        self.read_btn = QtWidgets.QPushButton("Read from selected")
        self.revert_btn = QtWidgets.QPushButton("Revert edits")
        self.settings_sync_label = QtWidgets.QLabel("Settings: unknown")
        self.settings_sync_label.setStyleSheet("font-weight: bold;")

        self.apply_btn = QtWidgets.QPushButton("Apply settings")
        self.apply_start_btn = QtWidgets.QPushButton("Apply + start recording")
        self.start_btn = QtWidgets.QPushButton("Start recording")
        self.stop_btn = QtWidgets.QPushButton("Stop recording")
        self.preview_btn = QtWidgets.QPushButton("Open preview…")

        settings = QtWidgets.QFormLayout()
        settings.addRow("Mode", self.mode)
        settings.addRow("Pixel format", self.pixel_format)
        settings.addRow("Bayer pattern", self.bayer_pattern)
        settings.addRow("Output kind", self.output_kind)
        settings.addRow("Width", self.width)
        settings.addRow("Height", self.height)
        settings.addRow("Offset X", self.offset_x)
        settings.addRow("Offset Y", self.offset_y)
        settings.addRow("Exposure (us)", self.exposure_us)
        settings.addRow("FPS", self.fps)
        settings.addRow("Gain (dB)", self.gain_db)
        settings.addRow("", self.hw_trigger)
        settings.addRow("", self.expected_hw_fps)

        settings_box = QtWidgets.QGroupBox("Camera Settings")
        settings_box.setLayout(settings)

        sync_row = QtWidgets.QHBoxLayout()
        sync_row.addWidget(self.read_btn)
        sync_row.addWidget(self.revert_btn)
        sync_row.addStretch(1)
        sync_row.addWidget(self.settings_sync_label)

        buttons = QtWidgets.QGridLayout()
        buttons.addWidget(self.apply_btn, 0, 0)
        buttons.addWidget(self.apply_start_btn, 0, 1)
        buttons.addWidget(self.start_btn, 1, 0)
        buttons.addWidget(self.stop_btn, 1, 1)
        buttons.addWidget(self.preview_btn, 2, 0, 1, 2)

        top_buttons = QtWidgets.QHBoxLayout()
        top_buttons.addWidget(self.discover_btn)
        top_buttons.addWidget(self.refresh_btn)
        top_buttons.addStretch(1)

        left = QtWidgets.QVBoxLayout()
        left.addLayout(top_buttons)
        left.addWidget(self.table)

        right = QtWidgets.QVBoxLayout()
        right.addWidget(settings_box)
        right.addLayout(sync_row)
        right.addLayout(buttons)
        right.addStretch(1)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        left_widget = QtWidgets.QWidget(); left_widget.setLayout(left)
        right_widget = QtWidgets.QWidget(); right_widget.setLayout(right)
        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(splitter)
        self.setLayout(layout)

        self.discover_btn.clicked.connect(self.discover)
        self.refresh_btn.clicked.connect(self.refresh_status)
        self.read_btn.clicked.connect(self.read_settings_from_selected)
        self.revert_btn.clicked.connect(self.revert_settings_edits)
        self.apply_btn.clicked.connect(lambda: self.apply_settings(False))
        self.apply_start_btn.clicked.connect(lambda: self.apply_settings(True))
        self.start_btn.clicked.connect(self.start_recording)
        self.stop_btn.clicked.connect(self.stop_recording)
        self.preview_btn.clicked.connect(self.preview_stub)

        for widget in self.tracked_setting_widgets().values():
            self._connect_setting_dirty_signal(widget)
        self.update_setting_styles()

        self.status_timer = QtCore.QTimer(self)
        self.status_timer.setInterval(STATUS_INTERVAL_MS)
        self.status_timer.timeout.connect(self.refresh_status)
        self.status_timer.start()

        self.discover_timer = QtCore.QTimer(self)
        self.discover_timer.setInterval(DISCOVERY_INTERVAL_MS)
        self.discover_timer.timeout.connect(self.discover)
        self.discover_timer.start()

    # ---- settings dirty/clean model ----
    def tracked_setting_widgets(self) -> Dict[str, QtWidgets.QWidget]:
        return {
            "mode": self.mode,
            "camera.pixel_format": self.pixel_format,
            "camera.bayer_pattern": self.bayer_pattern,
            "output.kind": self.output_kind,
            "camera.width": self.width,
            "camera.height": self.height,
            "camera.offset_x": self.offset_x,
            "camera.offset_y": self.offset_y,
            "camera.exposure_us": self.exposure_us,
            "camera.fps": self.fps,
            "camera.gain_db": self.gain_db,
            "camera.hardware_trigger": self.hw_trigger,
        }

    def _connect_setting_dirty_signal(self, widget: QtWidgets.QWidget):
        if isinstance(widget, QtWidgets.QComboBox):
            widget.currentIndexChanged.connect(self._settings_edited)
        elif isinstance(widget, QtWidgets.QSpinBox):
            widget.valueChanged.connect(self._settings_edited)
        elif isinstance(widget, QtWidgets.QDoubleSpinBox):
            widget.valueChanged.connect(self._settings_edited)
        elif isinstance(widget, QtWidgets.QCheckBox):
            widget.toggled.connect(self._settings_edited)

    def _commit_setting_editors(self):
        """Force spin boxes to commit any typed-but-not-yet-applied text.

        Without this, clicking Apply while the cursor is still inside a spin box can
        occasionally read the old numeric value even though the text visually changed.
        Qt calls this interpretation on focus changes too, but we do it explicitly
        before building YAML for CBRNG.
        """
        for widget in self.tracked_setting_widgets().values():
            if isinstance(widget, (QtWidgets.QSpinBox, QtWidgets.QDoubleSpinBox)):
                try:
                    widget.interpretText()
                except Exception:
                    pass

    def _settings_edited(self, *args):
        if self._populating_settings:
            return
        self.update_setting_styles()

    def _combo_value(self, combo: QtWidgets.QComboBox) -> Any:
        return combo.currentData()

    def _effective_output_kind_value(self) -> Any:
        value = self.output_kind.currentData()
        if value == "auto":
            mode = self.mode.currentData()
            return "rolling_raw_binary" if mode in ("raw8mono_rolling", "raw8bayerGBRG_rolling") else None
        return value

    def current_setting_values(self) -> Dict[str, Any]:
        return {
            "mode": self.mode.currentData(),
            "camera.pixel_format": self.pixel_format.currentData(),
            "camera.bayer_pattern": self.bayer_pattern.currentData(),
            "output.kind": self._effective_output_kind_value(),
            "camera.width": self.width.value(),
            "camera.height": self.height.value(),
            "camera.offset_x": self.offset_x.value(),
            "camera.offset_y": self.offset_y.value(),
            "camera.exposure_us": self.exposure_us.value(),
            "camera.fps": self.fps.value(),
            "camera.gain_db": self.gain_db.value(),
            "camera.hardware_trigger": self.hw_trigger.isChecked(),
        }

    def _set_combo_to_value(self, combo: QtWidgets.QComboBox, value: Any, label_prefix: str = ""):
        if value is None:
            return
        for i in range(combo.count()):
            if combo.itemData(i) == value:
                combo.setCurrentIndex(i)
                return
        label = f"{label_prefix}{value}" if label_prefix else str(value)
        combo.addItem(label, value)
        combo.setCurrentIndex(combo.count() - 1)

    def _set_widget_value(self, widget: QtWidgets.QWidget, value: Any):
        if value is None:
            return
        if isinstance(widget, QtWidgets.QComboBox):
            self._set_combo_to_value(widget, value)
        elif isinstance(widget, QtWidgets.QSpinBox):
            widget.setValue(int(float(value)))
        elif isinstance(widget, QtWidgets.QDoubleSpinBox):
            widget.setValue(float(value))
        elif isinstance(widget, QtWidgets.QCheckBox):
            widget.setChecked(bool(value))

    def _style_for_state(self, state: str) -> str:
        # Pale backgrounds: amber unknown/mixed, green synced, red local edit.
        if state == "synced":
            return "background-color: #dff4df;"
        if state == "dirty":
            return "background-color: #ffd9d6;"
        if state == "mixed":
            return "background-color: #fff1bf;"
        return "background-color: #fff1bf;"

    def update_setting_styles(self):
        current = self.current_setting_values()
        widgets = self.tracked_setting_widgets()
        any_dirty = False
        any_unknown = False

        for key, widget in widgets.items():
            if self.settings_state == "mixed":
                state = "mixed"
            elif key not in self.baseline_settings:
                state = "unknown"
                any_unknown = True
            elif values_equal(current.get(key), self.baseline_settings.get(key)):
                state = "synced"
            else:
                state = "dirty"
                any_dirty = True
            widget.setStyleSheet(self._style_for_state(state))

        if self.settings_state == "mixed":
            text = "Settings: mixed/partial ⚠"
        elif any_dirty:
            text = "Settings: local edits not applied"
        elif any_unknown:
            text = "Settings: unknown, read from camera"
        else:
            text = "Settings: synced ✅"
        self.settings_sync_label.setText(text)

    def _baseline_from_current(self):
        self.baseline_settings = self.current_setting_values()
        self.settings_state = "synced"
        self.update_setting_styles()

    def _set_mixed_or_unknown(self, msg: str = ""):
        self.settings_state = "mixed"
        self.update_setting_styles()
        if msg:
            self.log(msg)

    def read_settings_from_selected(self):
        nodes = self.selected_or_warn()
        if not nodes:
            return
        if len(nodes) > 1:
            self.log(f"Multiple cameras selected; reading settings from {nodes[0]} only.")
        self.read_settings_from_node(nodes[0])

    def read_settings_from_node(self, full: str, after_apply: bool = False):
        fut = self.ros.get_status_async(full)
        fut.add_done_callback(lambda f, full=full, after_apply=after_apply: self._read_settings_done(full, f, after_apply))

    def _read_settings_done(self, full: str, fut, after_apply: bool = False):
        try:
            resp = fut.result()
        except Exception as e:
            self._set_mixed_or_unknown(f"read settings failed for {full}: {e}")
            return
        text = getattr(resp, "effective_settings_yaml", "") or getattr(resp, "requested_settings_yaml", "") or ""
        values = parse_settings_yaml(text)
        if not values:
            self._set_mixed_or_unknown(f"read settings from {full}: no effective settings YAML returned")
            return

        self._populating_settings = True
        try:
            # Pull the first camera's effective settings into the editable GUI fields.
            for key, widget in self.tracked_setting_widgets().items():
                if key in values:
                    self._set_widget_value(widget, values[key])
            # expected_hardware_fps is a helper checkbox: if it matches FPS, keep it checked.
            if "camera.expected_hardware_fps" in values and "camera.fps" in values:
                try:
                    self.expected_hw_fps.setChecked(values_equal(values["camera.expected_hardware_fps"], values["camera.fps"]))
                except Exception:
                    pass
        finally:
            self._populating_settings = False

        current = self.current_setting_values()
        # Baseline tracks the GUI-relevant settings after the read. If a key was not
        # present in effective_settings_yaml, use the current GUI value so the read
        # operation leaves the panel in an all-green understandable state.
        self.baseline_settings = {key: current.get(key) for key in self.tracked_setting_widgets().keys()}
        self.settings_state = "synced"
        self.update_setting_styles()
        suffix = " after apply" if after_apply else ""
        vals = self.current_setting_values()
        self.log(
            f"read settings{suffix} from {full}; GUI fields synced to camera "
            f"(width={vals.get('camera.width')}, height={vals.get('camera.height')}, "
            f"exposure_us={vals.get('camera.exposure_us')}, fps={vals.get('camera.fps')})"
        )

    def revert_settings_edits(self):
        if not self.baseline_settings:
            self.log("No camera settings baseline yet; use Read from selected first.")
            self.update_setting_styles()
            return
        self._populating_settings = True
        try:
            for key, widget in self.tracked_setting_widgets().items():
                if key in self.baseline_settings:
                    self._set_widget_value(widget, self.baseline_settings[key])
        finally:
            self._populating_settings = False
        self.settings_state = "synced"
        self.update_setting_styles()
        self.log("reverted local camera setting edits")

    def _begin_apply_batch(self, nodes: List[str]) -> int:
        self._apply_batch_counter += 1
        batch_id = self._apply_batch_counter
        self._pending_apply_batches[batch_id] = {
            "nodes": list(nodes),
            "remaining": len(nodes),
            "success": {},
            "messages": {},
        }
        return batch_id

    def _record_apply_result(self, batch_id: int, full: str, ok: bool, message: str):
        batch = self._pending_apply_batches.get(batch_id)
        if batch is None:
            return
        batch["success"][full] = ok
        batch["messages"][full] = message
        batch["remaining"] -= 1
        if batch["remaining"] > 0:
            return

        nodes = batch["nodes"]
        successes = batch["success"]
        n_ok = sum(1 for v in successes.values() if v)
        n_total = len(nodes)
        if n_ok == n_total:
            self.log(f"apply_settings succeeded on {n_ok}/{n_total} camera(s); reading back {nodes[0] if nodes else 'camera'} to verify effective settings")
            # Read back the first selected camera so normalization/defaults are reflected.
            if nodes:
                self.read_settings_from_node(nodes[0], after_apply=True)
            else:
                self._baseline_from_current()
        else:
            failed = [node for node, ok in successes.items() if not ok]
            self._set_mixed_or_unknown(f"apply_settings partial/failed: {n_ok}/{n_total} OK; failed: {', '.join(failed)}")
            QtWidgets.QMessageBox.warning(
                self,
                "Apply settings incomplete",
                f"Applied settings to {n_ok}/{n_total} camera(s).\n\nFailed:\n" + "\n".join(failed),
            )
        self._pending_apply_batches.pop(batch_id, None)

    def selected_or_warn(self) -> List[str]:
        nodes = self.table.selected_full_names()
        if not nodes:
            QtWidgets.QMessageBox.information(self, "No cameras selected", "Select one or more cameras first.")
        return nodes

    def discover(self):
        nodes = self.ros.discover_camera_nodes()
        self.table.set_nodes(nodes)
        for _, _, full in nodes:
            self.ros.ensure_event_subscriptions(full)
        self.refresh_status()

    def refresh_status(self):
        for full in self.table.all_full_names():
            fut = self.ros.get_status_async(full)
            fut.add_done_callback(lambda f, full=full: self._status_done(full, f))

    def _status_done(self, full: str, fut):
        try:
            resp = fut.result()
        except Exception as e:
            self.log(f"status failed for {full}: {e}")
            return
        self.table.update_status(full, resp)

    def build_settings_for_node(self, full: str, md: Optional[SessionMetadata]) -> str:
        self._commit_setting_editors()
        mode = self.mode.currentData()
        pixel_format = self.pixel_format.currentData()
        bayer_pattern = self.bayer_pattern.currentData()
        output_kind = self.output_kind.currentData()

        if output_kind == "auto":
            output_kind = "rolling_raw_binary" if mode in ("raw8mono_rolling", "raw8bayerGBRG_rolling") else None

        settings: Dict[str, Any] = {
            "camera.width": self.width.value(),
            "camera.height": self.height.value(),
            "camera.offset_x": self.offset_x.value(),
            "camera.offset_y": self.offset_y.value(),
            "camera.exposure_us": self.exposure_us.value(),
            "camera.fps": self.fps.value(),
            "camera.gain_db": self.gain_db.value(),
            "camera.hardware_trigger": self.hw_trigger.isChecked(),
            "mode": mode,
            "camera.pixel_format": pixel_format,
            "camera.bayer_pattern": bayer_pattern,
            "output.kind": output_kind,
        }
        if self.expected_hw_fps.isChecked():
            settings["camera.expected_hardware_fps"] = self.fps.value()

        if md is not None:
            md.write_session_yaml()
            session_dir = md.session_dir()
            node = safe_token(full.strip("/"), "cam")
            node_dir = session_dir / node
            node_dir.mkdir(parents=True, exist_ok=True)
            prefix = md.node_prefix(node)
            settings["output.dir"] = str(node_dir)
            settings["output.prefix"] = prefix
            settings["metadata_path"] = str(node_dir / f"{prefix}.metadata.yaml")
            # These are harmless extra requested-settings keys. CBRNG metadata will preserve them.
            settings["session.session_yaml"] = str(session_dir / "session.yaml")
            settings["session.experiment_id"] = md.experiment_id
            settings["session.animal_id"] = md.animal_id
            settings["session.timepoint"] = md.timepoint
            settings["session.group"] = md.group
            settings["session.operator"] = md.operator
            settings["session.trial_id"] = md.trial_id
            settings["session.speed_cm_s"] = md.speed_cm_s
            settings["session.condition"] = md.condition
            settings["session.notes"] = md.notes
            settings["session.circular_trigger_type"] = md.circular_trigger_type
            settings["session.pre_trigger_s"] = md.pre_trigger_s
            settings["session.post_trigger_s"] = md.post_trigger_s

        return flat_yaml(settings)

    def apply_settings(self, activate_after_apply: bool):
        nodes = self.selected_or_warn()
        if not nodes:
            return
        md = None
        if activate_after_apply:
            md = self.metadata_panel.ensure_confirmed(self, "Apply + start recording needs trial metadata.")
            if md is None:
                return
        else:
            # Applying settings can be metadata-free, but if confirmed metadata exists, use it for output paths.
            md = self.metadata_panel.current_metadata() if self.metadata_panel.confirmed else None

        self._commit_setting_editors()
        self.update_setting_styles()
        batch_id = self._begin_apply_batch(nodes)
        for full in nodes:
            settings_yaml = self.build_settings_for_node(full, md)
            vals = self.current_setting_values()
            self.log(
                f"{full} apply_settings request: "
                f"mode={vals.get('mode')!r}, pixel={vals.get('camera.pixel_format')!r}, "
                f"width={vals.get('camera.width')}, height={vals.get('camera.height')}, "
                f"offset=({vals.get('camera.offset_x')},{vals.get('camera.offset_y')}), "
                f"exposure_us={vals.get('camera.exposure_us')}, fps={vals.get('camera.fps')}, "
                f"hw_trigger={vals.get('camera.hardware_trigger')}"
            )
            fut = self.ros.apply_settings_async(
                full,
                settings_yaml=settings_yaml,
                merge_with_current=True,
                restart_if_active=False,
                activate_after_apply=activate_after_apply,
            )
            fut.add_done_callback(lambda f, full=full, batch_id=batch_id: self._apply_done(batch_id, full, f))

    def _apply_done(self, batch_id: int, full: str, fut):
        try:
            resp = fut.result()
            ok = bool(resp.success)
            message = str(resp.message)
        except Exception as e:
            ok = False
            message = str(e)
        self.log(f"{full} apply_settings: {'OK' if ok else 'FAIL'} - {message}")
        self._record_apply_result(batch_id, full, ok, message)
        self.refresh_status()

    def start_recording(self):
        nodes = self.selected_or_warn()
        if not nodes:
            return
        md = self.metadata_panel.ensure_confirmed(self, "Start recording needs trial metadata.")
        if md is None:
            return
        # Important: start_recording alone does not reconfigure CBRNG paths.
        # We apply metadata/output settings first, then start.
        batch_id = self._begin_apply_batch(nodes)
        for full in nodes:
            settings_yaml = self.build_settings_for_node(full, md)
            fut = self.ros.apply_settings_async(
                full,
                settings_yaml=settings_yaml,
                merge_with_current=True,
                restart_if_active=False,
                activate_after_apply=True,
            )
            fut.add_done_callback(lambda f, full=full, batch_id=batch_id: self._apply_done(batch_id, full, f))

    def stop_recording(self):
        nodes = self.selected_or_warn()
        if not nodes:
            return
        for full in nodes:
            fut = self.ros.trigger_async(full, start=False)
            fut.add_done_callback(lambda f, full=full: self._trigger_done(full, "stop", f))

    def _trigger_done(self, full: str, label: str, fut):
        try:
            resp = fut.result()
        except Exception as e:
            self.log(f"{label} failed for {full}: {e}")
            return
        self.log(f"{full} {label}: {'OK' if resp.success else 'FAIL'} - {resp.message}")
        self.refresh_status()

    def preview_stub(self):
        QtWidgets.QMessageBox.information(
            self,
            "Preview mode",
            "Preview is intentionally a stub for this pass. Next step: add a CBRNG preview service/workflow so this panel can temporarily take camera ownership.",
        )

    def log(self, msg: str):
        parent = self.window()
        if hasattr(parent, "append_log"):
            parent.append_log(msg)
        else:
            print(msg)



# --------------------------
# Startup / shutdown tab
# --------------------------
@dataclass
class StartupHost:
    label: str
    hostname: str
    ip: str
    user: str = getpass.getuser()
    workspace: str = "~/ros2_ws"
    ros_distro: str = "humble"
    default_camera_node: str = "cam1"
    default_triggerbox_node: str = "triggerbox_host"
    notes: str = ""

    @property
    def is_local(self) -> bool:
        return self.ip.lower() in ("local", "localhost", "127.0.0.1") or self.hostname.lower() in ("local", "localhost", socket.gethostname().lower())

    def ssh_target(self) -> str:
        return f"{self.user}@{self.ip}"


@dataclass
class CameraLaunchProfile:
    label: str
    params_file: str = "none"
    package: str = "cambuffer_recorder_ng"
    executable: str = "cambuffer_recorder_ng"


@dataclass
class GenericLaunchProfile:
    label: str
    package: str
    executable: str
    args: str = ""
    default_node: str = "triggerbox_host"


@dataclass
class StartupConfig:
    hosts: List[StartupHost]
    camera_profiles: List[CameraLaunchProfile]
    triggerbox_profiles: List[GenericLaunchProfile]


def startup_config_path() -> Path:
    # camera_control/camera_control.py -> repo/package root is parents[1]
    try:
        return Path(__file__).resolve().parents[1] / "startup_hosts.yaml"
    except Exception:
        return Path.cwd() / "startup_hosts.yaml"


def default_startup_config() -> StartupConfig:
    user = getpass.getuser()
    hostname = socket.gethostname()
    ws = "~/ros2_ws"
    hosts = [
        StartupHost(label="local", hostname=hostname, ip="local", user=user, workspace=ws, ros_distro="humble", default_camera_node="cam1", notes="This computer"),
        StartupHost(label="camdev", hostname="camdev", ip="10.0.0.21", user=user, workspace=ws, ros_distro="humble", default_camera_node="cam1"),
        StartupHost(label="ros2test", hostname="ros2test", ip="10.0.0.22", user=user, workspace=ws, ros_distro="humble", default_camera_node="cam2"),
    ]
    camera_profiles = [
        CameraLaunchProfile("none", "none"),
        CameraLaunchProfile("cam1 ximea mono8 rolling hwtrigger", "~/ros2_ws/src/cambuffer_recorder_ng/config/cam1_ximea_raw8mono_rolling_hwtrigger.yaml"),
        CameraLaunchProfile("ximea mono8 rolling hwtrigger", "~/ros2_ws/src/cambuffer_recorder_ng/config/ximea_raw8mono_rolling_hwtrigger.yaml"),
        CameraLaunchProfile("ximea mono8 rolling", "~/ros2_ws/src/cambuffer_recorder_ng/config/ximea_raw8mono_rolling.yaml"),
        CameraLaunchProfile("fake raw8 BayerGBRG rolling", "~/ros2_ws/src/cambuffer_recorder_ng/config/fake_raw8bayerGBRG_rolling.yaml"),
    ]
    triggerbox_profiles = [
        GenericLaunchProfile("triggerbox_host default", "triggerbox_host", "triggerbox_host", "", "triggerbox_host"),
    ]
    return StartupConfig(hosts=hosts, camera_profiles=camera_profiles, triggerbox_profiles=triggerbox_profiles)


def load_startup_config(path: Optional[Path] = None) -> StartupConfig:
    path = path or startup_config_path()
    if not path.exists():
        return default_startup_config()
    try:
        import yaml  # type: ignore
        raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        hosts = [StartupHost(**item) for item in raw.get("hosts", [])]
        camera_profiles = [CameraLaunchProfile(**item) for item in raw.get("camera_profiles", [])]
        triggerbox_profiles = [GenericLaunchProfile(**item) for item in raw.get("triggerbox_profiles", [])]
        cfg = default_startup_config()
        return StartupConfig(
            hosts=hosts or cfg.hosts,
            camera_profiles=camera_profiles or cfg.camera_profiles,
            triggerbox_profiles=triggerbox_profiles or cfg.triggerbox_profiles,
        )
    except Exception as e:
        print(f"Failed to load {path}: {e}; using defaults", file=sys.stderr)
        return default_startup_config()


class LaunchCameraDialog(QtWidgets.QDialog):
    def __init__(self, host: StartupHost, profiles: List[CameraLaunchProfile], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Launch Camera")
        self.host = host
        self.profiles = profiles

        self.node_name = QtWidgets.QLineEdit(host.default_camera_node or "cam1")
        self.profile_combo = QtWidgets.QComboBox()
        for prof in profiles:
            self.profile_combo.addItem(prof.label, prof)
        self.params_file = QtWidgets.QLineEdit(profiles[0].params_file if profiles else "none")
        self.package = QtWidgets.QLineEdit(profiles[0].package if profiles else "cambuffer_recorder_ng")
        self.executable = QtWidgets.QLineEdit(profiles[0].executable if profiles else "cambuffer_recorder_ng")
        self.extra_ros_args = QtWidgets.QLineEdit()
        self.extra_ros_args.setPlaceholderText("optional, e.g. -p camera.width:=2048")

        self.profile_combo.currentIndexChanged.connect(self._profile_changed)
        self._profile_changed(0)

        form = QtWidgets.QFormLayout()
        form.addRow("Host", QtWidgets.QLabel(f"{host.label} ({host.ip})"))
        form.addRow("Node name", self.node_name)
        form.addRow("Settings profile", self.profile_combo)
        form.addRow("Params file", self.params_file)
        form.addRow("Package", self.package)
        form.addRow("Executable", self.executable)
        form.addRow("Extra ROS args", self.extra_ros_args)

        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout = QtWidgets.QVBoxLayout()
        layout.addLayout(form)
        layout.addWidget(buttons)
        self.setLayout(layout)
        self.resize(650, 220)

    def _profile_changed(self, idx: int):
        prof = self.profile_combo.currentData()
        if not prof:
            return
        self.params_file.setText(prof.params_file)
        self.package.setText(prof.package)
        self.executable.setText(prof.executable)

    def command(self, host: StartupHost) -> Tuple[str, str]:
        node = safe_token(self.node_name.text().strip(), host.default_camera_node or "cam1")
        package = self.package.text().strip() or "cambuffer_recorder_ng"
        executable = self.executable.text().strip() or "cambuffer_recorder_ng"
        params = self.params_file.text().strip()
        extra = self.extra_ros_args.text().strip()
        args = [f"-r __node:={node}"]
        if params and params.lower() not in ("none", "no", "null", ""):
            args.append(f"--params-file {shlex.quote(params)}")
        if extra:
            args.append(extra)
        setup = f"source /opt/ros/{shlex.quote(host.ros_distro)}/setup.bash && source {shlex.quote(host.workspace)}/install/setup.bash"
        cmd = f"{setup} && ros2 run {shlex.quote(package)} {shlex.quote(executable)} --ros-args " + " ".join(args)
        return node, cmd


class LaunchTriggerboxDialog(QtWidgets.QDialog):
    def __init__(self, host: StartupHost, profiles: List[GenericLaunchProfile], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Launch Triggerbox")
        self.host = host
        self.profiles = profiles
        default_profile = profiles[0] if profiles else GenericLaunchProfile("triggerbox_host default", "triggerbox_host", "triggerbox_host")

        self.node_name = QtWidgets.QLineEdit(host.default_triggerbox_node or default_profile.default_node)
        self.profile_combo = QtWidgets.QComboBox()
        for prof in profiles:
            self.profile_combo.addItem(prof.label, prof)
        self.package = QtWidgets.QLineEdit(default_profile.package)
        self.executable = QtWidgets.QLineEdit(default_profile.executable)
        self.args = QtWidgets.QLineEdit(default_profile.args)
        self.args.setPlaceholderText("optional package args before --ros-args")
        self.extra_ros_args = QtWidgets.QLineEdit()
        self.extra_ros_args.setPlaceholderText("optional ROS args")
        self.profile_combo.currentIndexChanged.connect(self._profile_changed)

        form = QtWidgets.QFormLayout()
        form.addRow("Host", QtWidgets.QLabel(f"{host.label} ({host.ip})"))
        form.addRow("Node name", self.node_name)
        form.addRow("Profile", self.profile_combo)
        form.addRow("Package", self.package)
        form.addRow("Executable", self.executable)
        form.addRow("Package args", self.args)
        form.addRow("Extra ROS args", self.extra_ros_args)

        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout = QtWidgets.QVBoxLayout()
        layout.addLayout(form)
        layout.addWidget(buttons)
        self.setLayout(layout)
        self.resize(650, 220)

    def _profile_changed(self, idx: int):
        prof = self.profile_combo.currentData()
        if not prof:
            return
        self.package.setText(prof.package)
        self.executable.setText(prof.executable)
        self.args.setText(prof.args)
        if not self.node_name.text().strip():
            self.node_name.setText(prof.default_node)

    def command(self, host: StartupHost) -> Tuple[str, str]:
        node = safe_token(self.node_name.text().strip(), host.default_triggerbox_node or "triggerbox_host")
        package = self.package.text().strip() or "triggerbox_host"
        executable = self.executable.text().strip() or "triggerbox_host"
        pkg_args = self.args.text().strip()
        extra_ros_args = self.extra_ros_args.text().strip()
        ros_args = f"--ros-args -r __node:={node}"
        if extra_ros_args:
            ros_args += " " + extra_ros_args
        setup = f"source /opt/ros/{shlex.quote(host.ros_distro)}/setup.bash && source {shlex.quote(host.workspace)}/install/setup.bash"
        cmd = f"{setup} && ros2 run {shlex.quote(package)} {shlex.quote(executable)}"
        if pkg_args:
            cmd += " " + pkg_args
        cmd += " " + ros_args
        return node, cmd


class StartupShutdownPanel(QtWidgets.QWidget):
    def __init__(self, ros: CameraControlRos, log_callback: Callable[[str], None]):
        super().__init__()
        self.ros = ros
        self.log = log_callback
        self.config_path = startup_config_path()
        self.config = load_startup_config(self.config_path)
        self.processes: Dict[str, QtCore.QProcess] = {}
        self.node_host_map: Dict[str, str] = {}

        self.nodes_table = QtWidgets.QTableWidget(0, 4)
        self.nodes_table.setHorizontalHeaderLabels(["Node name", "Node type", "Host/IP", "Status"])
        self.nodes_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.nodes_table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        self.nodes_table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        self.nodes_table.horizontalHeader().setSectionResizeMode(3, QtWidgets.QHeaderView.Stretch)
        self.nodes_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.nodes_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)

        self.hosts_table = QtWidgets.QTableWidget(0, 6)
        self.hosts_table.setHorizontalHeaderLabels(["Label", "Hostname", "IP", "Ping", "SSH", "Notes"])
        self.hosts_table.horizontalHeader().setSectionResizeMode(5, QtWidgets.QHeaderView.Stretch)
        self.hosts_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.hosts_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.hosts_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)

        self.config_label = QtWidgets.QLabel(f"Startup config: {self.config_path}")
        self.config_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.reload_config_btn = QtWidgets.QPushButton("Reload config")
        self.refresh_nodes_btn = QtWidgets.QPushButton("Refresh nodes")
        self.ping_all_btn = QtWidgets.QPushButton("Ping all")
        self.ssh_test_btn = QtWidgets.QPushButton("SSH test selected")
        self.launch_camera_btn = QtWidgets.QPushButton("Launch Camera")
        self.shutdown_camera_btn = QtWidgets.QPushButton("Shutdown Camera")
        self.launch_triggerbox_btn = QtWidgets.QPushButton("Launch Triggerbox")
        self.shutdown_triggerbox_btn = QtWidgets.QPushButton("Shutdown Triggerbox")

        top_group = QtWidgets.QGroupBox("ROS2 nodes on domain")
        top_layout = QtWidgets.QVBoxLayout()
        top_layout.addWidget(self.nodes_table)
        top_group.setLayout(top_layout)

        mid_group = QtWidgets.QGroupBox("Computers")
        mid_layout = QtWidgets.QVBoxLayout()
        mid_layout.addWidget(self.config_label)
        mid_layout.addWidget(self.hosts_table)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.reload_config_btn)
        row.addWidget(self.ping_all_btn)
        row.addWidget(self.ssh_test_btn)
        row.addWidget(self.refresh_nodes_btn)
        row.addStretch(1)
        mid_layout.addLayout(row)
        mid_group.setLayout(mid_layout)

        button_group = QtWidgets.QGroupBox("Selected computer actions")
        button_layout = QtWidgets.QGridLayout()
        button_layout.addWidget(self.launch_camera_btn, 0, 0)
        button_layout.addWidget(self.shutdown_camera_btn, 0, 1)
        button_layout.addWidget(self.launch_triggerbox_btn, 1, 0)
        button_layout.addWidget(self.shutdown_triggerbox_btn, 1, 1)
        button_group.setLayout(button_layout)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(top_group, stretch=2)
        layout.addWidget(mid_group, stretch=2)
        layout.addWidget(button_group, stretch=0)
        self.setLayout(layout)

        self.reload_config_btn.clicked.connect(self.reload_config)
        self.refresh_nodes_btn.clicked.connect(self.refresh_nodes)
        self.ping_all_btn.clicked.connect(self.ping_all)
        self.ssh_test_btn.clicked.connect(self.ssh_test_selected)
        self.launch_camera_btn.clicked.connect(self.launch_camera)
        self.shutdown_camera_btn.clicked.connect(lambda: self.shutdown_node("camera"))
        self.launch_triggerbox_btn.clicked.connect(self.launch_triggerbox)
        self.shutdown_triggerbox_btn.clicked.connect(lambda: self.shutdown_node("triggerbox"))

        self.populate_hosts()
        self.refresh_nodes()
        self.refresh_timer = QtCore.QTimer(self)
        self.refresh_timer.setInterval(2000)
        self.refresh_timer.timeout.connect(self.refresh_nodes)
        self.refresh_timer.start()

    def reload_config(self):
        self.config = load_startup_config(self.config_path)
        self.populate_hosts()
        self.log(f"Startup config reloaded from {self.config_path}")

    def populate_hosts(self):
        self.hosts_table.setRowCount(0)
        for host in self.config.hosts:
            row = self.hosts_table.rowCount()
            self.hosts_table.insertRow(row)
            vals = [host.label, host.hostname, host.ip, "local" if host.is_local else "?", "local" if host.is_local else "?", host.notes]
            for col, val in enumerate(vals):
                self.hosts_table.setItem(row, col, QtWidgets.QTableWidgetItem(str(val)))
        if self.hosts_table.rowCount() > 0:
            self.hosts_table.selectRow(0)

    def selected_host(self) -> Optional[StartupHost]:
        rows = self.hosts_table.selectionModel().selectedRows()
        if not rows:
            QtWidgets.QMessageBox.information(self, "No host selected", "Select one computer first.")
            return None
        idx = rows[0].row()
        if idx < 0 or idx >= len(self.config.hosts):
            return None
        return self.config.hosts[idx]

    def set_host_status(self, host: StartupHost, column_name: str, text: str):
        col = {"ping": 3, "ssh": 4}.get(column_name)
        if col is None:
            return
        for row, h in enumerate(self.config.hosts):
            if h is host:
                item = self.hosts_table.item(row, col)
                if item:
                    item.setText(text)
                return

    def _node_type(self, full: str, service_types: Dict[str, List[str]]) -> str:
        if "cambuffer_recorder_ng/srv/GetStatus" in service_types.get(f"{full}/get_status", []):
            return "camera"
        low = full.lower()
        if "trigger" in low:
            return "triggerbox"
        if "camera_control" in low:
            return "gui"
        return "unknown"

    def refresh_nodes(self):
        service_types = {name: types for name, types in self.ros.get_service_names_and_types()}
        rows = []
        own_full = full_node_name(self.ros.get_name(), self.ros.get_namespace())
        for name, ns in self.ros.get_node_names_and_namespaces():
            full = full_node_name(name, ns)
            ntype = self._node_type(full, service_types)
            host = self.node_host_map.get(full, "local" if full == own_full else "unknown")
            status = "visible"
            if ntype == "camera" and f"{full}/get_status" in service_types:
                status = "CBRNG status service visible"
            rows.append((full, ntype, host, status))
        rows.sort(key=lambda r: (r[1], r[0]))
        self.nodes_table.setRowCount(0)
        for vals in rows:
            row = self.nodes_table.rowCount()
            self.nodes_table.insertRow(row)
            for col, val in enumerate(vals):
                self.nodes_table.setItem(row, col, QtWidgets.QTableWidgetItem(str(val)))

    def ping_all(self):
        for host in self.config.hosts:
            self.ping_host(host)

    def ping_host(self, host: StartupHost):
        if host.is_local:
            self.set_host_status(host, "ping", "ok")
            return
        proc = QtCore.QProcess(self)
        proc.setProgram("ping")
        proc.setArguments(["-c", "1", "-W", "1", host.ip])
        proc.finished.connect(lambda code, status, h=host, p=proc: self._ping_finished(h, p, code))
        proc.start()

    def _ping_finished(self, host: StartupHost, proc: QtCore.QProcess, code: int):
        ok = code == 0
        self.set_host_status(host, "ping", "ok" if ok else "fail")
        self.log(f"ping {host.label} ({host.ip}): {'ok' if ok else 'fail'}")
        proc.deleteLater()

    def ssh_test_selected(self):
        host = self.selected_host()
        if host is None:
            return
        if host.is_local:
            self.set_host_status(host, "ssh", "local")
            self.log("SSH test local: ok")
            return
        proc = QtCore.QProcess(self)
        proc.setProgram("ssh")
        proc.setArguments(["-o", "BatchMode=yes", "-o", "ConnectTimeout=3", host.ssh_target(), "hostname && echo SSH_OK"])
        proc.finished.connect(lambda code, status, h=host, p=proc: self._ssh_finished(h, p, code))
        proc.start()

    def _ssh_finished(self, host: StartupHost, proc: QtCore.QProcess, code: int):
        out = bytes(proc.readAllStandardOutput()).decode(errors="replace").strip()
        err = bytes(proc.readAllStandardError()).decode(errors="replace").strip()
        ok = code == 0 and "SSH_OK" in out
        self.set_host_status(host, "ssh", "ok" if ok else "fail")
        self.log(f"ssh test {host.label}: {'ok' if ok else 'fail'} {out} {err}".strip())
        proc.deleteLater()

    def _start_process(self, key: str, host: StartupHost, shell_cmd: str, long_running: bool = True):
        if key in self.processes and self.processes[key].state() != QtCore.QProcess.NotRunning:
            QtWidgets.QMessageBox.information(self, "Already running", f"A process for {key} is already running from this GUI.")
            return
        proc = QtCore.QProcess(self)
        if host.is_local:
            proc.setProgram("bash")
            proc.setArguments(["-lc", shell_cmd])
        else:
            remote = f"bash -lc {shlex.quote(shell_cmd)}"
            proc.setProgram("ssh")
            proc.setArguments([host.ssh_target(), remote])
        proc.readyReadStandardOutput.connect(lambda p=proc, k=key: self._read_proc_output(k, p, False))
        proc.readyReadStandardError.connect(lambda p=proc, k=key: self._read_proc_output(k, p, True))
        proc.finished.connect(lambda code, status, k=key, p=proc: self._process_finished(k, p, code, status))
        self.processes[key] = proc
        self.log(f"launch {key} on {host.label}: {shell_cmd}")
        proc.start()

    def _run_short_command(self, key: str, host: StartupHost, shell_cmd: str):
        proc = QtCore.QProcess(self)
        if host.is_local:
            proc.setProgram("bash")
            proc.setArguments(["-lc", shell_cmd])
        else:
            proc.setProgram("ssh")
            proc.setArguments([host.ssh_target(), f"bash -lc {shlex.quote(shell_cmd)}"])
        proc.readyReadStandardOutput.connect(lambda p=proc, k=key: self._read_proc_output(k, p, False))
        proc.readyReadStandardError.connect(lambda p=proc, k=key: self._read_proc_output(k, p, True))
        proc.finished.connect(lambda code, status, k=key, p=proc: self._short_finished(k, p, code))
        self.log(f"run {key} on {host.label}: {shell_cmd}")
        proc.start()

    def _read_proc_output(self, key: str, proc: QtCore.QProcess, is_err: bool):
        data = proc.readAllStandardError() if is_err else proc.readAllStandardOutput()
        text = bytes(data).decode(errors="replace").strip()
        if not text:
            return
        prefix = "stderr" if is_err else "stdout"
        for line in text.splitlines():
            self.log(f"{key} {prefix}: {line}")

    def _process_finished(self, key: str, proc: QtCore.QProcess, code: int, status):
        self.log(f"process {key} finished with code {code}")
        proc.deleteLater()

    def _short_finished(self, key: str, proc: QtCore.QProcess, code: int):
        self.log(f"command {key} finished with code {code}")
        proc.deleteLater()
        QtCore.QTimer.singleShot(500, self.refresh_nodes)

    def launch_camera(self):
        host = self.selected_host()
        if host is None:
            return
        dlg = LaunchCameraDialog(host, self.config.camera_profiles, self)
        if dlg.exec() != QtWidgets.QDialog.Accepted:
            return
        node, cmd = dlg.command(host)
        full = f"/{node}"
        self.node_host_map[full] = f"{host.label}/{host.ip}"
        self._start_process(f"camera:{host.label}:{node}", host, cmd, long_running=True)
        QtCore.QTimer.singleShot(1500, self.refresh_nodes)

    def launch_triggerbox(self):
        host = self.selected_host()
        if host is None:
            return
        dlg = LaunchTriggerboxDialog(host, self.config.triggerbox_profiles, self)
        if dlg.exec() != QtWidgets.QDialog.Accepted:
            return
        node, cmd = dlg.command(host)
        full = f"/{node}"
        self.node_host_map[full] = f"{host.label}/{host.ip}"
        self._start_process(f"triggerbox:{host.label}:{node}", host, cmd, long_running=True)
        QtCore.QTimer.singleShot(1500, self.refresh_nodes)

    def shutdown_node(self, node_kind: str):
        host = self.selected_host()
        if host is None:
            return
        default_node = host.default_camera_node if node_kind == "camera" else host.default_triggerbox_node
        node, ok = QtWidgets.QInputDialog.getText(self, f"Shutdown {node_kind}", "Node name", text=default_node)
        if not ok:
            return
        node = safe_token(node, default_node)
        key_prefix = f"{node_kind}:{host.label}:{node}"
        proc = self.processes.get(key_prefix)
        if proc is not None and proc.state() != QtCore.QProcess.NotRunning:
            self.log(f"terminating GUI-launched process {key_prefix}")
            proc.terminate()
        # Also ask ROS lifecycle nicely, then use a narrow pkill fallback for dev convenience.
        setup = f"source /opt/ros/{shlex.quote(host.ros_distro)}/setup.bash && source {shlex.quote(host.workspace)}/install/setup.bash"
        if node_kind == "camera":
            pattern = f"cambuffer_recorder_ng.*__node:={node}"
        else:
            pattern = f"triggerbox.*__node:={node}"
        cmd = (
            f"{setup} && "
            f"(ros2 lifecycle set /{shlex.quote(node)} shutdown || true); "
            f"sleep 0.5; "
            f"(pkill -f {shlex.quote(pattern)} || true)"
        )
        self._run_short_command(f"shutdown:{node_kind}:{host.label}:{node}", host, cmd)


# --------------------------
# Main window
# --------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, ros: CameraControlRos):
        super().__init__()
        self.ros = ros
        self.setWindowTitle("camera_control: cameras + metadata + startup/shutdown")
        # Let Qt choose a natural initial size; avoid unnecessary scrollbars.

        self.metadata_panel = MetadataPanel()
        self.metadata_summary = MetadataSummary(self.metadata_panel)
        self.camera_panel = CameraPanel(ros, self.metadata_panel)
        self.ros.set_event_callback(self.append_log)

        self.log_box = QtWidgets.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(800)
        self.log_box.setMaximumHeight(170)
        self.log_box.setPlaceholderText("Camera events, recording events, storage updates, and service results appear here.")

        self.clear_log_btn = QtWidgets.QPushButton("Clear")
        self.pause_log_chk = QtWidgets.QCheckBox("Pause")
        self.autoscroll_chk = QtWidgets.QCheckBox("Auto-scroll")
        self.autoscroll_chk.setChecked(True)
        self.clear_log_btn.clicked.connect(self.log_box.clear)

        self.tabs = QtWidgets.QTabWidget()

        tab_camera = QtWidgets.QWidget()
        cam_layout = QtWidgets.QVBoxLayout()
        cam_layout.setContentsMargins(6, 6, 6, 6)
        cam_layout.addWidget(self.metadata_summary)
        cam_layout.addWidget(self.camera_panel, stretch=1)

        log_header = QtWidgets.QHBoxLayout()
        log_header.addWidget(QtWidgets.QLabel("Event log"))
        log_header.addStretch(1)
        log_header.addWidget(self.pause_log_chk)
        log_header.addWidget(self.autoscroll_chk)
        log_header.addWidget(self.clear_log_btn)

        log_layout = QtWidgets.QVBoxLayout()
        log_layout.addLayout(log_header)
        log_layout.addWidget(self.log_box)
        log_group = QtWidgets.QGroupBox()
        log_group.setLayout(log_layout)
        cam_layout.addWidget(log_group)

        tab_camera.setLayout(cam_layout)
        self.tabs.addTab(tab_camera, "Cameras")

        tab_meta = QtWidgets.QWidget()
        meta_layout = QtWidgets.QVBoxLayout()
        meta_layout.setContentsMargins(12, 12, 12, 12)
        meta_layout.addWidget(self.metadata_panel)
        meta_layout.addStretch(1)
        tab_meta.setLayout(meta_layout)
        self.tabs.addTab(tab_meta, "Metadata")

        self.startup_panel = StartupShutdownPanel(self.ros, self.append_log)
        self.tabs.addTab(self.startup_panel, "Startup/Shutdown")

        tab_preview = QtWidgets.QWidget()
        preview_layout = QtWidgets.QVBoxLayout()
        preview_layout.addWidget(QtWidgets.QLabel(
            "Preview workflow stub. Later this can temporarily take camera ownership, "
            "request low-rate frames, and release cameras when closed."
        ))
        preview_layout.addStretch(1)
        tab_preview.setLayout(preview_layout)
        self.tabs.addTab(tab_preview, "Preview")

        self.setCentralWidget(self.tabs)

        # Give the camera table enough room without forcing skyscraper mode.
        self.resize(1250, 760)

        self.metadata_summary.edit_requested.connect(self.goto_metadata_tab)
        QtCore.QTimer.singleShot(100, self.camera_panel.discover)

    def goto_metadata_tab(self):
        self.tabs.setCurrentIndex(1)

    def append_log(self, msg: str):
        if hasattr(self, "pause_log_chk") and self.pause_log_chk.isChecked():
            return
        # Collapse multi-line YAML-ish event strings into compact one-line log entries.
        compact = " ".join(str(msg).split())
        ts = datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}] {compact}"
        self.log_box.appendPlainText(line)
        if self.autoscroll_chk.isChecked():
            bar = self.log_box.verticalScrollBar()
            bar.setValue(bar.maximum())
        self.statusBar().showMessage(compact, 5000)


def main():
    rclpy.init(args=None)
    ros = CameraControlRos()
    app = QtWidgets.QApplication(sys.argv)

    spin_timer = QtCore.QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(ros, timeout_sec=0.0))
    spin_timer.start(SPIN_INTERVAL_MS)

    win = MainWindow(ros)
    win.show()
    ret = app.exec()

    ros.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()
