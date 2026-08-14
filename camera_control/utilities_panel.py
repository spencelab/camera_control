"""Small hardware/rig maintenance utilities for camera_control.

This module is intentionally independent of recording and processing logic.
It reads the currently selected rig through a callback, derives where each
camera process lives, and performs diagnostics/maintenance without touching
camera-control state.
"""
from __future__ import annotations

import concurrent.futures
import re
import socket
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Optional

from PySide6 import QtCore, QtWidgets


SSH_USER = "spencelab"
CAMERA_SESSIONS_ROOT = "/home/spencelab/camera_sessions"


@dataclass(frozen=True)
class CameraHost:
    camera: str
    host: str
    local: bool


@dataclass(frozen=True)
class HostResult:
    label: str
    host: str
    ok: bool
    output: str
    returncode: int


def _short_hostname() -> str:
    return socket.gethostname().split(".", 1)[0] or "controller"


def camera_hosts_from_rig(rig: Any) -> list[CameraHost]:
    """Infer camera -> computer mapping from RigPreset process names.

    Existing rigs.yaml naming is sufficient:
      cam1            -> local camera process
      cam1@cam1       -> camera process on host cam1
      cam2@10.0.0.2   -> camera process on that address
    """
    nodes = [str(x).strip().lstrip("/") for x in getattr(rig, "camera_nodes", ()) if str(x).strip()]
    process_names = [str(getattr(p, "name", "")).strip() for p in getattr(rig, "processes", ())]
    out: list[CameraHost] = []
    for node in nodes:
        match = next((name for name in process_names if name == node or name.startswith(node + "@")), "")
        if "@" in match:
            _camera, host = match.split("@", 1)
            out.append(CameraHost(node, host.strip(), False))
        else:
            out.append(CameraHost(node, "local", True))
    return out


def unique_computers(camera_hosts: Iterable[CameraHost]) -> list[CameraHost]:
    out: list[CameraHost] = []
    seen: set[str] = set()
    for spec in camera_hosts:
        key = "local" if spec.local else spec.host.lower()
        if key in seen:
            continue
        seen.add(key)
        out.append(spec)
    return out


def _run_host_command(spec: CameraHost, script: str, timeout: float = 30.0) -> HostResult:
    if spec.local:
        argv = ["bash", "-lc", script]
        host_label = _short_hostname()
    else:
        argv = [
            "ssh", "-T",
            "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=5",
            "-o", "StrictHostKeyChecking=accept-new",
            f"{SSH_USER}@{spec.host}",
            script,
        ]
        host_label = spec.host
    try:
        cp = subprocess.run(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )
        return HostResult(spec.camera, host_label, cp.returncode == 0, (cp.stdout or "").strip(), cp.returncode)
    except subprocess.TimeoutExpired as exc:
        partial = ""
        if exc.stdout:
            partial = exc.stdout if isinstance(exc.stdout, str) else exc.stdout.decode(errors="replace")
        return HostResult(spec.camera, host_label, False, (partial + "\nTIMEOUT").strip(), 124)
    except Exception as exc:
        return HostResult(spec.camera, host_label, False, str(exc), 125)


def _run_parallel(specs: list[CameraHost], script: str, timeout: float = 30.0) -> list[HostResult]:
    if not specs:
        return []
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(specs)) as executor:
        futures = [executor.submit(_run_host_command, spec, script, timeout) for spec in specs]
        return [future.result() for future in futures]


USB_PROBE_SCRIPT = r'''
python3 - <<'REMOTE_PY'
from pathlib import Path

def read(path):
    try:
        return path.read_text(errors="replace").strip()
    except Exception:
        return ""

found = 0
for dev in sorted(Path("/sys/bus/usb/devices").glob("*")):
    if not (dev / "idVendor").is_file():
        continue
    vendor = read(dev / "idVendor").lower()
    manufacturer = read(dev / "manufacturer")
    product = read(dev / "product")
    text = f"{manufacturer} {product}".lower()
    if "ximea" not in text and vendor != "20f7":
        continue
    found += 1
    speed = read(dev / "speed") or "?"
    busnum = read(dev / "busnum") or "?"
    devnum = read(dev / "devnum") or "?"
    print(f"XIMEA\tpath={dev.name}\tbus={busnum}\tdev={devnum}\tspeed={speed}\tmanufacturer={manufacturer or '?'}\tproduct={product or '?'}")
if not found:
    print("NO_XIMEA_USB_DEVICE")
    raise SystemExit(3)
REMOTE_PY
'''.strip()


TRIM_SCRIPT = f'''
set -eo pipefail
ROOT={CAMERA_SESSIONS_ROOT!r}
MOUNT=$(findmnt -n -o TARGET -T "$ROOT" | head -n 1)
if [[ -z "$MOUNT" ]]; then
    echo "TRIM_FAILED reason=could_not_resolve_mount root=$ROOT"
    exit 92
fi
echo "TRIM_START mount=$MOUNT root=$ROOT"
sudo -n fstrim -v "$MOUNT"
echo "TRIM_OK mount=$MOUNT"
'''.strip()


class UtilityWorker(QtCore.QThread):
    completed = QtCore.Signal(str, bool, str)

    def __init__(self, name: str, fn: Callable[[], tuple[bool, str]], parent: Optional[QtCore.QObject] = None):
        super().__init__(parent)
        self.name = name
        self.fn = fn

    def run(self) -> None:
        try:
            ok, text = self.fn()
        except Exception as exc:
            ok, text = False, f"{type(exc).__name__}: {exc}"
        self.completed.emit(self.name, ok, text)


class UtilitiesPanel(QtWidgets.QWidget):
    """Rig diagnostics and low-risk maintenance tools."""

    log_line = QtCore.Signal(str)

    def __init__(self, rig_provider: Callable[[], Any], parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self._rig_provider = rig_provider
        self._workers: dict[str, UtilityWorker] = {}

        self.rig_label = QtWidgets.QLabel("Rig: ?")
        self.rig_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)

        self.clock_btn = QtWidgets.QPushButton("Check rig clocks")
        self.clock_status = QtWidgets.QLabel("Not checked")
        self.clock_output = self._output_box(235)
        clock_row = QtWidgets.QHBoxLayout()
        clock_row.addWidget(self.clock_btn)
        clock_row.addWidget(self.clock_status)
        clock_row.addStretch(1)
        clock_group = QtWidgets.QGroupBox("Clock synchronization")
        clock_layout = QtWidgets.QVBoxLayout(clock_group)
        clock_layout.addLayout(clock_row)
        clock_layout.addWidget(self.clock_output)

        self.usb_btn = QtWidgets.QPushButton("Check XIMEA USB speeds")
        self.usb_status = QtWidgets.QLabel("Not checked")
        self.usb_output = self._output_box(150)
        usb_row = QtWidgets.QHBoxLayout()
        usb_row.addWidget(self.usb_btn)
        usb_row.addWidget(self.usb_status)
        usb_row.addStretch(1)
        usb_group = QtWidgets.QGroupBox("Camera USB links")
        usb_layout = QtWidgets.QVBoxLayout(usb_group)
        usb_layout.addLayout(usb_row)
        usb_layout.addWidget(QtWidgets.QLabel("Each XIMEA should report at least 5000 Mb/s; 480 Mb/s is a bad USB2 link."))
        usb_layout.addWidget(self.usb_output)

        self.trim_btn = QtWidgets.QPushButton("Trim camera drives")
        self.trim_status = QtWidgets.QLabel("Not run")
        self.trim_output = self._output_box(120)
        trim_row = QtWidgets.QHBoxLayout()
        trim_row.addWidget(self.trim_btn)
        trim_row.addWidget(self.trim_status)
        trim_row.addStretch(1)
        trim_group = QtWidgets.QGroupBox("SSD maintenance")
        trim_layout = QtWidgets.QVBoxLayout(trim_group)
        trim_layout.addLayout(trim_row)
        trim_layout.addWidget(QtWidgets.QLabel(
            "Runs one fstrim per camera computer in parallel and waits for the results. "
            "It does not delete files. Requires a non-interactive sudo rule for fstrim."
        ))
        trim_layout.addWidget(self.trim_output)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addWidget(self.rig_label)
        layout.addWidget(clock_group)
        layout.addWidget(usb_group)
        layout.addWidget(trim_group)
        layout.addStretch(1)

        self.clock_btn.clicked.connect(self.check_clocks)
        self.usb_btn.clicked.connect(self.check_usb)
        self.trim_btn.clicked.connect(self.trim_drives)
        self._refresh_rig_label()

    @staticmethod
    def _output_box(min_height: int) -> QtWidgets.QPlainTextEdit:
        box = QtWidgets.QPlainTextEdit()
        box.setReadOnly(True)
        box.setMinimumHeight(min_height)
        box.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        font = box.font()
        font.setFamily("monospace")
        box.setFont(font)
        return box

    def _active_rig(self) -> Any:
        rig = self._rig_provider()
        self.rig_label.setText(f"Rig: {getattr(rig, 'label', getattr(rig, 'key', '?'))}")
        return rig

    def _refresh_rig_label(self) -> None:
        try:
            self._active_rig()
        except Exception:
            self.rig_label.setText("Rig: unavailable")

    def _camera_hosts(self) -> list[CameraHost]:
        return camera_hosts_from_rig(self._active_rig())

    def _start(self, name: str, fn: Callable[[], tuple[bool, str]]) -> None:
        if name in self._workers and self._workers[name].isRunning():
            return
        worker = UtilityWorker(name, fn, self)
        worker.completed.connect(self._operation_finished)
        worker.finished.connect(lambda name=name: self._workers.pop(name, None))
        self._workers[name] = worker
        self._set_busy(name, True)
        worker.start()

    def _set_busy(self, name: str, busy: bool) -> None:
        mapping = {
            "clocks": (self.clock_btn, self.clock_status),
            "usb": (self.usb_btn, self.usb_status),
            "trim": (self.trim_btn, self.trim_status),
        }
        button, status = mapping[name]
        button.setEnabled(not busy)
        if busy:
            status.setText("Running…")

    def _operation_finished(self, name: str, ok: bool, text: str) -> None:
        self._set_busy(name, False)
        mapping = {
            "clocks": (self.clock_status, self.clock_output),
            "usb": (self.usb_status, self.usb_output),
            "trim": (self.trim_status, self.trim_output),
        }
        status, output = mapping[name]
        status.setText("OK" if ok else "CHECK")
        output.setPlainText(text)
        self.log_line.emit(f"Utilities: {name}: {'OK' if ok else 'CHECK'}")
        if not ok:
            QtWidgets.QMessageBox.warning(self, "Utilities check", f"{name} needs attention.\n\n{text[-1800:]}")

    def check_clocks(self) -> None:
        hosts = self._camera_hosts()
        remote = [host for host in hosts if not host.local]
        self.clock_output.clear()
        if not remote:
            text = "Single-computer rig: no cross-computer clock synchronization check is needed."
            self.clock_output.setPlainText(text)
            self.clock_status.setText("N/A")
            self.log_line.emit("Utilities: clocks: single-computer rig; skipped")
            return

        def work() -> tuple[bool, str]:
            repo_root = Path(__file__).resolve().parents[1]
            tool = repo_root / "tools" / "rig-clocks"
            if not tool.exists():
                return False, f"rig-clocks not found: {tool}"
            argv = [sys.executable, str(tool), "--no-color", "--host", f"{_short_hostname()}=local"]
            for spec in remote:
                argv.extend(["--host", f"{spec.camera}={spec.host}"])
            cp = subprocess.run(
                argv,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=20,
                check=False,
            )
            text = (cp.stdout or "").strip()
            return cp.returncode == 0, text or f"rig-clocks exited {cp.returncode} with no output"

        self._start("clocks", work)

    def check_usb(self) -> None:
        specs = unique_computers(self._camera_hosts())
        self.usb_output.clear()

        def work() -> tuple[bool, str]:
            results = _run_parallel(specs, USB_PROBE_SCRIPT, timeout=15)
            lines: list[str] = []
            all_ok = True
            for result in results:
                host_ok = result.ok
                speed_values = [float(x) for x in re.findall(r"\bspeed=([0-9.]+)", result.output)]
                if not speed_values or any(speed < 5000.0 for speed in speed_values):
                    host_ok = False
                all_ok = all_ok and host_ok
                state = "OK" if host_ok else "BAD"
                lines.append(f"===== {result.label} @ {result.host}: {state} =====")
                lines.append(result.output or f"command exited {result.returncode} with no output")
                if speed_values:
                    lines.append("USB speeds: " + ", ".join(f"{speed:g} Mb/s" for speed in speed_values))
                lines.append("")
            return all_ok, "\n".join(lines).rstrip()

        self._start("usb", work)

    def trim_drives(self) -> None:
        specs = unique_computers(self._camera_hosts())
        if not specs:
            self.trim_status.setText("N/A")
            self.trim_output.setPlainText("No camera computers are defined by the active rig.")
            return
        answer = QtWidgets.QMessageBox.question(
            self,
            "Trim camera drives",
            "Run fstrim once on the filesystem containing camera_sessions on every camera computer?\n\n"
            "The trims run in parallel. This does not delete files.",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        self.trim_output.clear()

        def work() -> tuple[bool, str]:
            results = _run_parallel(specs, TRIM_SCRIPT, timeout=300)
            lines: list[str] = []
            all_ok = True
            for result in results:
                all_ok = all_ok and result.ok
                state = "OK" if result.ok else "FAILED"
                lines.append(f"===== {result.label} @ {result.host}: {state} =====")
                lines.append(result.output or f"command exited {result.returncode} with no output")
                lines.append("")
            return all_ok, "\n".join(lines).rstrip()

        self._start("trim", work)
