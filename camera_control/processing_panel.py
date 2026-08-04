"""Processing tab for camera_control.

Minimal MERB-pilot tools:
  - scan local ~/camera_sessions for sessions
  - ask cam1-cam5 to extract first-frame PNG thumbnails from matching .cbrraw files
  - copy thumbnails back to the local tmill session folder

This module intentionally avoids rclpy. It is just PySide6 + subprocess so the
main camera cockpit does not grow another tentacle.
"""

from __future__ import annotations

import os
import shlex
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional

from PySide6 import QtCore, QtWidgets


@dataclass(frozen=True)
class ThumbnailJob:
    session: str
    cam: str
    host: str
    local_thumb: Path


class ThumbnailWorker(QtCore.QObject):
    log = QtCore.Signal(str)
    progress = QtCore.Signal(int, int)
    finished = QtCore.Signal(int, int)

    def __init__(
        self,
        *,
        base_dir: Path,
        sessions: List[str],
        cameras: List[str],
        r_gain: float,
        g_gain: float,
        b_gain: float,
        gamma: float,
        parent: Optional[QtCore.QObject] = None,
    ):
        super().__init__(parent)
        self.base_dir = base_dir.expanduser()
        self.sessions = sessions
        self.cameras = cameras
        self.r_gain = r_gain
        self.g_gain = g_gain
        self.b_gain = b_gain
        self.gamma = gamma
        self._cancelled = False

    @QtCore.Slot()
    def cancel(self) -> None:
        self._cancelled = True

    @QtCore.Slot()
    def run(self) -> None:
        jobs = [
            ThumbnailJob(
                session=session,
                cam=cam,
                host=cam,
                local_thumb=self.base_dir / session / "thumbnails" / f"{cam}_first.png",
            )
            for session in self.sessions
            for cam in self.cameras
        ]
        total = len(jobs)
        done = 0
        ok = 0
        self.log.emit(f"Processing: thumbnail scan starting for {len(self.sessions)} sessions x {len(self.cameras)} cameras")

        for job in jobs:
            if self._cancelled:
                self.log.emit("Processing: cancelled")
                break
            if self._process_one(job):
                ok += 1
            done += 1
            self.progress.emit(done, total)

        self.log.emit(f"Processing: thumbnails complete: {ok}/{done} copied")
        self.finished.emit(ok, done)

    def _run_local(self, argv: List[str], *, timeout_s: int = 60) -> subprocess.CompletedProcess:
        return subprocess.run(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout_s,
            check=False,
        )

    def _remote_script(self, session: str, cam: str) -> str:
        # The remote camera node records into its own local ~/camera_sessions tree.
        remote_dir = f"~/camera_sessions/{shlex.quote(session)}/{shlex.quote(cam)}"
        r = shlex.quote(f"{self.r_gain:.6g}")
        g = shlex.quote(f"{self.g_gain:.6g}")
        b = shlex.quote(f"{self.b_gain:.6g}")
        gamma = shlex.quote(f"{self.gamma:.6g}")

        return f"""
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
DIR={remote_dir}
if [[ ! -d "$DIR" ]]; then
  echo "NO_SESSION_DIR $DIR"
  exit 20
fi
RAW=$(find "$DIR" -maxdepth 1 -type f -name '*_0000.cbrraw' | sort | head -n 1)
if [[ -z "${{RAW}}" ]]; then
  echo "NO_CBRRAW $DIR"
  exit 21
fi
META="${{RAW%_0000.cbrraw}}.metadata.yaml"
PNG="$DIR/{cam}_first.png"
if [[ ! -f "$META" ]]; then
  META=none
fi
ros2 run cambuffer_recorder_ng raw_rolling_to_mp4 "$RAW" "$PNG" 1 0 "$META" {r} {g} {b} {gamma}
echo "$PNG"
""".strip()

    def _process_one(self, job: ThumbnailJob) -> bool:
        job.local_thumb.parent.mkdir(parents=True, exist_ok=True)
        remote_script = self._remote_script(job.session, job.cam)
        self.log.emit(f"Processing: {job.session}/{job.cam}: extracting first frame on {job.host}")

        try:
            proc = self._run_local(
                ["ssh", "-T", f"spencelab@{job.host}", remote_script],
                timeout_s=120,
            )
        except subprocess.TimeoutExpired:
            self.log.emit(f"Processing: {job.session}/{job.cam}: TIMEOUT during remote extraction")
            return False

        out = (proc.stdout or "").strip()
        if proc.returncode != 0:
            # Missing sessions are expected when tmill has session.yaml but that camera did not record.
            first = out.splitlines()[0] if out else "no output"
            self.log.emit(f"Processing: {job.session}/{job.cam}: skip/fail rc={proc.returncode}: {first}")
            return False

        remote_png = out.splitlines()[-1].strip() if out else f"~/camera_sessions/{job.session}/{job.cam}/{job.cam}_first.png"
        self.log.emit(f"Processing: {job.session}/{job.cam}: copying thumbnail to {job.local_thumb}")
        try:
            scp = self._run_local(
                ["scp", "-q", f"spencelab@{job.host}:{remote_png}", str(job.local_thumb)],
                timeout_s=60,
            )
        except subprocess.TimeoutExpired:
            self.log.emit(f"Processing: {job.session}/{job.cam}: TIMEOUT during scp")
            return False

        if scp.returncode != 0:
            first = (scp.stdout or "").strip().splitlines()[0] if (scp.stdout or "").strip() else "scp failed"
            self.log.emit(f"Processing: {job.session}/{job.cam}: scp failed rc={scp.returncode}: {first}")
            return False

        return True


class ProcessingPanel(QtWidgets.QWidget):
    """Small processing tab for pilot-day remote thumbnail generation."""

    log_line = QtCore.Signal(str)

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self._thread: Optional[QtCore.QThread] = None
        self._worker: Optional[ThumbnailWorker] = None

        self.base_dir_edit = QtWidgets.QLineEdit(str(Path.home() / "camera_sessions"))
        self.cameras_edit = QtWidgets.QLineEdit("cam1 cam2 cam3 cam4 cam5")

        self.r_spin = self._gain_spin(1.0)
        self.g_spin = self._gain_spin(1.0)
        self.b_spin = self._gain_spin(1.0)
        self.gamma_spin = self._gain_spin(1.0)
        self.gamma_spin.setMinimum(0.05)
        self.gamma_spin.setMaximum(5.0)

        self.refresh_btn = QtWidgets.QPushButton("Refresh sessions")
        self.create_btn = QtWidgets.QPushButton("Create and copy thumbnails")
        self.cancel_btn = QtWidgets.QPushButton("Cancel")
        self.cancel_btn.setEnabled(False)

        self.session_list = QtWidgets.QListWidget()
        self.session_list.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.session_list.setMinimumHeight(220)

        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(0)

        self.status_label = QtWidgets.QLabel("Ready.")
        self.status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)

        self._build_layout()

        self.refresh_btn.clicked.connect(self.refresh_sessions)
        self.create_btn.clicked.connect(self.create_thumbnails)
        self.cancel_btn.clicked.connect(self.cancel)
        QtCore.QTimer.singleShot(0, self.refresh_sessions)

    def _gain_spin(self, value: float) -> QtWidgets.QDoubleSpinBox:
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(0.0, 10.0)
        spin.setDecimals(3)
        spin.setSingleStep(0.05)
        spin.setValue(value)
        return spin

    def _build_layout(self) -> None:
        top = QtWidgets.QFormLayout()
        top.addRow("Local sessions root", self.base_dir_edit)
        top.addRow("Camera hosts/nodes", self.cameras_edit)

        wb = QtWidgets.QHBoxLayout()
        wb.addWidget(QtWidgets.QLabel("R"))
        wb.addWidget(self.r_spin)
        wb.addWidget(QtWidgets.QLabel("G"))
        wb.addWidget(self.g_spin)
        wb.addWidget(QtWidgets.QLabel("B"))
        wb.addWidget(self.b_spin)
        wb.addWidget(QtWidgets.QLabel("Gamma"))
        wb.addWidget(self.gamma_spin)
        wb.addStretch(1)
        top.addRow("White balance / gamma", wb)

        buttons = QtWidgets.QHBoxLayout()
        buttons.addWidget(self.refresh_btn)
        buttons.addWidget(self.create_btn)
        buttons.addWidget(self.cancel_btn)
        buttons.addStretch(1)

        hint = QtWidgets.QLabel(
            "Scans local session.yaml folders, asks each matching camera host to create "
            "camN_first.png from its first *_0000.cbrraw, then copies thumbnails into "
            "<session>/thumbnails/ on tmill."
        )
        hint.setWordWrap(True)

        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addLayout(top)
        layout.addLayout(buttons)
        layout.addWidget(hint)
        layout.addWidget(QtWidgets.QLabel("Sessions"))
        layout.addWidget(self.session_list, stretch=1)
        layout.addWidget(self.progress)
        layout.addWidget(self.status_label)
        self.setLayout(layout)

    def _base_dir(self) -> Path:
        return Path(self.base_dir_edit.text().strip() or "~/camera_sessions").expanduser()

    def _camera_names(self) -> List[str]:
        names = [x.strip() for x in self.cameras_edit.text().replace(",", " ").split()]
        return [x for x in names if x]

    @QtCore.Slot()
    def refresh_sessions(self) -> None:
        base = self._base_dir()
        self.session_list.clear()
        sessions = []
        if base.is_dir():
            for child in sorted(base.iterdir(), key=lambda p: p.name):
                if child.is_dir() and (child / "session.yaml").exists():
                    sessions.append(child.name)
        for name in sessions:
            item = QtWidgets.QListWidgetItem(name)
            item.setSelected(True)
            self.session_list.addItem(item)
        self.status_label.setText(f"Found {len(sessions)} local sessions under {base}")

    def _selected_sessions(self) -> List[str]:
        return [item.text() for item in self.session_list.selectedItems()]

    @QtCore.Slot()
    def create_thumbnails(self) -> None:
        if self._thread is not None:
            return
        sessions = self._selected_sessions()
        if not sessions:
            self.status_label.setText("No sessions selected.")
            return
        cameras = self._camera_names()
        if not cameras:
            self.status_label.setText("No camera hosts/nodes configured.")
            return

        total = len(sessions) * len(cameras)
        self.progress.setRange(0, total)
        self.progress.setValue(0)
        self.status_label.setText(f"Creating thumbnails for {len(sessions)} sessions x {len(cameras)} cameras...")
        self.create_btn.setEnabled(False)
        self.cancel_btn.setEnabled(True)

        self._thread = QtCore.QThread(self)
        self._worker = ThumbnailWorker(
            base_dir=self._base_dir(),
            sessions=sessions,
            cameras=cameras,
            r_gain=float(self.r_spin.value()),
            g_gain=float(self.g_spin.value()),
            b_gain=float(self.b_spin.value()),
            gamma=float(self.gamma_spin.value()),
        )
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.log.connect(self.log_line.emit)
        self._worker.progress.connect(self._on_progress)
        self._worker.finished.connect(self._on_finished)
        self._worker.finished.connect(self._thread.quit)
        self._thread.finished.connect(self._thread.deleteLater)
        self._thread.start()

    @QtCore.Slot()
    def cancel(self) -> None:
        if self._worker is not None:
            self._worker.cancel()
            self.cancel_btn.setEnabled(False)
            self.status_label.setText("Cancelling after current remote command...")

    @QtCore.Slot(int, int)
    def _on_progress(self, done: int, total: int) -> None:
        self.progress.setRange(0, max(1, total))
        self.progress.setValue(done)
        self.status_label.setText(f"Thumbnail jobs: {done}/{total}")

    @QtCore.Slot(int, int)
    def _on_finished(self, ok: int, done: int) -> None:
        self.status_label.setText(f"Done: copied {ok}/{done} thumbnails.")
        self.create_btn.setEnabled(True)
        self.cancel_btn.setEnabled(False)
        self._worker = None
        self._thread = None
