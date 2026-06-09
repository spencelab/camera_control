# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""HiitPanel — the PySide6 widget for the Treadmill tab's HIIT trainer.

PySide6 only; NO rclpy. It is a "dumb" view: button clicks call the injected
controller's request_* methods, and the controller pushes updates back via
show_protocol / show_error / apply_progress. This keeps the widget runnable
standalone on an MBP (see ``python3 -m hiit.panel``) with a mock controller.
"""

from __future__ import annotations

from typing import Optional

from PySide6 import QtCore, QtWidgets

from .runner import HiitProgress, HiitState


def _fmt_mmss(seconds: float) -> str:
    seconds = max(0, int(round(seconds)))
    return f"{seconds // 60:d}:{seconds % 60:02d}"


# Status-chip styles for the regimen label (no external deps; plain stylesheet).
_REGIMEN_UNLOADED_STYLE = (
    "QLabel { color: #b00020; background: #fce8e6; border: 1px solid #d93025;"
    " border-radius: 4px; padding: 4px 8px; }"
)
_REGIMEN_LOADED_STYLE = (
    "QLabel { color: #137333; background: #e6f4ea; border: 1px solid #34a853;"
    " border-radius: 4px; padding: 4px 8px; font-weight: bold; }"
)
_REGIMEN_NONE_TEXT = "🛑 No regimen loaded"


class HiitPanel(QtWidgets.QGroupBox):
    """HIIT / phased trainer controls, mounted under the manual TreadmillPanel."""

    def __init__(self, controller=None, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__("HIIT / Phased Trainer", parent)
        self._controller = controller
        self._has_protocol = False

        # --- widgets ---
        self.import_btn = QtWidgets.QPushButton("⬆ Import Regimen…")
        self.regimen_label = QtWidgets.QLabel(_REGIMEN_NONE_TEXT)
        self.regimen_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.regimen_label.setStyleSheet(_REGIMEN_UNLOADED_STYLE)

        self.run_btn = QtWidgets.QPushButton("▶ Run Protocol")
        self.pause_btn = QtWidgets.QPushButton("⏸ Pause")
        self.abort_btn = QtWidgets.QPushButton("⏹ Abort")
        self.reset_btn = QtWidgets.QPushButton("Reset")

        self.phase_label = QtWidgets.QLabel("Phase: —")
        self.speed_label = QtWidgets.QLabel("Speed: cmd — / target — cm/s")
        self.stage_bar = QtWidgets.QProgressBar()
        self.stage_bar.setRange(0, 100)
        self.stage_bar.setFormat("stage %p%")
        self.overall_bar = QtWidgets.QProgressBar()
        self.overall_bar.setRange(0, 100)
        self.overall_bar.setFormat("overall %p%")
        self.time_label = QtWidgets.QLabel("Stage 0.0 / 0.0 s   |   Total 0:00 / 0:00")

        # Manual Ramp (the target/step/every seeds): stepwise climb, no file needed.
        self.ramp_target = QtWidgets.QSpinBox()
        self.ramp_target.setRange(0, 100)
        self.ramp_target.setValue(30)
        self.ramp_target.setSuffix(" cm/s")
        self.ramp_step = QtWidgets.QSpinBox()
        self.ramp_step.setRange(1, 50)
        self.ramp_step.setValue(5)
        self.ramp_step.setPrefix("+")
        self.ramp_step.setSuffix(" cm/s")
        self.ramp_every = QtWidgets.QSpinBox()
        self.ramp_every.setRange(1, 3600)
        self.ramp_every.setValue(60)
        self.ramp_every.setSuffix(" s")
        self.run_ramp_btn = QtWidgets.QPushButton("Run Ramp")

        # --- layout ---
        top = QtWidgets.QHBoxLayout()
        top.addWidget(self.import_btn)
        top.addWidget(self.regimen_label, stretch=1)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(self.run_btn)
        controls.addWidget(self.pause_btn)
        controls.addWidget(self.abort_btn)
        controls.addWidget(self.reset_btn)
        controls.addStretch(1)

        ramp_row = QtWidgets.QHBoxLayout()
        ramp_row.addWidget(QtWidgets.QLabel("Manual ramp →"))
        ramp_row.addWidget(self.ramp_target)
        ramp_row.addWidget(QtWidgets.QLabel("by"))
        ramp_row.addWidget(self.ramp_step)
        ramp_row.addWidget(QtWidgets.QLabel("every"))
        ramp_row.addWidget(self.ramp_every)
        ramp_row.addWidget(self.run_ramp_btn)
        ramp_row.addStretch(1)

        layout = QtWidgets.QVBoxLayout()
        layout.addLayout(top)
        layout.addLayout(controls)
        layout.addWidget(self.phase_label)
        layout.addWidget(self.speed_label)
        layout.addWidget(self.stage_bar)
        layout.addWidget(self.overall_bar)
        layout.addWidget(self.time_label)
        layout.addLayout(ramp_row)
        self.setLayout(layout)

        # --- signals ---
        self.import_btn.clicked.connect(self._on_import_clicked)
        self.run_btn.clicked.connect(lambda: self._request("request_start"))
        self.pause_btn.clicked.connect(lambda: self._request("request_toggle_pause"))
        self.abort_btn.clicked.connect(lambda: self._request("request_abort"))
        self.reset_btn.clicked.connect(lambda: self._request("request_reset"))
        self.run_ramp_btn.clicked.connect(self._on_run_ramp_clicked)

        self._set_buttons_for_state(HiitState.IDLE)

    # -------- wiring --------
    def set_controller(self, controller) -> None:
        self._controller = controller

    def _request(self, method: str) -> None:
        if self._controller is None:
            return
        fn = getattr(self._controller, method, None)
        if fn is not None:
            fn()

    def _on_import_clicked(self) -> None:
        start_dir = ""
        if self._controller is not None and hasattr(self._controller, "default_dir"):
            start_dir = str(self._controller.default_dir())
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Import HIIT regimen", start_dir, "YAML protocols (*.yaml *.yml);;All files (*)"
        )
        if path and self._controller is not None:
            self._controller.request_import(path)

    def _on_run_ramp_clicked(self) -> None:
        if self._controller is None:
            return
        self._controller.request_run_ramp(
            self.ramp_target.value(), self.ramp_step.value(), self.ramp_every.value()
        )

    def set_ramp_seeds(self, target, step, every) -> None:
        """Pre-fill the manual ramp spinboxes from an imported regimen's seeds."""
        if target is not None:
            self.ramp_target.setValue(max(0, min(100, int(target))))
        if step is not None:
            self.ramp_step.setValue(max(1, min(50, int(step))))
        if every is not None:
            self.ramp_every.setValue(max(1, min(3600, int(every))))

    # -------- controller -> view --------
    def show_protocol(self, name: str, date: str, stage_count: int, est_total_s: float) -> None:
        self._has_protocol = True
        suffix = f" | {date}" if date else ""
        self.regimen_label.setText(
            f"✅ {name}{suffix}  —  {stage_count} stages, ~{_fmt_mmss(est_total_s)}"
        )
        self.regimen_label.setStyleSheet(_REGIMEN_LOADED_STYLE)
        self.stage_bar.setValue(0)
        self.overall_bar.setValue(0)
        self.phase_label.setText("Phase: ready")
        self._set_buttons_for_state(HiitState.IDLE)

    def show_error(self, message: str) -> None:
        QtWidgets.QMessageBox.warning(self, "HIIT regimen", message)

    def apply_progress(self, p: HiitProgress) -> None:
        self.phase_label.setText(
            f"Phase: {p.stage_label or '—'}  ({p.phase}, stage {p.stage_index + 1}/{p.stage_count})"
            if p.stage_index >= 0 else "Phase: —"
        )
        self.speed_label.setText(
            f"Speed: cmd {p.commanded_speed} / target {p.target_speed} cm/s"
        )
        stage_pct = int(100 * p.stage_elapsed_s / p.stage_total_s) if p.stage_total_s > 0 else (100 if p.phase == "done" else 0)
        overall_pct = int(100 * p.total_elapsed_s / p.total_estimated_s) if p.total_estimated_s > 0 else 0
        self.stage_bar.setValue(max(0, min(100, stage_pct)))
        self.overall_bar.setValue(max(0, min(100, overall_pct)))
        self.time_label.setText(
            f"Stage {p.stage_elapsed_s:.1f} / {p.stage_total_s:.1f} s   |   "
            f"Total {_fmt_mmss(p.total_elapsed_s)} / {_fmt_mmss(p.total_estimated_s)}"
        )
        self._set_buttons_for_state(p.state)

    # -------- button state machine --------
    def _set_buttons_for_state(self, state: HiitState) -> None:
        running = state == HiitState.RUNNING
        paused = state == HiitState.PAUSED
        terminal = state in (HiitState.COMPLETE, HiitState.ABORTED)
        idle = state in (HiitState.IDLE, HiitState.LOADING)

        self.import_btn.setEnabled(not (running or paused))
        self.run_btn.setEnabled(idle and self._has_protocol)
        self.pause_btn.setEnabled(running or paused)
        self.pause_btn.setText("▶ Resume" if paused else "⏸ Pause")
        self.abort_btn.setEnabled(running or paused)
        self.reset_btn.setEnabled(terminal)
        # Manual ramp controls share the lockout: usable only when not running.
        for w in (self.ramp_target, self.ramp_step, self.ramp_every, self.run_ramp_btn):
            w.setEnabled(not (running or paused))


# ----------------------------------------------------------------------------
# Standalone MBP demo: real HiitController + HiitRunner driven by mocks.
# Run from camera_control/camera_control/:  python3 -m hiit.panel
# ----------------------------------------------------------------------------
def _demo() -> int:
    import sys

    from .controller import HiitController

    class _MockStatus:
        commanded_speed_cm_s = 0

    class _MockRos:
        """Stand-in for CameraControlRos: logs instead of calling ROS."""

        def treadmill_trigger_async(self, action):
            print(f"[ros] trigger {action}")
            return None

        def treadmill_set_speed_async(self, speed):
            print(f"[ros] set_speed {speed}")
            return None

    class _MockTreadmillPanel:
        """Stand-in for TreadmillPanel: records set_speed + lock state."""

        def __init__(self):
            self.latest_status = _MockStatus()
            self._hiit_lock = False

        def set_speed(self, v):
            print(f"[belt] {v} cm/s")
            self.latest_status.commanded_speed_cm_s = v

        def set_manual_enabled(self, enabled):
            print(f"[panel] manual controls {'enabled' if enabled else 'LOCKED'}")

    app = QtWidgets.QApplication(sys.argv)
    panel = HiitPanel()
    ros = _MockRos()
    tmill = _MockTreadmillPanel()
    controller = HiitController(ros, tmill, panel, log_fn=lambda m: print(f"[log] {m}"))
    panel.set_controller(controller)
    # No regimen is loaded by default: the panel opens in the red "no regimen"
    # state. Use "⬆ Import Regimen…" to load one (file dialog opens in the
    # configs/hiit_protocols dir).

    win = QtWidgets.QWidget()
    lay = QtWidgets.QVBoxLayout(win)
    lay.addWidget(panel)
    win.resize(560, 320)
    win.setWindowTitle("HiitPanel standalone demo")
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(_demo())
