# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""HiitController — adapter between the pure HiitRunner and the live GUI/ROS.

Imports PySide6 (for the tick QTimer) and the pure hiit modules, but NOT rclpy:
the ``ros`` object (a CameraControlRos) and the manual ``treadmill_panel`` are
injected. This keeps the controller importable on an MBP and lets the standalone
panel demo drive it with mocks.

Wiring:
  - runner.set_speed   -> treadmill_panel.set_speed()  (the choke point: reuses
                          the existing [0,100] clamp + spinbox/UI sync)
  - take/run/stop/release -> ros.treadmill_trigger_async(action)
  - mutual exclusion: while RUNNING/PAUSED, lock out manual control
    (_hiit_lock flag + set_manual_enabled(False)); revert on terminal states.
"""

from __future__ import annotations

import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Optional

from PySide6 import QtCore

from . import protocol as protocol_mod
from . import settings as hsettings
from .runlog import RunLog
from .runner import HiitProgress, HiitRunner, HiitState


class HiitController(QtCore.QObject):
    def __init__(
        self,
        ros: Any,
        treadmill_panel: Any,
        panel: Any,
        log_fn: Optional[Callable[[str], None]] = None,
        tick_ms: int = 100,
        clock: Optional[Callable[[], float]] = None,
        enable_run_log: bool = True,
        run_log_dir: Optional[Any] = None,
    ) -> None:
        super().__init__()
        self.ros = ros
        self.treadmill_panel = treadmill_panel
        self.panel = panel
        self._log_fn = log_fn
        self._clock = clock  # injectable for headless/deterministic tests
        self._mono = clock if clock is not None else time.monotonic
        self.enable_run_log = enable_run_log
        self.run_log_dir = run_log_dir
        self._runner: Optional[HiitRunner] = None
        self._runlog: Optional[RunLog] = None
        self._loaded_protocol = None  # last imported phased regimen
        self._profile_dialog = None
        self._telemetry_dialog = None

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(tick_ms)
        self._timer.timeout.connect(self._on_tick)

        # Forward treadmill status to the telemetry scope (no new subscription).
        sc = getattr(treadmill_panel, "status_changed", None)
        if sc is not None:
            try:
                sc.connect(self._on_status)
            except Exception:
                pass

    # -------- helpers --------
    def default_dir(self) -> Path:
        return protocol_mod.default_hiit_dir()

    def _log(self, msg: str) -> None:
        if self._log_fn is not None:
            self._log_fn(msg)

    def _on_tick(self) -> None:
        if self._runner is not None:
            self._runner.tick()

    # -------- panel -> controller intents --------
    def request_import(self, path: str) -> None:
        try:
            proto = protocol_mod.load_protocol(path)
        except Exception as exc:  # validation / IO / YAML errors
            self.panel.show_error(f"Failed to load regimen:\n{exc}")
            self._log(f"HIIT import FAILED ({path}): {exc}")
            return
        self._loaded_protocol = proto
        self.panel.show_protocol(
            proto.protocol_name, proto.date, len(proto.stages), proto.estimated_total_s
        )
        # Seed the manual Ramp Protocol spinboxes from the file, if present.
        fn = getattr(self.panel, "set_ramp_seeds", None)
        if fn is not None:
            fn(proto.seed_target, proto.seed_step, proto.seed_every)
        self._push_protocol_views(proto)
        self._log(
            f"HIIT regimen loaded: {proto.protocol_name} "
            f"({len(proto.stages)} stages, ~{proto.estimated_total_s:.0f}s)"
        )

    def request_start(self) -> None:
        """Run the imported phased regimen."""
        if self._loaded_protocol is None:
            return
        self._build_and_start(self._loaded_protocol)

    def request_run_ramp(self, target: int, step: int, every: float) -> None:
        """Build and run a stepwise manual ramp from the current belt speed."""
        try:
            proto = protocol_mod.build_ramp_protocol(
                target, step, every, start=self._current_initial_speed()
            )
        except Exception as exc:
            self.panel.show_error(f"Invalid ramp settings:\n{exc}")
            self._log(f"HIIT ramp invalid: {exc}")
            return
        self._build_and_start(proto)

    def _current_initial_speed(self) -> int:
        status = getattr(self.treadmill_panel, "latest_status", None)
        if status is not None:
            cmd = getattr(status, "commanded_speed_cm_s", -1)
            if isinstance(cmd, int) and cmd >= 0:
                return cmd
        return 0

    def _build_and_start(self, proto) -> None:
        self._runner = HiitRunner(
            proto,
            set_speed=self._sink_set_speed,
            take_control=lambda: self._fire("take_control"),
            run_belt=lambda: self._fire("run"),
            stop_belt=lambda: self._fire("stop"),
            release_control=lambda: self._fire("release_control"),
            on_state_change=self._on_state_change,
            on_progress=self._on_progress,
            on_stage_change=self._on_stage_change,
            tick_interval_s=self._timer.interval() / 1000.0,
            **({"clock": self._clock} if self._clock is not None else {}),
        )
        initial = self._current_initial_speed()
        self._push_protocol_views(proto)
        # Begin a run-log before starting: start() fires the first stage event.
        if self.enable_run_log:
            self._runlog = RunLog(proto.protocol_name, proto.source_path, proto.estimated_total_s)
            self._runlog.start(datetime.now(), self._mono())
        else:
            self._runlog = None
        if self._runner.start(initial_speed=initial):
            self._log(f"HIIT '{proto.protocol_name}' started (from {initial} cm/s)")

    def request_toggle_pause(self) -> None:
        if self._runner is None:
            return
        if self._runner.state == HiitState.RUNNING:
            if self._runner.pause():
                self._log("HIIT protocol paused (belt holds current speed)")
        elif self._runner.state == HiitState.PAUSED:
            if self._runner.resume():
                self._log("HIIT protocol resumed")

    def request_abort(self) -> None:
        if self._runner is not None and self._runner.abort():
            self._log("HIIT protocol aborted (belt stopped)")

    def request_reset(self) -> None:
        if self._runner is not None and self._runner.reset():
            self._log("HIIT protocol reset")
            self.panel.apply_progress(self._runner.current_progress())

    def request_create_regimen(self) -> None:
        """Open the interactive builder; optionally load the saved regimen."""
        from .builder import RegimenBuilderDialog  # local import keeps panel import light

        parent = self.panel.window() if hasattr(self.panel, "window") else None
        dlg = RegimenBuilderDialog(parent)
        if dlg.exec() and dlg.saved_path:
            self._log(f"HIIT regimen saved: {dlg.saved_path}")
            if dlg.load_after:
                self.request_import(dlg.saved_path)

    # -------- detachable graph windows --------
    def toggle_profile_graph(self, on: bool) -> None:
        from .graphs import ProfileGraphDialog

        if on:
            if self._profile_dialog is None:
                self._profile_dialog = ProfileGraphDialog(self._dialog_parent())
                self._profile_dialog.closed.connect(lambda: self.panel.set_profile_checked(False))
            if self._loaded_protocol is not None:
                self._profile_dialog.set_protocol(
                    protocol_mod.speed_profile_points(self._loaded_protocol),
                    self._loaded_protocol.estimated_total_s,
                )
            self._profile_dialog.show()
            self._profile_dialog.raise_()
        elif self._profile_dialog is not None:
            self._profile_dialog.hide()

    def toggle_telemetry_graph(self, on: bool) -> None:
        from .graphs import TelemetryGraphDialog

        if on:
            if self._telemetry_dialog is None:
                self._telemetry_dialog = TelemetryGraphDialog(parent=self._dialog_parent())
                self._telemetry_dialog.closed.connect(lambda: self.panel.set_telemetry_checked(False))
            self._telemetry_dialog.show()
            self._telemetry_dialog.raise_()
        elif self._telemetry_dialog is not None:
            self._telemetry_dialog.hide()

    def _dialog_parent(self):
        return self.panel.window() if hasattr(self.panel, "window") else None

    def _on_status(self, msg) -> None:
        if self._telemetry_dialog is not None and self._telemetry_dialog.isVisible():
            self._telemetry_dialog.add_sample(
                self._mono(),
                getattr(msg, "commanded_speed_cm_s", -1),
                getattr(msg, "reported_speed_cm_s", -1),
            )

    def _push_protocol_views(self, proto) -> None:
        fn = getattr(self.panel, "set_scrubber_stages", None)
        if fn is not None:
            fn([(s.duration, float(s.speed), s.label) for s in proto.stages])
        if self._profile_dialog is not None:
            self._profile_dialog.set_protocol(
                protocol_mod.speed_profile_points(proto), proto.estimated_total_s
            )

    # -------- runner sinks --------
    def _sink_set_speed(self, speed: int) -> None:
        # Route through the manual panel so the existing clamp + UI sync apply.
        self.treadmill_panel.set_speed(int(speed))

    def _fire(self, action: str) -> None:
        try:
            fut = self.ros.treadmill_trigger_async(action)
        except Exception as exc:
            self._log(f"HIIT treadmill {action}: FAIL - {exc}")
            return
        if fut is None:
            self._log(f"HIIT treadmill {action}: requested (no ROS future)")
            return
        fut.add_done_callback(lambda f, a=action: self._fire_done(a, f))

    def _fire_done(self, action: str, fut) -> None:
        try:
            resp = fut.result()
            ok = bool(getattr(resp, "success", False))
            msg = str(getattr(resp, "message", ""))
            self._log(f"HIIT treadmill {action}: {'OK' if ok else 'FAIL'} - {msg}")
        except Exception as exc:
            self._log(f"HIIT treadmill {action}: FAIL - {exc}")

    # -------- runner callbacks --------
    def _on_state_change(self, old: HiitState, new: HiitState) -> None:
        if new == HiitState.RUNNING and old != HiitState.PAUSED:
            self._engage_lock()
            self._timer.start()
        elif new in (HiitState.COMPLETE, HiitState.ABORTED):
            self._timer.stop()
            self._release_lock()
            self._finalize_run_log(new)
        self._log(f"HIIT state: {old.value} -> {new.value}")

    def _on_progress(self, progress: HiitProgress) -> None:
        self.panel.apply_progress(progress)
        if self._profile_dialog is not None and self._profile_dialog.isVisible():
            self._profile_dialog.set_cursor(progress.total_elapsed_s)

    def _on_stage_change(self, index: int, stage) -> None:
        if self._runlog is not None:
            self._runlog.stage_started(index, stage, datetime.now(), self._mono())

    def _finalize_run_log(self, state: HiitState) -> None:
        if self._runlog is None:
            return
        outcome = "complete" if state == HiitState.COMPLETE else "aborted"
        self._runlog.finish(outcome, datetime.now(), self._mono())
        target_dir = self.run_log_dir if self.run_log_dir is not None else hsettings.get_run_log_dir()
        try:
            path = self._runlog.write(target_dir)
            self._log(f"HIIT run-log written: {path}")
        except Exception as exc:  # never let logging break the run teardown
            self._log(f"HIIT run-log write FAILED: {exc}")
        finally:
            self._runlog = None

    # -------- mutual exclusion --------
    def _engage_lock(self) -> None:
        self.treadmill_panel._hiit_lock = True
        fn = getattr(self.treadmill_panel, "set_manual_enabled", None)
        if fn is not None:
            fn(False)

    def _release_lock(self) -> None:
        self.treadmill_panel._hiit_lock = False
        fn = getattr(self.treadmill_panel, "set_manual_enabled", None)
        if fn is not None:
            fn(True)
