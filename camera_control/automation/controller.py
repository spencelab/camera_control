# Author: Andrew England (andrewengland19)
"""AutoRunController — the one-click Autostart & Automated Protocol state machine.

Chains the *existing* manual controls (it only calls their public methods) into a
single trial loop:

    READY
      -> reset HIIT protocol
      -> register + confirm metadata (rat ID / condition)
      -> start camera recording, await ROS confirmation (``status.recording``)
      -> run the HIIT protocol
      -> when the protocol completes AND belt telemetry reads 0 cm/s, stop recording
      -> COMPLETE (returns to READY)

Any timeout or error routes to a safe teardown (gentle-stop the belt if running,
stop recording if active) and an ERROR state, so the rig is never left recording
or with the belt moving.

No rclpy / PySide6 widget construction here beyond ``QObject`` + ``QTimer``: the
ROS node and the manual panels are injected, so this is unit-testable headlessly.
Because rclpy is spun from the Qt event loop in this app, service futures and
timers all fire on the GUI thread — no locking is required.
"""

from __future__ import annotations

import time
from typing import Any, Callable, List, Optional

from PySide6 import QtCore


# -- state keys (also used by the panel to pick a banner colour) --------------
READY = "ready"
RESETTING = "resetting"
METADATA = "metadata"
STARTING_REC = "starting_rec"
PROTOCOL = "protocol"
STOPPING_REC = "stopping_rec"
COMPLETE = "complete"
ERROR = "error"

# Human-facing labels for the big status banner.
STATE_LABELS = {
    READY: "READY",
    RESETTING: "RESETTING PROTOCOL",
    METADATA: "REGISTERING METADATA",
    STARTING_REC: "STARTING RECORDING…",
    PROTOCOL: "PROTOCOL ACTIVE",
    STOPPING_REC: "STOPPING RECORDING…",
    COMPLETE: "COMPLETE",
    ERROR: "ERROR",
}

_ACTIVE_STATES = {RESETTING, METADATA, STARTING_REC, PROTOCOL, STOPPING_REC}


class AutoRunController(QtCore.QObject):
    # (state_key, human_label)
    state_changed = QtCore.Signal(str, str)
    # free-text progress / log line for the panel + event log
    message = QtCore.Signal(str)

    def __init__(
        self,
        ros: Any,
        camera_panel: Any,
        metadata_panel: Any,
        treadmill_panel: Any,
        hiit_controller: Any,
        speaker: Any = None,
        log_fn: Optional[Callable[[str], None]] = None,
        clock: Optional[Callable[[], float]] = None,
        poll_ms: int = 300,
        rec_confirm_timeout_s: float = 8.0,
        rec_stop_timeout_s: float = 8.0,
        protocol_margin_s: float = 15.0,
        parent: Optional[QtCore.QObject] = None,
    ) -> None:
        super().__init__(parent)
        self.ros = ros
        self.camera_panel = camera_panel
        self.metadata_panel = metadata_panel
        self.treadmill_panel = treadmill_panel
        self.hiit_controller = hiit_controller
        self.speaker = speaker
        self._log_fn = log_fn
        self._clock = clock if clock is not None else time.monotonic

        self.rec_confirm_timeout_s = rec_confirm_timeout_s
        self.rec_stop_timeout_s = rec_stop_timeout_s
        self.protocol_margin_s = protocol_margin_s

        self.state: str = READY
        self._nodes: List[str] = []
        self._deadline: float = 0.0
        self._recording_confirmed: set = set()
        self._protocol_terminal: bool = False
        self._belt_zero: bool = False

        self._poll = QtCore.QTimer(self)
        self._poll.setInterval(poll_ms)
        self._poll.timeout.connect(self._on_poll)

        # Observe protocol completion + belt telemetry (never re-subscribes ROS).
        sc = getattr(hiit_controller, "state_changed", None)
        if sc is not None:
            try:
                sc.connect(self._on_hiit_state)
            except Exception:
                pass
        tsc = getattr(treadmill_panel, "status_changed", None)
        if tsc is not None:
            try:
                tsc.connect(self._on_treadmill_status)
            except Exception:
                pass

    # ------------------------------------------------------------- utilities
    def _log(self, msg: str) -> None:
        if self._log_fn is not None:
            self._log_fn(msg)
        self.message.emit(msg)

    def _say(self, phrase: str) -> None:
        if self.speaker is not None:
            try:
                self.speaker.say(phrase)
            except Exception:
                pass

    def _now(self) -> float:
        return self._clock()

    def is_active(self) -> bool:
        return self.state in _ACTIVE_STATES

    def _selected_cameras(self) -> List[str]:
        """Return the currently selected camera node names.

        The real GUI exposes selection on ``camera_panel.table`` (a CameraTable);
        the ``selected_full_names`` method is not on the panel itself. Fall back
        gracefully so tests can inject a panel that exposes it directly.
        """
        panel = self.camera_panel
        for target in (panel, getattr(panel, "table", None)):
            if target is None:
                continue
            fn = getattr(target, "selected_full_names", None)
            if callable(fn):
                try:
                    return list(fn())
                except Exception:
                    return []
        return []

    def _set_state(self, state: str) -> None:
        self.state = state
        self.state_changed.emit(state, STATE_LABELS.get(state, state.upper()))

    # ---------------------------------------------------------- readiness
    def readiness(self) -> "AutoRunReadiness":
        """Non-mutating check used by the panel to enable/disable Start."""
        regimen = None
        if self.hiit_controller is not None:
            fn = getattr(self.hiit_controller, "loaded_protocol", None)
            if callable(fn):
                regimen = fn()
        nodes = self._selected_cameras()
        return AutoRunReadiness(
            regimen_name=getattr(regimen, "protocol_name", None) if regimen else None,
            estimated_total_s=float(getattr(regimen, "estimated_total_s", 0.0)) if regimen else 0.0,
            camera_count=len(nodes),
        )

    # ---------------------------------------------------------- entry point
    def start_auto_run(
        self, animal_id: Optional[str] = None, condition: Optional[str] = None
    ) -> bool:
        """Kick off the automated loop. Returns False if preconditions fail."""
        if self.is_active():
            self._log("Auto-Run already in progress; ignoring start.")
            return False

        rd = self.readiness()
        if rd.regimen_name is None:
            self._fail("No HIIT regimen loaded — load one on the Treadmill tab first.",
                       "No protocol loaded")
            return False
        if rd.camera_count == 0:
            self._fail("No cameras selected — select at least one camera first.",
                       "No cameras selected")
            return False

        # 1) reset the HIIT protocol so it can (re)run from the top.
        self._set_state(RESETTING)
        self._log("Auto-Run: resetting HIIT protocol")
        try:
            self.hiit_controller.request_reset()
        except Exception as exc:
            self._fail(f"Failed to reset HIIT protocol: {exc}", "Reset failed")
            return False

        # 2) register + confirm active metadata (rat ID / condition).
        self._set_state(METADATA)
        if not self._register_metadata(animal_id, condition):
            return False  # _register_metadata already reported + reset state

        # 3) start recording and await ROS confirmation.
        self._nodes = self._selected_cameras()
        self._recording_confirmed = set()
        self._set_state(STARTING_REC)
        self._log(f"Auto-Run: starting recording on {len(self._nodes)} camera(s)")
        try:
            self.camera_panel.start_recording()
        except Exception as exc:
            self._fail(f"start_recording raised: {exc}", "Recording start failed")
            return False
        self._deadline = self._now() + self.rec_confirm_timeout_s
        self._poll.start()
        return True

    def _register_metadata(self, animal_id: Optional[str], condition: Optional[str]) -> bool:
        try:
            if animal_id is not None and animal_id.strip():
                self.metadata_panel.animal_id.setText(animal_id.strip())
            if condition is not None and condition.strip():
                self.metadata_panel.condition.setText(condition.strip())
            confirmed = bool(self.metadata_panel.confirm())
        except Exception as exc:
            self._fail(f"Metadata registration failed: {exc}", "Metadata error")
            return False
        if not confirmed:
            # confirm() itself pops a warning box listing the missing fields.
            self._fail("Metadata incomplete — fill required fields and retry.",
                       "Metadata incomplete")
            return False
        self._log("Auto-Run: metadata confirmed")
        return True

    # ---------------------------------------------------------- poll loop
    def _on_poll(self) -> None:
        if self.state == STARTING_REC:
            self._poll_recording_started()
        elif self.state == PROTOCOL:
            self._poll_protocol()
        elif self.state == STOPPING_REC:
            self._poll_recording_stopped()
        else:
            self._poll.stop()

    # ---- awaiting "recording == True" on every selected node
    def _poll_recording_started(self) -> None:
        for full in self._nodes:
            if full in self._recording_confirmed:
                continue
            fut = self.ros.get_status_async(full)
            if fut is None:
                continue
            fut.add_done_callback(
                lambda f, full=full: self._on_status_result(f, full, want_recording=True)
            )
        if self._recording_confirmed.issuperset(self._nodes):
            self._recording_active()
        elif self._now() > self._deadline:
            self._fail(
                "Camera did not confirm recording within "
                f"{self.rec_confirm_timeout_s:.0f}s — treadmill NOT started.",
                "Camera failed to start",
            )

    def _recording_active(self) -> None:
        self._say("Camera recording started")
        self._log("Auto-Run: recording confirmed active")
        # 4) start the HIIT protocol.
        self._protocol_terminal = False
        self._belt_zero = False
        rd = self.readiness()
        self._deadline = self._now() + rd.estimated_total_s + self.protocol_margin_s
        self._set_state(PROTOCOL)
        try:
            self.hiit_controller.request_start()
        except Exception as exc:
            self._fail(f"Failed to start HIIT protocol: {exc}", "Protocol start failed")
            return
        self._say("Treadmill protocol running")
        self._log("Auto-Run: HIIT protocol running")

    # ---- awaiting protocol completion + belt at 0 cm/s
    def _poll_protocol(self) -> None:
        if self._protocol_terminal and self._belt_zero:
            self._protocol_complete()
        elif self._now() > self._deadline:
            if self._protocol_terminal:
                # Protocol finished but telemetry never reported 0 — proceed, warn.
                self._log("Auto-Run: protocol done; belt 0 cm/s not confirmed by "
                          "telemetry (proceeding to stop recording).")
                self._protocol_complete()
            else:
                self._fail("HIIT protocol overran its expected duration.",
                           "Protocol timeout")

    def _protocol_complete(self) -> None:
        self._say("Treadmill stopped")
        self._log("Auto-Run: protocol complete, belt stopped")
        # 5) stop recording and await confirmation.
        self._recording_confirmed = set()
        self._set_state(STOPPING_REC)
        try:
            self.camera_panel.stop_recording()
        except Exception as exc:
            self._fail(f"stop_recording raised: {exc}", "Recording stop failed")
            return
        self._deadline = self._now() + self.rec_stop_timeout_s

    # ---- awaiting "recording == False" on every selected node
    def _poll_recording_stopped(self) -> None:
        for full in self._nodes:
            if full in self._recording_confirmed:
                continue
            fut = self.ros.get_status_async(full)
            if fut is None:
                continue
            fut.add_done_callback(
                lambda f, full=full: self._on_status_result(f, full, want_recording=False)
            )
        if self._recording_confirmed.issuperset(self._nodes):
            self._done()
        elif self._now() > self._deadline:
            # Recording didn't confirm stopped; surface it but don't error-loop —
            # the belt is already stopped, so end the run with a warning.
            self._say("Camera recording stopped")
            self._log("Auto-Run: stop not confirmed within "
                      f"{self.rec_stop_timeout_s:.0f}s (ending run anyway).")
            self._done()

    def _done(self) -> None:
        self._poll.stop()
        self._say("Camera recording stopped")
        self._log("Auto-Run: complete ✔")
        self._set_state(COMPLETE)

    # ---------------------------------------------------------- observers
    def _on_status_result(self, fut: Any, full: str, want_recording: bool) -> None:
        try:
            resp = fut.result()
        except Exception:
            return
        if bool(getattr(resp, "recording", False)) == want_recording:
            self._recording_confirmed.add(full)

    def _on_hiit_state(self, old: Any, new: Any) -> None:
        if self.state != PROTOCOL:
            return
        name = getattr(new, "name", getattr(new, "value", str(new)))
        if str(name).lower() in ("complete", "aborted"):
            self._protocol_terminal = True

    def _on_treadmill_status(self, msg: Any) -> None:
        # Only accept 0 cm/s as "completed" once the protocol has actually reached a
        # terminal state — otherwise a pre-run idle belt (also 0 cm/s) would falsely
        # satisfy completion before the protocol has run.
        if self.state != PROTOCOL or not self._protocol_terminal:
            return
        rep = getattr(msg, "reported_speed_cm_s", -1)
        if rep == 0:
            self._belt_zero = True

    # ---------------------------------------------------------- abort / fail
    def abort(self) -> None:
        """Operator-requested stop: tear down to a safe state."""
        if not self.is_active():
            return
        self._log("Auto-Run: ABORT requested")
        self._safe_teardown()
        self._say("Auto run aborted")
        self._set_state(ERROR)

    def _fail(self, msg: str, spoken: str) -> None:
        self._poll.stop()
        self._log(f"Auto-Run ERROR: {msg}")
        self._safe_teardown()
        self._say(spoken)
        self._set_state(ERROR)

    def _safe_teardown(self) -> None:
        """Best-effort: never leave the belt moving or a recording running."""
        self._poll.stop()
        # Stop the belt if a protocol may be running.
        try:
            status = getattr(self.treadmill_panel, "latest_status", None)
            running = bool(getattr(status, "running", False)) if status is not None else False
            rep = getattr(status, "reported_speed_cm_s", 0) if status is not None else 0
            if self.hiit_controller is not None and (running or (isinstance(rep, int) and rep > 0)):
                self.hiit_controller.request_gentle_stop()
        except Exception as exc:
            self._log(f"Auto-Run teardown: belt stop failed: {exc}")
        # Stop recording if we had started it.
        if self.state in (STARTING_REC, PROTOCOL, STOPPING_REC):
            try:
                self.camera_panel.stop_recording()
            except Exception as exc:
                self._log(f"Auto-Run teardown: stop_recording failed: {exc}")


class AutoRunReadiness:
    """Small value object describing whether an auto-run can start."""

    def __init__(self, regimen_name: Optional[str], estimated_total_s: float, camera_count: int) -> None:
        self.regimen_name = regimen_name
        self.estimated_total_s = estimated_total_s
        self.camera_count = camera_count

    @property
    def ready(self) -> bool:
        return self.regimen_name is not None and self.camera_count > 0
