# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""Headless integration smoke test: HiitPanel + HiitController + HiitRunner.

Skips automatically where PySide6 is unavailable (e.g. a ROS-less MBP). Runs
under the offscreen Qt platform with an injected fake clock and ticked manually,
so no real event loop or ROS is needed. Intended to run on the VM (PySide6 ships
with ROS) and anywhere PySide6 is installed.
"""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6 import QtWidgets  # noqa: E402

from hiit.controller import HiitController  # noqa: E402
from hiit.panel import HiitPanel  # noqa: E402
from hiit.runner import HiitState  # noqa: E402


class _FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def set(self, t):
        self.t = float(t)


class _MockStatus:
    commanded_speed_cm_s = 0


class _MockRos:
    def __init__(self):
        self.actions = []

    def treadmill_trigger_async(self, action):
        self.actions.append(action)
        return None


class _MockTreadmillPanel:
    def __init__(self):
        self.latest_status = _MockStatus()
        self._hiit_lock = False
        self.speeds = []
        self.manual_enabled = True

    def set_speed(self, v):
        self.speeds.append(v)
        self.latest_status.commanded_speed_cm_s = v

    def set_manual_enabled(self, enabled):
        self.manual_enabled = enabled


@pytest.fixture(scope="module")
def _app():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def _write_protocol(tmp_path):
    p = tmp_path / "smoke.yaml"
    p.write_text(
        "protocol_name: Smoke\n"
        "defaults:\n  ramp_rate_cm_s2: 5\n"
        "steps:\n"
        "  - {type: run, speed: 10, duration: 2, ramp_rate: 5, label: go}\n"
        "  - {type: run, speed: 0, duration: 0, ramp_rate: 10, label: stop}\n",
        encoding="utf-8",
    )
    return p


def test_import_run_complete_with_lockout(_app, tmp_path):
    clk = _FakeClock()
    ros = _MockRos()
    tmill = _MockTreadmillPanel()
    panel = HiitPanel()
    run_dir = tmp_path / "runs"
    ctrl = HiitController(
        ros, tmill, panel, log_fn=lambda m: None, clock=clk, run_log_dir=str(run_dir)
    )
    panel.set_controller(ctrl)

    # import
    ctrl.request_import(str(_write_protocol(tmp_path)))
    assert panel.run_btn.isEnabled()
    assert "Smoke" in panel.regimen_label.text()

    # start -> lockout engaged, take_control + run issued
    ctrl.request_start()
    assert tmill._hiit_lock is True
    assert tmill.manual_enabled is False
    assert "take_control" in ros.actions and "run" in ros.actions
    assert panel.run_btn.isEnabled() is False
    assert panel.abort_btn.isEnabled() is True

    # drive the schedule to completion
    for t in [x * 0.5 for x in range(1, 12)]:
        clk.set(t)
        ctrl._on_tick()

    assert ctrl._runner.state == HiitState.COMPLETE
    assert tmill.speeds[-1] == 0          # final stop stage
    assert "stop" in ros.actions
    # lockout released -> manual control restored
    assert tmill._hiit_lock is False
    assert tmill.manual_enabled is True
    assert panel.reset_btn.isEnabled() is True

    # a run-log was written with both stages and a complete outcome
    import yaml
    logs = list(run_dir.glob("hiit_run_*.yaml"))
    assert len(logs) == 1
    data = yaml.safe_load(logs[0].read_text(encoding="utf-8"))["hiit_run"]
    assert data["outcome"] == "complete"
    assert data["stage_count"] == 2
    assert data["protocol_name"] == "Smoke"


def test_pause_resume_and_abort(_app, tmp_path):
    clk = _FakeClock()
    ros = _MockRos()
    tmill = _MockTreadmillPanel()
    panel = HiitPanel()
    ctrl = HiitController(ros, tmill, panel, log_fn=lambda m: None, clock=clk)
    panel.set_controller(ctrl)
    ctrl.request_import(str(_write_protocol(tmp_path)))
    ctrl.request_start()

    clk.set(0.5)
    ctrl._on_tick()
    ctrl.request_toggle_pause()
    assert ctrl._runner.state == HiitState.PAUSED
    assert panel.pause_btn.text().endswith("Resume")
    assert tmill._hiit_lock is True  # stays locked while paused

    ctrl.request_toggle_pause()  # resume
    assert ctrl._runner.state == HiitState.RUNNING

    ctrl.request_abort()
    assert ctrl._runner.state == HiitState.ABORTED
    assert tmill.speeds[-1] == 0
    assert tmill._hiit_lock is False  # released on abort


def test_bad_file_shows_error_not_crash(_app, tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("protocol_name: x\nsteps:\n  - {type: run, speed: 999, duration: 1, ramp_rate: 1}\n", encoding="utf-8")
    errors = []
    ros = _MockRos()
    tmill = _MockTreadmillPanel()
    panel = HiitPanel()
    panel.show_error = lambda msg: errors.append(msg)  # avoid modal dialog in headless
    ctrl = HiitController(ros, tmill, panel, log_fn=lambda m: None)
    panel.set_controller(ctrl)
    ctrl.request_import(str(bad))
    assert errors and "out of range" in errors[0]
    assert ctrl._runner is None  # nothing loaded
