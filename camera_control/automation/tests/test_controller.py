# Author: Andrew England (andrewengland19)
"""State-machine tests for AutoRunController with mocked ros/panels/hiit and an
injected clock. Runs headless under offscreen Qt (the controller uses QObject +
QTimer). The poll timer is never relied on to fire — ``_on_poll`` is driven
manually so the sequence is deterministic."""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6 import QtCore, QtWidgets  # noqa: E402

from automation.controller import (  # noqa: E402
    AutoRunController,
    COMPLETE,
    ERROR,
    PROTOCOL,
    STARTING_REC,
)
from hiit.runner import HiitState  # noqa: E402


@pytest.fixture(scope="module")
def _qapp():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


# --------------------------------------------------------------------- fakes
class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += float(dt)


class FakeStatus:
    def __init__(self, recording):
        self.recording = recording


class FakeFuture:
    def __init__(self, result=None, exc=None):
        self._r, self._e = result, exc

    def add_done_callback(self, cb):
        cb(self)  # synchronous — deterministic in tests

    def result(self):
        if self._e is not None:
            raise self._e
        return self._r


class FakeRos:
    def __init__(self):
        self.recording = {}

    def get_status_async(self, full):
        return FakeFuture(result=FakeStatus(self.recording.get(full, False)))


class FakeLine:
    def __init__(self, text=""):
        self._t = text

    def setText(self, t):
        self._t = t

    def text(self):
        return self._t


class FakeCamera:
    def __init__(self, ros, nodes, auto_confirm=True):
        self.ros, self._nodes, self._auto = ros, nodes, auto_confirm
        self.start_called = self.stop_called = 0

    def selected_full_names(self):
        return list(self._nodes)

    def start_recording(self):
        self.start_called += 1
        if self._auto:
            for n in self._nodes:
                self.ros.recording[n] = True

    def stop_recording(self):
        self.stop_called += 1
        for n in self._nodes:
            self.ros.recording[n] = False


class FakeMetadata:
    def __init__(self, ok=True):
        self.animal_id = FakeLine()
        self.condition = FakeLine()
        self._ok = ok
        self.confirmed = False
        self.confirm_calls = 0

    def confirm(self):
        self.confirm_calls += 1
        self.confirmed = self._ok
        return self._ok


class FakeTStatus:
    def __init__(self, running=False, reported=0):
        self.running = running
        self.reported_speed_cm_s = reported


class FakeTreadmill(QtCore.QObject):
    status_changed = QtCore.Signal(object)

    def __init__(self):
        super().__init__()
        self.latest_status = FakeTStatus()


class FakeProto:
    protocol_name = "unit-regimen"
    estimated_total_s = 30.0


class FakeHiit(QtCore.QObject):
    state_changed = QtCore.Signal(object, object)

    def __init__(self, proto=True):
        super().__init__()
        self._proto = FakeProto() if proto else None
        self.reset_calls = self.start_calls = self.gentle_calls = 0

    def loaded_protocol(self):
        return self._proto

    def request_reset(self):
        self.reset_calls += 1

    def request_start(self):
        self.start_calls += 1

    def request_gentle_stop(self):
        self.gentle_calls += 1


def _make(nodes=("cam1",), auto_confirm=True, meta_ok=True, proto=True):
    ros = FakeRos()
    cam = FakeCamera(ros, list(nodes), auto_confirm=auto_confirm)
    meta = FakeMetadata(ok=meta_ok)
    tread = FakeTreadmill()
    hiit = FakeHiit(proto=proto)
    clock = FakeClock()
    spoken = []

    class Spk:
        def say(self, phrase):
            spoken.append(phrase)

    ctrl = AutoRunController(
        ros, cam, meta, tread, hiit, speaker=Spk(), clock=clock,
    )
    return ctrl, dict(ros=ros, cam=cam, meta=meta, tread=tread, hiit=hiit,
                      clock=clock, spoken=spoken)


# --------------------------------------------------------------------- tests
def test_happy_path_full_loop(_qapp):
    ctrl, ctx = _make()
    assert ctrl.start_auto_run(animal_id="rat1", condition="baseline") is True

    # metadata mirrored + confirmed, HIIT reset, recording started
    assert ctx["meta"].animal_id.text() == "rat1"
    assert ctx["meta"].condition.text() == "baseline"
    assert ctx["hiit"].reset_calls == 1
    assert ctx["meta"].confirm_calls == 1
    assert ctx["cam"].start_called == 1
    assert ctrl.state == STARTING_REC

    # poll: recording confirmed -> protocol starts
    ctrl._on_poll()
    assert ctrl.state == PROTOCOL
    assert ctx["hiit"].start_calls == 1
    assert "Camera recording started" in ctx["spoken"]
    assert "Treadmill protocol running" in ctx["spoken"]

    # belt runs, then protocol completes + telemetry reads 0 cm/s
    ctx["tread"].status_changed.emit(FakeTStatus(running=True, reported=20))
    ctx["hiit"].state_changed.emit(HiitState.RUNNING, HiitState.COMPLETE)
    ctx["tread"].status_changed.emit(FakeTStatus(running=False, reported=0))

    ctrl._on_poll()  # sees terminal + belt zero -> stop recording
    assert "Treadmill stopped" in ctx["spoken"]
    assert ctx["cam"].stop_called == 1

    ctrl._on_poll()  # stop confirmed -> complete
    assert ctrl.state == COMPLETE
    assert "Camera recording stopped" in ctx["spoken"]


def test_camera_fail_to_start_does_not_start_treadmill(_qapp):
    ctrl, ctx = _make(auto_confirm=False)  # recording never confirms
    ctrl.start_auto_run(animal_id="rat1")
    assert ctrl.state == STARTING_REC

    ctrl._on_poll()               # still waiting
    assert ctrl.state == STARTING_REC
    ctx["clock"].advance(9.0)     # blow past the 8s confirm timeout
    ctrl._on_poll()

    assert ctrl.state == ERROR
    assert ctx["hiit"].start_calls == 0          # treadmill NEVER started
    assert ctx["cam"].stop_called >= 1           # teardown stopped recording


def test_metadata_incomplete_aborts_before_recording(_qapp):
    ctrl, ctx = _make(meta_ok=False)
    ctrl.start_auto_run(animal_id="rat1")
    assert ctrl.state == ERROR
    assert ctx["hiit"].reset_calls == 1          # reset happened first
    assert ctx["cam"].start_called == 0          # recording never started
    assert ctx["hiit"].start_calls == 0


def test_no_regimen_loaded_refuses(_qapp):
    ctrl, ctx = _make(proto=False)
    assert ctrl.start_auto_run() is False
    assert ctrl.state == ERROR
    assert ctx["hiit"].reset_calls == 0          # bailed before touching HIIT
    assert ctx["cam"].start_called == 0


def test_no_cameras_selected_refuses(_qapp):
    ctrl, ctx = _make(nodes=())
    assert ctrl.start_auto_run() is False
    assert ctrl.state == ERROR
    assert ctx["cam"].start_called == 0


def test_abort_while_protocol_running_stops_belt_and_recording(_qapp):
    ctrl, ctx = _make()
    ctrl.start_auto_run(animal_id="rat1")
    ctrl._on_poll()  # -> PROTOCOL
    assert ctrl.state == PROTOCOL
    # belt is moving
    ctx["tread"].latest_status = FakeTStatus(running=True, reported=25)

    ctrl.abort()
    assert ctrl.state == ERROR
    assert ctx["hiit"].gentle_calls == 1         # belt eased to a stop
    assert ctx["cam"].stop_called == 1           # recording stopped


def test_protocol_overrun_times_out(_qapp):
    ctrl, ctx = _make()
    ctrl.start_auto_run(animal_id="rat1")
    ctrl._on_poll()  # -> PROTOCOL
    # Never signal completion; run past estimated_total_s + margin.
    ctx["clock"].advance(30.0 + 15.0 + 1.0)
    ctrl._on_poll()
    assert ctrl.state == ERROR


def test_protocol_done_but_no_zero_telemetry_still_completes(_qapp):
    ctrl, ctx = _make()
    ctrl.start_auto_run(animal_id="rat1")
    ctrl._on_poll()  # -> PROTOCOL
    ctx["hiit"].state_changed.emit(HiitState.RUNNING, HiitState.COMPLETE)
    # No telemetry 0 arrives; timeout should proceed (belt already commanded 0).
    ctx["clock"].advance(30.0 + 15.0 + 1.0)
    ctrl._on_poll()          # terminal + deadline -> proceeds to stop
    ctrl._on_poll()          # stop confirmed -> complete
    assert ctrl.state == COMPLETE


def test_readiness_reports_regimen_and_cameras(_qapp):
    ctrl, ctx = _make(nodes=("cam1", "cam2"))
    rd = ctrl.readiness()
    assert rd.regimen_name == "unit-regimen"
    assert rd.camera_count == 2
    assert rd.ready is True
