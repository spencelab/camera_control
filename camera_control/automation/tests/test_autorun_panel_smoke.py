# Author: Andrew England (andrewengland19)
"""Headless smoke test: AutoRunPanel + AutoRunController wired together. Confirms
the panel constructs offscreen, reflects state transitions on the banner, toggles
the primary button between START and ABORT, and mirrors the mute checkbox."""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6 import QtWidgets  # noqa: E402

from automation.controller import AutoRunController, PROTOCOL, READY  # noqa: E402
from automation.panel import AutoRunPanel  # noqa: E402

# Reuse the controller test fakes.
from test_controller import FakeClock, FakeRos, FakeCamera, FakeMetadata, FakeTreadmill, FakeHiit  # noqa: E402


@pytest.fixture(scope="module")
def _qapp():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _wire(nodes=("cam1",)):
    ros = FakeRos()
    cam = FakeCamera(ros, list(nodes))
    meta = FakeMetadata(ok=True)
    tread = FakeTreadmill()
    hiit = FakeHiit(proto=True)

    class Spk:
        muted = False

        def say(self, phrase):
            pass

    ctrl = AutoRunController(ros, cam, meta, tread, hiit, speaker=Spk(), clock=FakeClock())
    panel = AutoRunPanel(ctrl)
    return panel, ctrl


def test_panel_constructs_and_shows_readiness(_qapp):
    panel, ctrl = _wire(nodes=("cam1", "cam2"))
    assert "unit-regimen" in panel.readiness_label.text()
    assert panel.primary_btn.isEnabled()          # ready -> start enabled
    assert panel.banner.text() == "READY"


def test_start_toggles_button_to_abort(_qapp):
    panel, ctrl = _wire()
    panel.animal_id.setText("rat9")
    panel._on_primary_clicked()                    # START
    ctrl._on_poll()                                # advance to PROTOCOL
    assert ctrl.state == PROTOCOL
    assert "ABORT" in panel.primary_btn.text()
    assert "PROTOCOL" in panel.banner.text()

    panel._on_primary_clicked()                    # now acts as ABORT
    assert not ctrl.is_active()
    assert "START" in panel.primary_btn.text()


def test_mute_checkbox_mirrors_speaker(_qapp):
    panel, ctrl = _wire()
    panel.mute_chk.setChecked(True)
    assert ctrl.speaker.muted is True
    panel.mute_chk.setChecked(False)
    assert ctrl.speaker.muted is False


def test_disabled_start_when_no_regimen(_qapp):
    ros = FakeRos()
    cam = FakeCamera(ros, ["cam1"])
    ctrl = AutoRunController(ros, cam, FakeMetadata(), FakeTreadmill(), FakeHiit(proto=False),
                             clock=FakeClock())
    panel = AutoRunPanel(ctrl)
    assert not panel.primary_btn.isEnabled()
    assert panel.banner.text() == "READY"
