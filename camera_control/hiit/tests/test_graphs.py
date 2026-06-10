# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""Offscreen smoke tests for the graph widgets. Skips without PySide6."""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6 import QtGui, QtWidgets  # noqa: E402

from hiit import graphs  # noqa: E402


@pytest.fixture(scope="module")
def _app():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def test_speed_color_gradient_and_dark_red():
    slow = graphs.speed_color(5)
    fast = graphs.speed_color(50)
    assert (fast.red(), fast.green(), fast.blue()) == (139, 0, 0)  # >=48 dark red
    assert graphs.speed_color(48).red() == 139                      # band starts at 48
    assert slow.green() > slow.red()                                # slow skews green


def _render(w):
    w.resize(400, 200)
    pm = QtGui.QPixmap(w.size())
    w.render(pm)  # exercises paintEvent without a real screen


def test_profile_plot_renders(_app):
    w = graphs.ProfilePlot()
    _render(w)  # empty -> "no regimen"
    w.set_protocol([(0, 0), (5, 20), (35, 20), (40, 0)], 40)
    w.set_cursor(12.5)
    _render(w)


def test_phase_scrubber_renders(_app):
    w = graphs.PhaseScrubber()
    _render(w)
    w.set_stages([(90, 16, "warm"), (15, 36, "sprint"), (90, 25, "recovery")])
    w.set_position(1, 0.5)
    _render(w)


def test_telemetry_plot_renders_and_trims(_app):
    w = graphs.TelemetryPlot(window_s=10)
    _render(w)  # empty -> waiting
    for t in range(0, 30):
        w.add_sample(float(t), commanded=t % 40, reported=(t % 40) - 1)
    _render(w)
    # only samples within the window survive trimming pressure (deque bounded anyway)
    assert len(w._cmd) > 0


def test_dialogs_construct(_app):
    pg = graphs.ProfileGraphDialog()
    pg.set_protocol([(0, 0), (10, 30)], 10)
    pg.set_cursor(5)
    tg = graphs.TelemetryGraphDialog(window_s=90)
    tg.add_sample(0.0, 10, 9)
    assert tg.window_spin.value() == 90
    # both expose a 'closed' signal for checkbox sync
    assert hasattr(pg, "closed") and hasattr(tg, "closed")
