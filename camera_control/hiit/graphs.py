# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""Live visualizations for the HIIT trainer: speed-vs-time profile, a colored
phase scrubber, and a commanded-vs-reported telemetry scope.

PySide6 only (QtGui.QPainter — no QtCharts, so no extra VM dependency). No
rclpy. All widgets are fed plain numbers by the controller, so they are
constructible and renderable headlessly (offscreen) for tests.
"""

from __future__ import annotations

from collections import deque
from typing import Deque, List, Optional, Tuple

from PySide6 import QtCore, QtGui, QtWidgets

from . import settings as hsettings

MAX_SPEED = 100  # y-axis ceiling (matches device max)


# --------------------------
# Color mapping
# --------------------------
def speed_color(speed: float) -> QtGui.QColor:
    """Green (slow) -> red (fast); dark red at/above 48 cm/s (48-60 band)."""
    cap = 48.0
    if speed >= cap:
        return QtGui.QColor(139, 0, 0)  # dark red
    t = max(0.0, min(1.0, speed / cap))
    c = QtGui.QColor()
    c.setHsvF((120.0 * (1.0 - t)) / 360.0, 0.85, 0.92 - 0.30 * t)
    return c


def _grid_pen() -> QtGui.QPen:
    return QtGui.QPen(QtGui.QColor(210, 210, 210), 1)


# --------------------------
# Speed vs time profile
# --------------------------
class ProfilePlot(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._points: List[Tuple[float, float]] = []
        self._total = 0.0
        self._cursor_t: Optional[float] = None
        self.setMinimumSize(420, 220)

    def set_protocol(self, points: List[Tuple[float, float]], total_s: float) -> None:
        self._points = list(points)
        self._total = max(total_s, 1e-6)
        self._cursor_t = None
        self.update()

    def set_cursor(self, t: Optional[float]) -> None:
        self._cursor_t = t
        self.update()

    def paintEvent(self, _event) -> None:
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor(255, 255, 255))
        m = 36
        r = self.rect().adjusted(m, 12, -12, -24)
        # axes
        p.setPen(QtGui.QPen(QtGui.QColor(120, 120, 120), 1))
        p.drawLine(r.bottomLeft(), r.bottomRight())
        p.drawLine(r.topLeft(), r.bottomLeft())
        # y gridlines 0..100
        p.setPen(_grid_pen())
        for s in range(0, MAX_SPEED + 1, 20):
            y = r.bottom() - (s / MAX_SPEED) * r.height()
            p.drawLine(int(r.left()), int(y), int(r.right()), int(y))
            p.setPen(QtGui.QPen(QtGui.QColor(120, 120, 120)))
            p.drawText(2, int(y) + 4, f"{s}")
            p.setPen(_grid_pen())
        if not self._points:
            p.setPen(QtGui.QColor(150, 150, 150))
            p.drawText(r, QtCore.Qt.AlignCenter, "No regimen loaded")
            return

        def to_px(t, spd):
            x = r.left() + (t / self._total) * r.width()
            y = r.bottom() - (spd / MAX_SPEED) * r.height()
            return QtCore.QPointF(x, y)

        poly = QtGui.QPolygonF([to_px(t, s) for t, s in self._points])
        p.setPen(QtGui.QPen(QtGui.QColor(33, 102, 172), 2))
        p.drawPolyline(poly)
        # cursor
        if self._cursor_t is not None and self._total > 0:
            cx = r.left() + (min(self._cursor_t, self._total) / self._total) * r.width()
            p.setPen(QtGui.QPen(QtGui.QColor(217, 48, 37), 2, QtCore.Qt.DashLine))
            p.drawLine(int(cx), int(r.top()), int(cx), int(r.bottom()))
        p.setPen(QtGui.QColor(120, 120, 120))
        p.drawText(int(r.right()) - 60, int(r.bottom()) + 18, f"{self._total:.0f}s")


# --------------------------
# Colored phase scrubber (inline)
# --------------------------
class PhaseScrubber(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._stages: List[Tuple[float, float, str]] = []  # (duration, speed, label)
        self._total = 0.0
        self._index = -1
        self._frac = 0.0
        self.setMinimumHeight(26)
        self.setMaximumHeight(30)

    def set_stages(self, stages: List[Tuple[float, float, str]]) -> None:
        self._stages = list(stages)
        self._total = max(sum(max(d, 0.0) for d, _, _ in self._stages), 1e-6)
        self._index = -1
        self._frac = 0.0
        self.update()

    def set_position(self, index: int, frac: float) -> None:
        self._index = index
        self._frac = max(0.0, min(1.0, frac))
        self.update()

    def paintEvent(self, _event) -> None:
        p = QtGui.QPainter(self)
        r = self.rect().adjusted(1, 1, -1, -1)
        if not self._stages:
            p.fillRect(r, QtGui.QColor(235, 235, 235))
            return
        x = float(r.left())
        for i, (dur, spd, _label) in enumerate(self._stages):
            w = (max(dur, 0.0) / self._total) * r.width()
            seg = QtCore.QRectF(x, r.top(), w, r.height())
            p.fillRect(seg, speed_color(spd))
            if i == self._index:
                p.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0), 2))
                p.drawRect(seg.adjusted(1, 1, -1, -1))
            x += w
        # position marker
        if self._index >= 0:
            done = sum(max(d, 0.0) for d, _, _ in self._stages[: self._index])
            cur = max(self._stages[self._index][0], 0.0) if self._index < len(self._stages) else 0.0
            px = r.left() + ((done + cur * self._frac) / self._total) * r.width()
            p.setPen(QtGui.QPen(QtGui.QColor(20, 20, 20), 2))
            p.drawLine(int(px), r.top(), int(px), r.bottom())


# --------------------------
# Commanded vs reported telemetry scope
# --------------------------
class TelemetryPlot(QtWidgets.QWidget):
    def __init__(self, window_s: float = 90.0, parent=None):
        super().__init__(parent)
        self._window = float(window_s)
        self._cmd: Deque[Tuple[float, float]] = deque(maxlen=4000)
        self._rep: Deque[Tuple[float, float]] = deque(maxlen=4000)
        self.setMinimumSize(420, 200)

    def set_window(self, seconds: float) -> None:
        self._window = max(5.0, float(seconds))
        self.update()

    def add_sample(self, t: float, commanded: Optional[float], reported: Optional[float]) -> None:
        if commanded is not None and commanded >= 0:
            self._cmd.append((t, float(commanded)))
        if reported is not None and reported >= 0:
            self._rep.append((t, float(reported)))
        self.update()

    def clear(self) -> None:
        self._cmd.clear()
        self._rep.clear()
        self.update()

    def _latest_t(self) -> Optional[float]:
        ts = [s[0] for s in self._cmd] + [s[0] for s in self._rep]
        return max(ts) if ts else None

    def paintEvent(self, _event) -> None:
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor(255, 255, 255))
        m = 36
        r = self.rect().adjusted(m, 24, -12, -24)
        p.setPen(QtGui.QPen(QtGui.QColor(120, 120, 120), 1))
        p.drawLine(r.bottomLeft(), r.bottomRight())
        p.drawLine(r.topLeft(), r.bottomLeft())
        p.setPen(_grid_pen())
        for s in range(0, MAX_SPEED + 1, 20):
            y = r.bottom() - (s / MAX_SPEED) * r.height()
            p.drawLine(int(r.left()), int(y), int(r.right()), int(y))
            p.setPen(QtGui.QPen(QtGui.QColor(120, 120, 120)))
            p.drawText(2, int(y) + 4, f"{s}")
            p.setPen(_grid_pen())

        latest = self._latest_t()
        # legend
        p.setPen(QtGui.QColor(33, 102, 172))
        p.drawText(int(r.left()), 16, "■ commanded")
        p.setPen(QtGui.QColor(217, 48, 37))
        p.drawText(int(r.left()) + 110, 16, "■ reported")
        if latest is None:
            p.setPen(QtGui.QColor(150, 150, 150))
            p.drawText(r, QtCore.Qt.AlignCenter, "waiting for /treadmill_host/status…")
            return

        t0 = latest - self._window

        def series_poly(series):
            pts = []
            for t, v in series:
                if t < t0:
                    continue
                x = r.left() + ((t - t0) / self._window) * r.width()
                y = r.bottom() - (min(v, MAX_SPEED) / MAX_SPEED) * r.height()
                pts.append(QtCore.QPointF(x, y))
            return QtGui.QPolygonF(pts)

        p.setPen(QtGui.QPen(QtGui.QColor(33, 102, 172), 2))
        p.drawPolyline(series_poly(self._cmd))
        p.setPen(QtGui.QPen(QtGui.QColor(217, 48, 37), 2))
        p.drawPolyline(series_poly(self._rep))
        p.setPen(QtGui.QColor(120, 120, 120))
        p.drawText(int(r.right()) - 70, int(r.bottom()) + 18, f"last {self._window:.0f}s")


# --------------------------
# Detachable dialogs
# --------------------------
class _GraphDialog(QtWidgets.QDialog):
    closed = QtCore.Signal()

    def __init__(self, title: str, geom_key: str, parent=None):
        super().__init__(parent)
        self._geom_key = geom_key
        self.setWindowTitle(title)
        self.setWindowFlag(QtCore.Qt.Window, True)  # independent, movable window
        self.setModal(False)
        geo = hsettings.restore_geometry(geom_key)
        if geo is not None:
            self.restoreGeometry(geo)

    def closeEvent(self, event) -> None:
        hsettings.save_geometry(self._geom_key, self.saveGeometry())
        self.closed.emit()
        super().closeEvent(event)


class ProfileGraphDialog(_GraphDialog):
    def __init__(self, parent=None):
        super().__init__("HIIT — Speed Profile", "profile_graph", parent)
        self.plot = ProfilePlot()
        lay = QtWidgets.QVBoxLayout(self)
        lay.addWidget(self.plot)
        self.resize(560, 300)

    def set_protocol(self, points, total_s):
        self.plot.set_protocol(points, total_s)

    def set_cursor(self, t):
        self.plot.set_cursor(t)


class TelemetryGraphDialog(_GraphDialog):
    def __init__(self, window_s: float = 90.0, parent=None):
        super().__init__("HIIT — Commanded vs Reported", "telemetry_graph", parent)
        self.plot = TelemetryPlot(window_s=window_s)
        win_row = QtWidgets.QHBoxLayout()
        win_row.addWidget(QtWidgets.QLabel("Window:"))
        self.window_spin = QtWidgets.QSpinBox()
        self.window_spin.setRange(10, 600)
        self.window_spin.setValue(int(window_s))
        self.window_spin.setSuffix(" s")
        self.window_spin.valueChanged.connect(self.plot.set_window)  # non-persisting
        win_row.addWidget(self.window_spin)
        win_row.addStretch(1)
        lay = QtWidgets.QVBoxLayout(self)
        lay.addLayout(win_row)
        lay.addWidget(self.plot)
        self.resize(560, 280)

    def add_sample(self, t, commanded, reported):
        self.plot.add_sample(t, commanded, reported)
