# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""Persistent, on-the-fly-changeable settings for the HIIT trainer.

Thin wrapper over QtCore.QSettings (Qt-native, no extra dependency). Persists
the regimen save directory, the run-log directory, and graph-window geometry so
the PI can point things wherever they like and have it stick between sessions.
No rclpy. Falls back to sane home-dir defaults.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from PySide6 import QtCore

ORG = "SpenceLab"
APP = "camera_control_hiit"

KEY_REGIMEN_DIR = "paths/regimen_dir"
KEY_RUN_LOG_DIR = "paths/run_log_dir"


def _settings() -> QtCore.QSettings:
    return QtCore.QSettings(ORG, APP)


def _default_regimen_dir() -> Path:
    return Path.home() / "hiit_protocols"


def _default_run_log_dir() -> Path:
    return Path.home() / "hiit_runs"


def get_regimen_dir() -> Path:
    v = _settings().value(KEY_REGIMEN_DIR, "")
    return Path(v).expanduser() if v else _default_regimen_dir()


def set_regimen_dir(path: str | Path) -> None:
    _settings().setValue(KEY_REGIMEN_DIR, str(path))


def get_run_log_dir() -> Path:
    v = _settings().value(KEY_RUN_LOG_DIR, "")
    return Path(v).expanduser() if v else _default_run_log_dir()


def set_run_log_dir(path: str | Path) -> None:
    _settings().setValue(KEY_RUN_LOG_DIR, str(path))


def save_geometry(name: str, geometry: QtCore.QByteArray) -> None:
    _settings().setValue(f"geometry/{name}", geometry)


def restore_geometry(name: str) -> Optional[QtCore.QByteArray]:
    v = _settings().value(f"geometry/{name}")
    return v if isinstance(v, QtCore.QByteArray) else None
