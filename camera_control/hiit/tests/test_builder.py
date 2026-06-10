# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""Offscreen smoke tests for the regimen builder. Skips without PySide6."""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

from PySide6 import QtWidgets  # noqa: E402

from hiit import protocol as protocol_mod  # noqa: E402
from hiit.builder import RegimenBuilderDialog  # noqa: E402


@pytest.fixture(scope="module")
def _app():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


def _example_dict():
    return {
        "protocol_name": "Built",
        "date": "2026-06-08",
        "description": "from builder",
        "target": 36, "step": 5, "every": 120,
        "defaults": {"ramp_rate_cm_s2": 3},
        "steps": [
            {"type": "run", "speed": 16, "duration": 90, "ramp_rate": 2, "label": "warm-up"},
            {"type": "loop", "count": 3, "steps": [
                {"type": "run", "speed": 36, "duration": 15, "ramp_rate": 6, "label": "sprint"},
                {"type": "run", "speed": 25, "duration": 90, "ramp_rate": 2, "label": "recovery"},
            ]},
            {"type": "run", "speed": 0, "duration": 0, "ramp_rate": 2, "label": "stop"},
        ],
    }


def test_builder_roundtrips_through_schema(_app):
    dlg = RegimenBuilderDialog()
    dlg.load_from_protocol_dict(_example_dict())
    # the tree assembles back into a schema-valid mapping
    data = dlg.assemble_dict()
    p = protocol_mod.protocol_from_dict(data)
    # 1 warm-up + 3*2 loop + 1 stop = 8 stages
    assert len(p.stages) == 8
    assert p.stages[0].label == "warm-up"
    assert p.stages[-1].speed == 0


def test_builder_live_validation(_app):
    dlg = RegimenBuilderDialog()
    dlg.load_from_protocol_dict(_example_dict())
    proto = dlg._refresh()
    assert proto is not None
    assert "valid" in dlg.status.text()


def test_builder_empty_is_invalid(_app):
    dlg = RegimenBuilderDialog()
    dlg.tree.clear()
    assert dlg._refresh() is None
    assert "⚠" in dlg.status.text()


def test_builder_add_step_programmatically(_app):
    dlg = RegimenBuilderDialog()
    dlg.tree.clear()
    dlg.tree.addTopLevelItem(
        dlg._make_run_item({"speed": 20, "duration": 30, "ramp_rate": 2, "label": "go"})
    )
    data = dlg.assemble_dict()
    assert data["steps"][0]["speed"] == 20
    assert protocol_mod.protocol_from_dict(data).stages[0].label == "go"
