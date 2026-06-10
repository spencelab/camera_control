# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""Interactive regimen builder — author HIIT protocols without writing YAML.

PySide6 only; no rclpy. A tree of run-steps and "repeat ×N" groups (the minimum
that is friendly and reproduces example_hiit.yaml). The assembled mapping is
validated with the SAME protocol.protocol_from_dict() the loader uses, then
written via protocol.to_yaml_document() to a location chosen on the fly
(remembered via settings). One source of truth: round-trips through the schema.
"""

from __future__ import annotations

from datetime import date as _date
from pathlib import Path
from typing import Any, Dict, List, Optional

from PySide6 import QtCore, QtWidgets

from . import protocol as protocol_mod
from . import settings as hsettings

_ROLE = QtCore.Qt.UserRole


# --------------------------
# Small per-item editors
# --------------------------
class _RunStepDialog(QtWidgets.QDialog):
    def __init__(self, parent=None, data: Optional[Dict[str, Any]] = None):
        super().__init__(parent)
        self.setWindowTitle("Run step")
        data = data or {}
        self.speed = QtWidgets.QSpinBox()
        self.speed.setRange(0, 100)
        self.speed.setSuffix(" cm/s")
        self.speed.setValue(int(data.get("speed", 20)))
        self.duration = QtWidgets.QDoubleSpinBox()
        self.duration.setRange(0, 36000)
        self.duration.setDecimals(1)
        self.duration.setSuffix(" s")
        self.duration.setValue(float(data.get("duration", 60)))
        self.ramp = QtWidgets.QDoubleSpinBox()
        self.ramp.setRange(0, 50)
        self.ramp.setDecimals(2)
        self.ramp.setSuffix(" cm/s²")
        self.ramp.setValue(float(data.get("ramp_rate", 3)))
        self.label = QtWidgets.QLineEdit(str(data.get("label", "")))

        form = QtWidgets.QFormLayout()
        form.addRow("Target speed", self.speed)
        form.addRow("Hold duration", self.duration)
        form.addRow("Ramp rate (0 = instant)", self.ramp)
        form.addRow("Label", self.label)
        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        lay = QtWidgets.QVBoxLayout(self)
        lay.addLayout(form)
        lay.addWidget(buttons)

    def result_data(self) -> Dict[str, Any]:
        return {
            "type": "run",
            "speed": self.speed.value(),
            "duration": self.duration.value(),
            "ramp_rate": self.ramp.value(),
            "label": self.label.text().strip(),
        }


class _LoopDialog(QtWidgets.QDialog):
    def __init__(self, parent=None, count: int = 2):
        super().__init__(parent)
        self.setWindowTitle("Repeat group")
        self.count = QtWidgets.QSpinBox()
        self.count.setRange(1, 1000)
        self.count.setValue(int(count))
        form = QtWidgets.QFormLayout()
        form.addRow("Repeat count", self.count)
        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        lay = QtWidgets.QVBoxLayout(self)
        lay.addLayout(form)
        lay.addWidget(buttons)


# --------------------------
# Builder dialog
# --------------------------
class RegimenBuilderDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Create / Edit HIIT Regimen")
        self.resize(720, 560)
        self.saved_path: Optional[str] = None
        self.load_after = False

        # header fields
        self.name = QtWidgets.QLineEdit("New Regimen")
        self.date = QtWidgets.QLineEdit(_date.today().isoformat())
        self.description = QtWidgets.QPlainTextEdit()
        self.description.setMaximumHeight(54)
        self.default_ramp = QtWidgets.QDoubleSpinBox()
        self.default_ramp.setRange(0, 50)
        self.default_ramp.setValue(3)
        self.default_ramp.setSuffix(" cm/s²")
        self.seed_target = QtWidgets.QSpinBox(); self.seed_target.setRange(0, 100); self.seed_target.setValue(36)
        self.seed_step = QtWidgets.QSpinBox(); self.seed_step.setRange(1, 50); self.seed_step.setValue(5)
        self.seed_every = QtWidgets.QSpinBox(); self.seed_every.setRange(1, 3600); self.seed_every.setValue(120)

        header = QtWidgets.QFormLayout()
        header.addRow("Protocol name", self.name)
        header.addRow("Date", self.date)
        header.addRow("Description", self.description)
        header.addRow("Default ramp rate", self.default_ramp)
        seed_row = QtWidgets.QHBoxLayout()
        seed_row.addWidget(QtWidgets.QLabel("target")); seed_row.addWidget(self.seed_target)
        seed_row.addWidget(QtWidgets.QLabel("step")); seed_row.addWidget(self.seed_step)
        seed_row.addWidget(QtWidgets.QLabel("every")); seed_row.addWidget(self.seed_every)
        seed_row.addStretch(1)
        header.addRow("Manual-ramp seeds", seed_row)

        # steps tree
        self.tree = QtWidgets.QTreeWidget()
        self.tree.setColumnCount(5)
        self.tree.setHeaderLabels(["Step", "speed", "duration (s)", "ramp (cm/s²)", "label"])
        self.tree.itemDoubleClicked.connect(lambda *_: self._edit_selected())

        add_step_btn = QtWidgets.QPushButton("Add step")
        add_loop_btn = QtWidgets.QPushButton("Add repeat group")
        add_in_loop_btn = QtWidgets.QPushButton("Add step to group")
        edit_btn = QtWidgets.QPushButton("Edit")
        dup_btn = QtWidgets.QPushButton("Duplicate")
        del_btn = QtWidgets.QPushButton("Delete")
        up_btn = QtWidgets.QPushButton("Move ↑")
        down_btn = QtWidgets.QPushButton("Move ↓")
        add_step_btn.clicked.connect(self._add_step)
        add_loop_btn.clicked.connect(self._add_loop)
        add_in_loop_btn.clicked.connect(self._add_step_in_loop)
        edit_btn.clicked.connect(self._edit_selected)
        dup_btn.clicked.connect(self._duplicate_selected)
        del_btn.clicked.connect(self._delete_selected)
        up_btn.clicked.connect(lambda: self._move_selected(-1))
        down_btn.clicked.connect(lambda: self._move_selected(1))
        btn_col = QtWidgets.QVBoxLayout()
        for b in (add_step_btn, add_loop_btn, add_in_loop_btn, edit_btn, dup_btn, del_btn, up_btn, down_btn):
            btn_col.addWidget(b)
        btn_col.addStretch(1)

        tree_row = QtWidgets.QHBoxLayout()
        tree_row.addWidget(self.tree, stretch=1)
        tree_row.addLayout(btn_col)

        # status + actions
        self.status = QtWidgets.QLabel("")
        self.status.setWordWrap(True)
        save_btn = QtWidgets.QPushButton("Save…")
        save_load_btn = QtWidgets.QPushButton("Save && Load")
        cancel_btn = QtWidgets.QPushButton("Cancel")
        save_btn.clicked.connect(lambda: self._save(load_after=False))
        save_load_btn.clicked.connect(lambda: self._save(load_after=True))
        cancel_btn.clicked.connect(self.reject)
        action_row = QtWidgets.QHBoxLayout()
        action_row.addWidget(self.status, stretch=1)
        action_row.addWidget(save_btn)
        action_row.addWidget(save_load_btn)
        action_row.addWidget(cancel_btn)

        lay = QtWidgets.QVBoxLayout(self)
        lay.addLayout(header)
        lay.addWidget(QtWidgets.QLabel("Steps (double-click to edit):"))
        lay.addLayout(tree_row, stretch=1)
        lay.addLayout(action_row)

        self.tree.itemChanged.connect(lambda *_: self._refresh())
        self._refresh()

    # ---- prefill (optional, for "edit existing") ----
    def load_from_protocol_dict(self, data: Dict[str, Any]) -> None:
        self.name.setText(str(data.get("protocol_name", "")))
        if data.get("date"):
            self.date.setText(str(data["date"]))
        self.description.setPlainText(str(data.get("description", "")))
        if (data.get("defaults") or {}).get("ramp_rate_cm_s2") is not None:
            self.default_ramp.setValue(float(data["defaults"]["ramp_rate_cm_s2"]))
        for key, w in (("target", self.seed_target), ("step", self.seed_step), ("every", self.seed_every)):
            if data.get(key) is not None:
                w.setValue(int(data[key]))
        self.tree.clear()
        for step in data.get("steps", []):
            if step.get("type") == "loop":
                loop_item = self._make_loop_item(int(step.get("count", 1)))
                self.tree.addTopLevelItem(loop_item)
                for child in step.get("steps", []):
                    loop_item.addChild(self._make_run_item(child, child=True))
                loop_item.setExpanded(True)
            else:
                self.tree.addTopLevelItem(self._make_run_item(step))
        self._refresh()

    # ---- tree item factories ----
    def _make_run_item(self, d: Dict[str, Any], child: bool = False) -> QtWidgets.QTreeWidgetItem:
        item = QtWidgets.QTreeWidgetItem()
        data = {
            "type": "run",
            "speed": int(d.get("speed", 0)),
            "duration": float(d.get("duration", 0)),
            "ramp_rate": float(d.get("ramp_rate", self.default_ramp.value())),
            "label": str(d.get("label", "")),
        }
        self._apply_run_item(item, data, child)
        return item

    def _apply_run_item(self, item, data, child=False) -> None:
        item.setData(0, _ROLE, data)
        item.setText(0, "↳ Run" if child else "Run")
        item.setText(1, str(data["speed"]))
        item.setText(2, f"{data['duration']:g}")
        item.setText(3, f"{data['ramp_rate']:g}")
        item.setText(4, data["label"])

    def _make_loop_item(self, count: int) -> QtWidgets.QTreeWidgetItem:
        item = QtWidgets.QTreeWidgetItem()
        item.setData(0, _ROLE, {"type": "loop", "count": int(count)})
        item.setText(0, f"Repeat ×{count}")
        return item

    # ---- actions ----
    def _selected(self) -> Optional[QtWidgets.QTreeWidgetItem]:
        items = self.tree.selectedItems()
        return items[0] if items else None

    def _add_step(self) -> None:
        dlg = _RunStepDialog(self)
        if dlg.exec():
            self.tree.addTopLevelItem(self._make_run_item(dlg.result_data()))
            self._refresh()

    def _add_loop(self) -> None:
        dlg = _LoopDialog(self)
        if dlg.exec():
            item = self._make_loop_item(dlg.count.value())
            self.tree.addTopLevelItem(item)
            item.setExpanded(True)
            self._refresh()

    def _loop_target(self, item) -> Optional[QtWidgets.QTreeWidgetItem]:
        if item is None:
            return None
        d = item.data(0, _ROLE) or {}
        if d.get("type") == "loop":
            return item
        parent = item.parent()
        if parent is not None and (parent.data(0, _ROLE) or {}).get("type") == "loop":
            return parent
        return None

    def _add_step_in_loop(self) -> None:
        loop = self._loop_target(self._selected())
        if loop is None:
            self.status.setText("Select a repeat group (or a step inside one) first.")
            return
        dlg = _RunStepDialog(self)
        if dlg.exec():
            loop.addChild(self._make_run_item(dlg.result_data(), child=True))
            loop.setExpanded(True)
            self._refresh()

    def _edit_selected(self) -> None:
        item = self._selected()
        if item is None:
            return
        d = item.data(0, _ROLE) or {}
        if d.get("type") == "loop":
            dlg = _LoopDialog(self, count=d.get("count", 2))
            if dlg.exec():
                item.setData(0, _ROLE, {"type": "loop", "count": dlg.count.value()})
                item.setText(0, f"Repeat ×{dlg.count.value()}")
        else:
            dlg = _RunStepDialog(self, data=d)
            if dlg.exec():
                self._apply_run_item(item, dlg.result_data(), child=item.parent() is not None)
        self._refresh()

    def _duplicate_selected(self) -> None:
        item = self._selected()
        if item is None:
            return
        d = item.data(0, _ROLE) or {}
        if d.get("type") == "loop":
            return  # keep it simple: duplicate run steps only
        clone = self._make_run_item(d, child=item.parent() is not None)
        parent = item.parent()
        if parent is not None:
            parent.insertChild(parent.indexOfChild(item) + 1, clone)
        else:
            self.tree.insertTopLevelItem(self.tree.indexOfTopLevelItem(item) + 1, clone)
        self._refresh()

    def _delete_selected(self) -> None:
        item = self._selected()
        if item is None:
            return
        parent = item.parent()
        if parent is not None:
            parent.removeChild(item)
        else:
            self.tree.takeTopLevelItem(self.tree.indexOfTopLevelItem(item))
        self._refresh()

    def _move_selected(self, delta: int) -> None:
        item = self._selected()
        if item is None:
            return
        parent = item.parent()
        if parent is not None:
            idx = parent.indexOfChild(item)
            new = idx + delta
            if 0 <= new < parent.childCount():
                parent.takeChild(idx)
                parent.insertChild(new, item)
                self.tree.setCurrentItem(item)
        else:
            idx = self.tree.indexOfTopLevelItem(item)
            new = idx + delta
            if 0 <= new < self.tree.topLevelItemCount():
                self.tree.takeTopLevelItem(idx)
                self.tree.insertTopLevelItem(new, item)
                self.tree.setCurrentItem(item)
        self._refresh()

    # ---- assemble / validate / save ----
    def assemble_dict(self) -> Dict[str, Any]:
        steps: List[Dict[str, Any]] = []
        for i in range(self.tree.topLevelItemCount()):
            it = self.tree.topLevelItem(i)
            d = it.data(0, _ROLE) or {}
            if d.get("type") == "loop":
                children = [
                    self._run_payload(it.child(j).data(0, _ROLE) or {})
                    for j in range(it.childCount())
                ]
                steps.append({"type": "loop", "count": int(d.get("count", 1)), "steps": children})
            else:
                steps.append(self._run_payload(d))
        return {
            "protocol_name": self.name.text().strip() or "Untitled",
            "date": self.date.text().strip(),
            "description": self.description.toPlainText().strip(),
            "target": self.seed_target.value(),
            "step": self.seed_step.value(),
            "every": self.seed_every.value(),
            "defaults": {"ramp_rate_cm_s2": self.default_ramp.value()},
            "steps": steps,
        }

    @staticmethod
    def _run_payload(d: Dict[str, Any]) -> Dict[str, Any]:
        out = {"type": "run", "speed": int(d.get("speed", 0)),
               "duration": d.get("duration", 0), "ramp_rate": d.get("ramp_rate", 0)}
        if d.get("label"):
            out["label"] = d["label"]
        return out

    def _refresh(self) -> Optional[protocol_mod.HiitProtocol]:
        try:
            proto = protocol_mod.protocol_from_dict(self.assemble_dict())
        except Exception as exc:
            self.status.setText(f"⚠ {exc}")
            self.status.setStyleSheet("color:#b00020;")
            return None
        mins = proto.estimated_total_s / 60.0
        self.status.setText(f"✓ valid — {len(proto.stages)} stages, ~{proto.estimated_total_s:.0f}s ({mins:.1f} min)")
        self.status.setStyleSheet("color:#137333;")
        return proto

    def _save(self, load_after: bool) -> None:
        if self._refresh() is None:
            QtWidgets.QMessageBox.warning(self, "Invalid regimen", "Fix the highlighted error before saving.")
            return
        default_dir = hsettings.get_regimen_dir()
        try:
            default_dir.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass
        safe = "".join(c if c.isalnum() or c in "-_." else "_" for c in self.name.text().strip()) or "regimen"
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save regimen", str(default_dir / f"{safe}.yaml"),
            "YAML protocols (*.yaml *.yml)",
        )
        if not path:
            return
        data = self.assemble_dict()
        header = f"{data['protocol_name']}\nGenerated by camera_control HIIT builder\nDate: {data['date']}"
        try:
            Path(path).write_text(protocol_mod.to_yaml_document(data, header), encoding="utf-8")
        except Exception as exc:
            QtWidgets.QMessageBox.critical(self, "Save failed", str(exc))
            return
        hsettings.set_regimen_dir(Path(path).parent)
        self.saved_path = path
        self.load_after = load_after
        self.accept()
