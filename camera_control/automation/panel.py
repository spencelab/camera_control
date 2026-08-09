# Author: Andrew England (andrewengland19)
"""AutoRunPanel — the dedicated, high-contrast "Auto-Run" tab.

A deliberately uncluttered control surface built for fast, repeated trials without
tab-switching:

  * a large colour-coded STATUS BANNER (Ready -> Recording -> Protocol Active ->
    Complete),
  * one primary button that is START AUTO-RUN when idle and ABORT while running,
  * compact rat ID + condition quick-entry (mirrored into the real Metadata tab),
  * a readiness line (loaded regimen + selected-camera count) and a live progress
    line, so the operator sees everything the run needs on one screen,
  * a Mute voice checkbox.

PySide6 only; no rclpy. All behaviour is delegated to the injected
``AutoRunController``.
"""

from __future__ import annotations

from typing import Optional

from PySide6 import QtCore, QtWidgets

from . import controller as autoctl


# Banner styles per state group. High contrast, readable in the lab.
def _banner_style(bg: str, fg: str = "#ffffff") -> str:
    return (
        f"QLabel {{ background: {bg}; color: {fg}; border-radius: 8px;"
        " padding: 18px; font-size: 26px; font-weight: 800; }"
    )


_STATE_BANNER = {
    autoctl.READY: _banner_style("#37474f"),          # slate
    autoctl.RESETTING: _banner_style("#5c6bc0"),      # indigo
    autoctl.METADATA: _banner_style("#5c6bc0"),
    autoctl.STARTING_REC: _banner_style("#c62828"),   # red — recording imminent
    autoctl.PROTOCOL: _banner_style("#ef6c00"),       # amber — protocol active
    autoctl.STOPPING_REC: _banner_style("#c62828"),
    autoctl.COMPLETE: _banner_style("#2e7d32"),        # green
    autoctl.ERROR: _banner_style("#b00020"),           # error red
}

_PRIMARY_START_STYLE = (
    "QPushButton { background: #2e7d32; color: white; font-size: 20px;"
    " font-weight: 800; padding: 16px; border-radius: 8px; }"
    "QPushButton:hover { background: #1b5e20; }"
    "QPushButton:disabled { background: #9e9e9e; color: #eeeeee; }"
)
_PRIMARY_ABORT_STYLE = (
    "QPushButton { background: #b00020; color: white; font-size: 20px;"
    " font-weight: 800; padding: 16px; border-radius: 8px; }"
    "QPushButton:hover { background: #7f0016; }"
)


class AutoRunPanel(QtWidgets.QWidget):
    """The Auto-Run tab widget. Delegates all logic to AutoRunController."""

    def __init__(self, controller: Optional[autoctl.AutoRunController] = None,
                 parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self._controller: Optional[autoctl.AutoRunController] = None

        # --- big status banner ---
        self.banner = QtWidgets.QLabel(autoctl.STATE_LABELS[autoctl.READY])
        self.banner.setMinimumHeight(80)
        self.banner.setAlignment(QtCore.Qt.AlignCenter)
        self.banner.setStyleSheet(_STATE_BANNER[autoctl.READY])

        # --- readiness + progress lines ---
        self.readiness_label = QtWidgets.QLabel("—")
        self.readiness_label.setWordWrap(True)
        self.progress_label = QtWidgets.QLabel("Idle.")
        self.progress_label.setWordWrap(True)
        self.progress_label.setStyleSheet("color: #444; font-style: italic;")

        # --- quick-entry metadata ---
        self.animal_id = QtWidgets.QLineEdit()
        self.animal_id.setPlaceholderText("e.g. rat042")
        self.condition = QtWidgets.QLineEdit()
        self.condition.setPlaceholderText("e.g. baseline / post-injury")
        quick = QtWidgets.QFormLayout()
        quick.addRow("Animal ID", self.animal_id)
        quick.addRow("Condition", self.condition)
        quick_box = QtWidgets.QGroupBox("Trial metadata (quick entry)")
        quick_box.setLayout(quick)

        # --- primary button + mute ---
        self.primary_btn = QtWidgets.QPushButton("▶  START AUTO-RUN")
        self.primary_btn.setMinimumHeight(64)
        self.primary_btn.setStyleSheet(_PRIMARY_START_STYLE)
        self.primary_btn.clicked.connect(self._on_primary_clicked)

        self.mute_chk = QtWidgets.QCheckBox("Mute voice")
        self.mute_chk.toggled.connect(self._on_mute_toggled)

        self.refresh_btn = QtWidgets.QPushButton("↻ Refresh readiness")
        self.refresh_btn.clicked.connect(self.refresh_readiness)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addWidget(self.mute_chk)
        btn_row.addStretch(1)
        btn_row.addWidget(self.refresh_btn)

        help_label = QtWidgets.QLabel(
            "One click runs the whole trial: reset protocol → confirm metadata → "
            "start recording (awaits confirmation) → run HIIT → stop recording at "
            "0 cm/s. Load a regimen and select cameras on the other tabs first."
        )
        help_label.setWordWrap(True)
        help_label.setStyleSheet("color: #666;")

        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)
        layout.addWidget(self.banner)
        layout.addWidget(self.readiness_label)
        layout.addWidget(quick_box)
        layout.addWidget(self.primary_btn)
        layout.addLayout(btn_row)
        layout.addWidget(self.progress_label)
        layout.addWidget(help_label)
        layout.addStretch(1)
        self.setLayout(layout)

        if controller is not None:
            self.set_controller(controller)

    # --------------------------------------------------------------- wiring
    def set_controller(self, controller: autoctl.AutoRunController) -> None:
        self._controller = controller
        controller.state_changed.connect(self._on_state_changed)
        controller.message.connect(self._on_message)
        # Reflect the current mute state of the speaker, if any.
        spk = getattr(controller, "speaker", None)
        if spk is not None:
            self.mute_chk.setChecked(bool(getattr(spk, "muted", False)))
        self.refresh_readiness()

    # --------------------------------------------------------------- actions
    def _on_primary_clicked(self) -> None:
        if self._controller is None:
            return
        if self._controller.is_active():
            self._controller.abort()
            return
        self._controller.start_auto_run(
            animal_id=self.animal_id.text(),
            condition=self.condition.text(),
        )

    def _on_mute_toggled(self, muted: bool) -> None:
        if self._controller is None:
            return
        spk = getattr(self._controller, "speaker", None)
        if spk is not None:
            spk.muted = bool(muted)

    def refresh_readiness(self) -> None:
        if self._controller is None:
            self.readiness_label.setText("Controller unavailable.")
            self.primary_btn.setEnabled(False)
            return
        rd = self._controller.readiness()
        regimen = rd.regimen_name or "— none —"
        cams = rd.camera_count
        ok = rd.ready
        mark = "✅" if ok else "⚠️"
        self.readiness_label.setText(
            f"{mark}  Regimen: <b>{regimen}</b>  |  Cameras selected: <b>{cams}</b>"
        )
        self.readiness_label.setTextFormat(QtCore.Qt.RichText)
        # Only gate the button when idle; while running it's the ABORT control.
        if not self._controller.is_active():
            self.primary_btn.setEnabled(ok)

    # -------------------------------------------------------------- callbacks
    def _on_state_changed(self, state: str, label: str) -> None:
        self.banner.setText(label)
        self.banner.setStyleSheet(_STATE_BANNER.get(state, _STATE_BANNER[autoctl.READY]))
        active = self._controller is not None and self._controller.is_active()
        if active:
            self.primary_btn.setText("■  ABORT")
            self.primary_btn.setStyleSheet(_PRIMARY_ABORT_STYLE)
            self.primary_btn.setEnabled(True)
            self._set_quick_enabled(False)
        else:
            self.primary_btn.setText("▶  START AUTO-RUN")
            self.primary_btn.setStyleSheet(_PRIMARY_START_STYLE)
            self._set_quick_enabled(True)
            self.refresh_readiness()

    def _set_quick_enabled(self, enabled: bool) -> None:
        self.animal_id.setEnabled(enabled)
        self.condition.setEnabled(enabled)

    def _on_message(self, text: str) -> None:
        self.progress_label.setText(text)


# Standalone smoke run: `python3 -m automation.panel` with a stub controller.
if __name__ == "__main__":  # pragma: no cover
    import sys

    app = QtWidgets.QApplication(sys.argv)
    w = AutoRunPanel()
    w.resize(520, 520)
    w.show()
    sys.exit(app.exec())
