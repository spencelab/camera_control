# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""HIIT run-log: a per-stage timeline of an executed protocol.

Pure Python + PyYAML. Records, for each stage actually entered, the wall-clock
and monotonic start/end times plus the stage's commanded target — so gait /
stride data captured during a run can be aligned to protocol stages offline
(the "gait at indexed speeds" goal). Writes a single YAML file; it does NOT
touch the camera recording path or any ROS interface.

Timestamps are supplied by the caller: wall (datetime, for chrony-aligned
comparison with camera frames) and monotonic (float, drift-free durations).
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional


def default_run_log_dir() -> Path:
    """Where run-logs are written. Override with CAMERA_CONTROL_HIIT_RUN_DIR."""
    env = os.environ.get("CAMERA_CONTROL_HIIT_RUN_DIR")
    if env:
        return Path(env).expanduser()
    return Path.home() / "hiit_runs"


def _safe(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "-", str(text).strip()) or "protocol"


@dataclass
class StageRecord:
    index: int
    label: str
    target_speed_cm_s: int
    ramp_rate_cm_s2: float
    loop_path: str
    start_wall: str
    start_mono: float
    end_wall: str = ""
    end_mono: Optional[float] = None
    duration_s: Optional[float] = None


class RunLog:
    def __init__(self, protocol_name: str, protocol_source: str, estimated_total_s: float):
        self.protocol_name = protocol_name
        self.protocol_source = protocol_source
        self.estimated_total_s = estimated_total_s
        self.started_wall: str = ""
        self.started_mono: Optional[float] = None
        self.finished_wall: str = ""
        self.finished_mono: Optional[float] = None
        self.outcome: str = "incomplete"
        self._stages: List[StageRecord] = []

    # -------- recording --------
    def start(self, wall: datetime, mono: float) -> None:
        self.started_wall = wall.isoformat()
        self.started_mono = mono

    def stage_started(self, index: int, stage, wall: datetime, mono: float) -> None:
        self._close_last(wall, mono)
        self._stages.append(
            StageRecord(
                index=index,
                label=getattr(stage, "label", ""),
                target_speed_cm_s=int(getattr(stage, "speed", 0)),
                ramp_rate_cm_s2=float(getattr(stage, "ramp_rate", 0.0)),
                loop_path=getattr(stage, "loop_path", ""),
                start_wall=wall.isoformat(),
                start_mono=mono,
            )
        )

    def finish(self, outcome: str, wall: datetime, mono: float) -> None:
        self._close_last(wall, mono)
        self.outcome = outcome
        self.finished_wall = wall.isoformat()
        self.finished_mono = mono

    def _close_last(self, wall: datetime, mono: float) -> None:
        if not self._stages:
            return
        last = self._stages[-1]
        if last.end_mono is None:
            last.end_wall = wall.isoformat()
            last.end_mono = mono
            last.duration_s = round(mono - last.start_mono, 4)

    # -------- output --------
    def actual_total_s(self) -> Optional[float]:
        if self.started_mono is not None and self.finished_mono is not None:
            return round(self.finished_mono - self.started_mono, 4)
        return None

    def to_dict(self) -> dict:
        return {
            "hiit_run": {
                "protocol_name": self.protocol_name,
                "protocol_source": self.protocol_source,
                "outcome": self.outcome,
                "started_wall": self.started_wall,
                "finished_wall": self.finished_wall,
                "estimated_total_s": self.estimated_total_s,
                "actual_total_s": self.actual_total_s(),
                "stage_count": len(self._stages),
                "stages": [
                    {
                        "index": s.index,
                        "label": s.label,
                        "target_speed_cm_s": s.target_speed_cm_s,
                        "ramp_rate_cm_s2": s.ramp_rate_cm_s2,
                        "loop_path": s.loop_path,
                        "start_wall": s.start_wall,
                        "end_wall": s.end_wall,
                        "start_mono": s.start_mono,
                        "end_mono": s.end_mono,
                        "duration_s": s.duration_s,
                    }
                    for s in self._stages
                ],
            }
        }

    def to_yaml(self) -> str:
        import yaml  # type: ignore

        return yaml.safe_dump(self.to_dict(), sort_keys=False, default_flow_style=False)

    def suggested_filename(self) -> str:
        stamp = (self.started_wall or datetime.now().isoformat())[:19].replace(":", "").replace("-", "").replace("T", "_")
        return f"hiit_run_{stamp}_{_safe(self.protocol_name)}.yaml"

    def write(self, directory: Optional[os.PathLike | str] = None) -> Path:
        d = Path(directory).expanduser() if directory is not None else default_run_log_dir()
        d.mkdir(parents=True, exist_ok=True)
        path = d / self.suggested_filename()
        path.write_text(self.to_yaml(), encoding="utf-8")
        return path
