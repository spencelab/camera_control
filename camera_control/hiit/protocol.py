# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""HIIT protocol schema, validation, and loader.

Pure Python + PyYAML. No rclpy, no Qt — importable and unit-testable on an MBP
without ROS 2. Mirrors the config conventions used by ``camera_control``'s rig
loader (PyYAML ``safe_load`` -> frozen dataclasses -> ``_..._from_dict`` validator
that raises descriptive ``ValueError``s, with a graceful default path).

Schema is 1:1 with ``configs/hiit_protocols/example_hiit.yaml``.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# --------------------------
# Constants
# --------------------------
MIN_SPEED = 0
MAX_SPEED = 100              # cm/s — matches the Treadmill tab spinbox/clamp ceiling
MAX_EXPANDED_STAGES = 10000  # guard against pathological loop counts
MAX_NESTING_DEPTH = 10       # guard against pathological loop nesting

_VALID_STEP_TYPES = ("run", "loop")


# --------------------------
# Dataclasses
# --------------------------
@dataclass(frozen=True)
class ResolvedStage:
    """A single, fully-resolved run phase after loop expansion.

    ``ramp_rate`` is in cm/s^2 (cm/s per second); 0 means an instant jump.
    ``duration`` is the hold time AT ``speed`` and excludes ramp time.
    """

    speed: int
    duration: float
    ramp_rate: float
    label: str = ""
    loop_path: str = ""  # provenance, e.g. "steps[7]/loop/iter2/steps[0]"


@dataclass(frozen=True)
class HiitProtocol:
    protocol_name: str
    date: str = ""
    description: str = ""
    # Seeds for the (deferred) manual Ramp Protocol spinboxes. Parsed and stored
    # but ignored by the phased runner.
    seed_target: Optional[int] = None
    seed_step: Optional[int] = None
    seed_every: Optional[int] = None
    default_ramp_rate: Optional[float] = None  # defaults.ramp_rate_cm_s2
    stages: Tuple[ResolvedStage, ...] = field(default_factory=tuple)
    estimated_total_s: float = 0.0
    source_path: str = ""


# --------------------------
# Small typed validators (descriptive errors, path-aware)
# --------------------------
def _is_bool(value: Any) -> bool:
    # In Python, bool is a subclass of int; YAML true/false should not pass as numbers.
    return isinstance(value, bool)


def _as_int(value: Any, where: str, field_name: str) -> int:
    """Accept an int or an integral float; reject bool, non-numeric, fractional."""
    if _is_bool(value):
        raise ValueError(f"{where}: '{field_name}' must be an integer, got boolean {value!r}")
    if isinstance(value, int):
        return value
    if isinstance(value, float) and value.is_integer():
        return int(value)
    raise ValueError(f"{where}: '{field_name}' must be an integer, got {value!r}")


def _as_number(value: Any, where: str, field_name: str) -> float:
    if _is_bool(value):
        raise ValueError(f"{where}: '{field_name}' must be a number, got boolean {value!r}")
    if isinstance(value, (int, float)):
        return float(value)
    raise ValueError(f"{where}: '{field_name}' must be a number, got {value!r}")


def _ramp_time(prev_speed: int, target_speed: int, ramp_rate: float) -> float:
    """Seconds to ramp from prev to target. 0 if instant (rate 0) or no change."""
    if ramp_rate == 0 or target_speed == prev_speed:
        return 0.0
    return abs(target_speed - prev_speed) / ramp_rate


# --------------------------
# Parsing / expansion
# --------------------------
def _parse_run_step(step: Dict[str, Any], where: str, default_ramp_rate: Optional[float]) -> ResolvedStage:
    if "speed" not in step:
        raise ValueError(f"{where}: run step missing required 'speed'")
    speed = _as_int(step["speed"], where, "speed")
    if not (MIN_SPEED <= speed <= MAX_SPEED):
        raise ValueError(
            f"{where}: 'speed' {speed} out of range [{MIN_SPEED}, {MAX_SPEED}] cm/s"
        )

    if "duration" not in step:
        raise ValueError(f"{where}: run step missing required 'duration'")
    duration = _as_number(step["duration"], where, "duration")
    if duration < 0:
        raise ValueError(f"{where}: 'duration' must be >= 0, got {duration}")

    if "ramp_rate" in step and step["ramp_rate"] is not None:
        ramp_rate = _as_number(step["ramp_rate"], where, "ramp_rate")
    elif default_ramp_rate is not None:
        ramp_rate = default_ramp_rate
    else:
        raise ValueError(
            f"{where}: run step omits 'ramp_rate' and no 'defaults.ramp_rate_cm_s2' is set"
        )
    if ramp_rate < 0:
        raise ValueError(f"{where}: 'ramp_rate' must be >= 0, got {ramp_rate}")

    label = str(step.get("label", "")) if step.get("label") is not None else ""
    return ResolvedStage(
        speed=speed,
        duration=duration,
        ramp_rate=ramp_rate,
        label=label,
        loop_path=where,
    )


def _expand_steps(
    steps: Any,
    default_ramp_rate: Optional[float],
    depth: int,
    path_prefix: str,
    out: List[ResolvedStage],
) -> None:
    if depth > MAX_NESTING_DEPTH:
        raise ValueError(
            f"{path_prefix}: loop nesting exceeds MAX_NESTING_DEPTH={MAX_NESTING_DEPTH}"
        )
    if not isinstance(steps, list):
        raise ValueError(f"{path_prefix or 'steps'}: 'steps' must be a list")
    if len(steps) == 0:
        raise ValueError(f"{path_prefix or 'steps'}: 'steps' must not be empty")

    for i, step in enumerate(steps):
        where = f"{path_prefix}steps[{i}]"
        if not isinstance(step, dict):
            raise ValueError(f"{where}: each step must be a mapping")
        step_type = step.get("type")
        if step_type not in _VALID_STEP_TYPES:
            raise ValueError(
                f"{where}: 'type' must be one of {_VALID_STEP_TYPES}, got {step_type!r}"
            )

        if step_type == "run":
            out.append(_parse_run_step(step, where, default_ramp_rate))
        else:  # loop
            if "count" not in step:
                raise ValueError(f"{where}: loop step missing required 'count'")
            count = _as_int(step["count"], where, "count")
            if count < 1:
                raise ValueError(f"{where}: loop 'count' must be >= 1, got {count}")
            if "steps" not in step:
                raise ValueError(f"{where}: loop step missing required 'steps'")
            for it in range(count):
                _expand_steps(
                    step["steps"],
                    default_ramp_rate,
                    depth + 1,
                    f"{where}/loop/iter{it}/",
                    out,
                )

        if len(out) > MAX_EXPANDED_STAGES:
            raise ValueError(
                f"protocol expands to more than MAX_EXPANDED_STAGES={MAX_EXPANDED_STAGES} stages"
            )


def _estimated_total_s(stages: Tuple[ResolvedStage, ...]) -> float:
    """Total run time assuming the belt starts at 0 cm/s (matches template intent)."""
    total = 0.0
    prev = 0
    for st in stages:
        total += _ramp_time(prev, st.speed, st.ramp_rate) + st.duration
        prev = st.speed
    return total


def _opt_int(data: Dict[str, Any], key: str) -> Optional[int]:
    if data.get(key) is None:
        return None
    return _as_int(data[key], "top level", key)


def protocol_from_dict(data: Any, source_path: str = "") -> HiitProtocol:
    """Validate a parsed YAML mapping and return a flattened HiitProtocol.

    Raises ValueError (with a path to the offending element) on any schema error.
    """
    if not isinstance(data, dict):
        raise ValueError("protocol file must be a YAML mapping at the top level")

    name = data.get("protocol_name")
    if not name or not str(name).strip():
        raise ValueError("missing required 'protocol_name'")
    protocol_name = str(name).strip()

    date = str(data["date"]) if data.get("date") is not None else ""
    description = str(data["description"]) if data.get("description") is not None else ""

    defaults = data.get("defaults") or {}
    if not isinstance(defaults, dict):
        raise ValueError("'defaults' must be a mapping")
    default_ramp_rate: Optional[float] = None
    if defaults.get("ramp_rate_cm_s2") is not None:
        default_ramp_rate = _as_number(
            defaults["ramp_rate_cm_s2"], "defaults", "ramp_rate_cm_s2"
        )
        if default_ramp_rate < 0:
            raise ValueError(
                f"defaults.ramp_rate_cm_s2 must be >= 0, got {default_ramp_rate}"
            )

    if "steps" not in data:
        raise ValueError("missing required 'steps'")
    stages_list: List[ResolvedStage] = []
    _expand_steps(data["steps"], default_ramp_rate, depth=0, path_prefix="", out=stages_list)
    stages = tuple(stages_list)

    return HiitProtocol(
        protocol_name=protocol_name,
        date=date,
        description=description,
        seed_target=_opt_int(data, "target"),
        seed_step=_opt_int(data, "step"),
        seed_every=_opt_int(data, "every"),
        default_ramp_rate=default_ramp_rate,
        stages=stages,
        estimated_total_s=_estimated_total_s(stages),
        source_path=source_path,
    )


# --------------------------
# Loading
# --------------------------
def default_hiit_dir() -> Path:
    """Directory holding shipped/importable protocols.

    Override with CAMERA_CONTROL_HIIT_DIR. Default: <repo>/configs/hiit_protocols.
    """
    env = os.environ.get("CAMERA_CONTROL_HIIT_DIR")
    if env:
        return Path(env).expanduser()
    # this file: <repo>/camera_control/hiit/protocol.py -> parents[2] == <repo>
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root / "configs" / "hiit_protocols"


def load_protocol(path: os.PathLike | str) -> HiitProtocol:
    """Read and validate a protocol YAML file. Raises on any error.

    Callers at the UI boundary should catch and surface errors gracefully rather
    than letting them crash the application.
    """
    import yaml  # type: ignore

    p = Path(path).expanduser()
    text = p.read_text(encoding="utf-8")
    data = yaml.safe_load(text)
    return protocol_from_dict(data, source_path=str(p))


def list_protocols(directory: Optional[os.PathLike | str] = None) -> List[Path]:
    """List *.yaml / *.yml protocol files in a directory (non-recursive)."""
    d = Path(directory).expanduser() if directory is not None else default_hiit_dir()
    if not d.is_dir():
        return []
    return sorted(p for p in d.iterdir() if p.suffix.lower() in (".yaml", ".yml"))
