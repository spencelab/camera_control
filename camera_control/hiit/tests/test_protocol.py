# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""Unit tests for hiit.protocol — runnable on an MBP without ROS 2 or Qt."""

from pathlib import Path

import pytest

from hiit import protocol
from hiit.protocol import (
    HiitProtocol,
    MAX_NESTING_DEPTH,
    MAX_SPEED,
    ResolvedStage,
    load_protocol,
    protocol_from_dict,
)

EXAMPLE = (
    Path(__file__).resolve().parents[3]
    / "configs"
    / "hiit_protocols"
    / "example_hiit.yaml"
)


# --------------------------
# Happy path — the shipped template
# --------------------------
def test_example_file_exists():
    assert EXAMPLE.is_file(), f"missing template at {EXAMPLE}"


def test_load_example():
    p = load_protocol(EXAMPLE)
    assert isinstance(p, HiitProtocol)
    assert p.protocol_name == "HIIT_ProgressiveSprints"
    assert p.date == "2026-06-02"          # YAML date coerced to str
    assert p.default_ramp_rate == 3        # defaults.ramp_rate_cm_s2
    assert p.source_path == str(EXAMPLE)


def test_example_seeds_parsed_but_separate():
    p = load_protocol(EXAMPLE)
    assert p.seed_target == 36
    assert p.seed_step == 5
    assert p.seed_every == 120


def test_example_stage_count_after_loop_expansion():
    # 2 (warmup/build) + 6 (sprint/recovery x3) + 3*2 (loop block) + 2 (cooldown/stop)
    p = load_protocol(EXAMPLE)
    assert len(p.stages) == 16


def test_example_first_and_last_stages():
    p = load_protocol(EXAMPLE)
    first = p.stages[0]
    assert (first.speed, first.duration, first.ramp_rate, first.label) == (16, 90, 2, "warm-up")
    last = p.stages[-1]
    assert (last.speed, last.duration, last.ramp_rate, last.label) == (0, 0, 2, "stop")


def test_example_loop_provenance():
    p = load_protocol(EXAMPLE)
    block = [s for s in p.stages if "loop" in s.loop_path]
    assert len(block) == 6  # 3 iterations x 2 steps
    assert "iter0" in block[0].loop_path and "iter2" in block[-1].loop_path


# --------------------------
# Ramp-time / estimated total (exact, deterministic)
# --------------------------
def test_estimated_total_exact():
    data = {
        "protocol_name": "t",
        "steps": [
            {"type": "run", "speed": 10, "duration": 5, "ramp_rate": 2},   # ramp 10/2=5 +5 = 10
            {"type": "run", "speed": 10, "duration": 3, "ramp_rate": 0},   # same speed -> +3 = 13
            {"type": "run", "speed": 0, "duration": 0, "ramp_rate": 5},    # ramp 10/5=2 +0 = 15
        ],
    }
    p = protocol_from_dict(data)
    assert p.estimated_total_s == pytest.approx(15.0)


def test_default_ramp_rate_fallback():
    data = {
        "protocol_name": "t",
        "defaults": {"ramp_rate_cm_s2": 4},
        "steps": [{"type": "run", "speed": 20, "duration": 1}],  # omits ramp_rate
    }
    p = protocol_from_dict(data)
    assert p.stages[0].ramp_rate == 4


def test_nested_loop_expansion():
    data = {
        "protocol_name": "t",
        "defaults": {"ramp_rate_cm_s2": 1},
        "steps": [
            {
                "type": "loop",
                "count": 2,
                "steps": [
                    {"type": "loop", "count": 3, "steps": [{"type": "run", "speed": 5, "duration": 1}]}
                ],
            }
        ],
    }
    p = protocol_from_dict(data)
    assert len(p.stages) == 6  # 2 * 3 * 1


# --------------------------
# Validation errors (descriptive ValueErrors)
# --------------------------
def _minimal(steps, **top):
    base = {"protocol_name": "t", "defaults": {"ramp_rate_cm_s2": 1}}
    base.update(top)
    base["steps"] = steps
    return base


def test_reject_top_level_not_mapping():
    with pytest.raises(ValueError, match="mapping at the top level"):
        protocol_from_dict([1, 2, 3])


def test_reject_missing_protocol_name():
    with pytest.raises(ValueError, match="protocol_name"):
        protocol_from_dict({"steps": [{"type": "run", "speed": 1, "duration": 1, "ramp_rate": 1}]})


def test_reject_missing_steps():
    with pytest.raises(ValueError, match="missing required 'steps'"):
        protocol_from_dict({"protocol_name": "t"})


def test_reject_empty_steps():
    with pytest.raises(ValueError, match="must not be empty"):
        protocol_from_dict({"protocol_name": "t", "steps": []})


def test_reject_speed_over_max():
    with pytest.raises(ValueError, match="out of range"):
        protocol_from_dict(_minimal([{"type": "run", "speed": MAX_SPEED + 1, "duration": 1, "ramp_rate": 1}]))


def test_reject_negative_speed():
    with pytest.raises(ValueError, match="out of range"):
        protocol_from_dict(_minimal([{"type": "run", "speed": -1, "duration": 1, "ramp_rate": 1}]))


def test_reject_missing_speed():
    with pytest.raises(ValueError, match="missing required 'speed'"):
        protocol_from_dict(_minimal([{"type": "run", "duration": 1, "ramp_rate": 1}]))


def test_reject_missing_duration():
    with pytest.raises(ValueError, match="missing required 'duration'"):
        protocol_from_dict(_minimal([{"type": "run", "speed": 10, "ramp_rate": 1}]))


def test_reject_negative_duration():
    with pytest.raises(ValueError, match="duration"):
        protocol_from_dict(_minimal([{"type": "run", "speed": 10, "duration": -2, "ramp_rate": 1}]))


def test_reject_unknown_type():
    with pytest.raises(ValueError, match="'type' must be one of"):
        protocol_from_dict(_minimal([{"type": "sprint", "speed": 10, "duration": 1, "ramp_rate": 1}]))


def test_reject_ramp_rate_omitted_without_defaults():
    with pytest.raises(ValueError, match="no 'defaults.ramp_rate_cm_s2'"):
        protocol_from_dict({"protocol_name": "t", "steps": [{"type": "run", "speed": 10, "duration": 1}]})


def test_reject_negative_ramp_rate():
    with pytest.raises(ValueError, match="ramp_rate' must be >= 0"):
        protocol_from_dict(_minimal([{"type": "run", "speed": 10, "duration": 1, "ramp_rate": -1}]))


def test_reject_loop_count_zero():
    with pytest.raises(ValueError, match="count' must be >= 1"):
        protocol_from_dict(_minimal([{"type": "loop", "count": 0, "steps": [{"type": "run", "speed": 1, "duration": 1, "ramp_rate": 1}]}]))


def test_reject_loop_missing_steps():
    with pytest.raises(ValueError, match="missing required 'steps'"):
        protocol_from_dict(_minimal([{"type": "loop", "count": 2}]))


def test_reject_bool_as_speed():
    with pytest.raises(ValueError, match="must be an integer, got boolean"):
        protocol_from_dict(_minimal([{"type": "run", "speed": True, "duration": 1, "ramp_rate": 1}]))


def test_reject_nesting_too_deep():
    # build a loop nested deeper than MAX_NESTING_DEPTH
    inner = {"type": "run", "speed": 1, "duration": 1, "ramp_rate": 1}
    node = inner
    for _ in range(MAX_NESTING_DEPTH + 2):
        node = {"type": "loop", "count": 1, "steps": [node]}
    with pytest.raises(ValueError, match="MAX_NESTING_DEPTH"):
        protocol_from_dict(_minimal([node]))


def test_error_path_names_offending_step():
    with pytest.raises(ValueError, match=r"steps\[1\]"):
        protocol_from_dict(_minimal([
            {"type": "run", "speed": 10, "duration": 1, "ramp_rate": 1},
            {"type": "run", "speed": 999, "duration": 1, "ramp_rate": 1},
        ]))


# --------------------------
# Misc API
# --------------------------
def test_list_protocols_finds_example():
    found = protocol.list_protocols(EXAMPLE.parent)
    assert EXAMPLE in found


def test_resolved_stage_is_frozen():
    s = ResolvedStage(speed=1, duration=1, ramp_rate=1)
    with pytest.raises(Exception):
        s.speed = 2  # type: ignore
