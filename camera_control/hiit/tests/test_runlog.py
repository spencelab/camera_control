# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""Unit tests for hiit.runlog — pure Python, no ROS/Qt."""

from datetime import datetime, timedelta

import yaml

from hiit.protocol import ResolvedStage
from hiit.runlog import RunLog, default_run_log_dir


def W(secs):
    return datetime(2026, 6, 8, 16, 0, 0) + timedelta(seconds=secs)


def test_records_and_closes_stages():
    rl = RunLog("P", "/x.yaml", 30.0)
    rl.start(W(0), 0.0)
    rl.stage_started(0, ResolvedStage(10, 5, 2, "warm", "steps[0]"), W(0), 0.0)
    rl.stage_started(1, ResolvedStage(20, 5, 1, "go", "steps[1]"), W(7), 7.0)
    rl.finish("complete", W(20), 20.0)

    d = rl.to_dict()["hiit_run"]
    assert d["stage_count"] == 2
    assert d["outcome"] == "complete"
    assert d["actual_total_s"] == 20.0
    s0, s1 = d["stages"]
    assert s0["duration_s"] == 7.0     # closed at next stage start
    assert s1["duration_s"] == 13.0    # closed at finish
    assert s0["target_speed_cm_s"] == 10
    assert s1["label"] == "go"
    assert s1["loop_path"] == "steps[1]"


def test_incomplete_outcome_default():
    rl = RunLog("P", "", 1.0)
    assert rl.to_dict()["hiit_run"]["outcome"] == "incomplete"
    assert rl.actual_total_s() is None


def test_write_roundtrip(tmp_path):
    rl = RunLog("My Proto", "/p.yaml", 10.0)
    rl.start(W(0), 0.0)
    rl.stage_started(0, ResolvedStage(5, 1, 5, "a", "steps[0]"), W(0), 0.0)
    rl.finish("aborted", W(3), 3.0)

    p = rl.write(tmp_path)
    assert p.exists()
    assert p.name.startswith("hiit_run_") and p.suffix == ".yaml"
    data = yaml.safe_load(p.read_text(encoding="utf-8"))["hiit_run"]
    assert data["outcome"] == "aborted"
    assert data["actual_total_s"] == 3.0
    assert data["stages"][0]["label"] == "a"


def test_default_dir_env(monkeypatch, tmp_path):
    monkeypatch.setenv("CAMERA_CONTROL_HIIT_RUN_DIR", str(tmp_path / "runs"))
    assert default_run_log_dir() == (tmp_path / "runs")


def test_filename_safe(tmp_path):
    rl = RunLog("weird/name:with*chars", "", 1.0)
    rl.start(W(0), 0.0)
    rl.finish("complete", W(1), 1.0)
    p = rl.write(tmp_path)
    assert "/" not in p.name and ":" not in p.name and "*" not in p.name
