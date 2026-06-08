# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""Unit tests for hiit.runner — fake-clock driven, no ROS/Qt."""

import pytest

from hiit.protocol import protocol_from_dict
from hiit.runner import HiitRunner, HiitState


class FakeClock:
    def __init__(self, t=0.0):
        self.t = float(t)

    def __call__(self):
        return self.t

    def set(self, t):
        self.t = float(t)


class Harness:
    """Records every sink interaction for assertions."""

    def __init__(self):
        self.speeds = []          # every set_speed value
        self.take = 0
        self.run = 0
        self.stop = 0
        self.release = 0
        self.states = []          # (old, new)

    def set_speed(self, v):
        self.speeds.append(v)

    def make(self, data, clock, **kw):
        proto = protocol_from_dict(data)
        return HiitRunner(
            proto,
            set_speed=self.set_speed,
            take_control=lambda: setattr(self, "take", self.take + 1),
            run_belt=lambda: setattr(self, "run", self.run + 1),
            stop_belt=lambda: setattr(self, "stop", self.stop + 1),
            release_control=lambda: setattr(self, "release", self.release + 1),
            on_state_change=lambda o, n: self.states.append((o, n)),
            clock=clock,
            **kw,
        )


def _ramp_proto(speed, duration, ramp_rate):
    return {
        "protocol_name": "t",
        "steps": [{"type": "run", "speed": speed, "duration": duration, "ramp_rate": ramp_rate}],
    }


# --------------------------
# Start / lifecycle ordering
# --------------------------
def test_start_takes_control_runs_and_enters_running():
    h = Harness()
    clk = FakeClock(0)
    r = h.make(_ramp_proto(4, 10, 1), clk)
    assert r.start(initial_speed=0) is True
    assert r.state == HiitState.RUNNING
    assert h.take == 1 and h.run == 1
    assert h.speeds[0] == 0  # emits starting speed immediately


def test_start_twice_is_rejected():
    h = Harness()
    r = h.make(_ramp_proto(4, 10, 1), FakeClock(0))
    assert r.start() is True
    assert r.start() is False


def test_pause_resume_abort_invalid_when_idle():
    h = Harness()
    r = h.make(_ramp_proto(4, 10, 1), FakeClock(0))
    assert r.pause() is False
    assert r.resume() is False
    assert r.abort() is False


# --------------------------
# Ramp behavior
# --------------------------
def test_linear_ramp_emits_increasing_integers():
    h = Harness()
    clk = FakeClock(0)
    r = h.make(_ramp_proto(4, 10, 1), clk)  # 0->4 over 4s, then hold
    r.start(0)
    for t in (1, 2, 3, 4, 5, 6):
        clk.set(t)
        r.tick()
    # ramp produced 0,1,2,3,4 ; hold adds nothing new
    assert h.speeds == [0, 1, 2, 3, 4]


def test_no_duplicate_speed_commands():
    h = Harness()
    clk = FakeClock(0)
    r = h.make(_ramp_proto(4, 10, 1), clk)
    r.start(0)
    for t in [x * 0.25 for x in range(1, 60)]:  # sub-step ticks
        clk.set(t)
        r.tick()
    # never two identical consecutive commands
    assert all(a != b for a, b in zip(h.speeds, h.speeds[1:]))
    assert h.speeds[-1] == 4


def test_instant_jump_ramp_rate_zero():
    h = Harness()
    clk = FakeClock(0)
    r = h.make(_ramp_proto(30, 5, 0), clk)  # ramp_rate 0 -> instant to 30
    r.start(0)
    assert 30 in h.speeds
    # first emit is the target, not a gradual climb
    assert h.speeds[0] == 30


def test_same_speed_stage_holds_without_extra_commands():
    h = Harness()
    clk = FakeClock(0)
    data = {
        "protocol_name": "t",
        "steps": [
            {"type": "run", "speed": 10, "duration": 2, "ramp_rate": 0},
            {"type": "run", "speed": 10, "duration": 2, "ramp_rate": 5},  # same speed -> ramp_time 0
        ],
    }
    r = h.make(data, clk)
    r.start(0)
    for t in [0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5]:
        clk.set(t)
        r.tick()
    assert h.speeds == [10]  # only one distinct command across both stages
    assert r.state == HiitState.COMPLETE


# --------------------------
# Multi-stage advance + completion
# --------------------------
def test_completion_stops_belt_and_sets_complete():
    h = Harness()
    clk = FakeClock(0)
    r = h.make(_ramp_proto(5, 0, 5), clk)  # ramp 0->5 over 1s, 0 hold
    r.start(0)
    clk.set(1.0)
    r.tick()
    assert r.state == HiitState.COMPLETE
    assert h.stop == 1
    assert h.speeds[-1] == 5


def test_final_stop_stage_ends_at_zero():
    h = Harness()
    clk = FakeClock(0)
    data = {
        "protocol_name": "t",
        "steps": [
            {"type": "run", "speed": 20, "duration": 1, "ramp_rate": 0},
            {"type": "run", "speed": 0, "duration": 0, "ramp_rate": 10},  # ease to stop
        ],
    }
    r = h.make(data, clk)
    r.start(0)
    for t in [x * 0.2 for x in range(1, 30)]:
        clk.set(t)
        r.tick()
    assert r.state == HiitState.COMPLETE
    assert h.speeds[-1] == 0


def test_long_tick_chains_multiple_stages():
    # one big tick jumps past several short stages at once
    h = Harness()
    clk = FakeClock(0)
    data = {
        "protocol_name": "t",
        "steps": [
            {"type": "run", "speed": 10, "duration": 1, "ramp_rate": 0},
            {"type": "run", "speed": 20, "duration": 1, "ramp_rate": 0},
            {"type": "run", "speed": 30, "duration": 1, "ramp_rate": 0},
        ],
    }
    r = h.make(data, clk)
    r.start(0)
    clk.set(100.0)  # way past the end
    r.tick()
    assert r.state == HiitState.COMPLETE
    assert h.speeds[-1] == 30
    assert h.stop == 1


# --------------------------
# Pause (holds current speed) / resume (continues)
# --------------------------
def test_pause_holds_speed_and_resume_continues():
    h = Harness()
    clk = FakeClock(0)
    r = h.make(_ramp_proto(10, 100, 1), clk)  # 0->10 over 10s
    r.start(0)
    clk.set(2)
    r.tick()
    assert h.speeds[-1] == 2
    n_before = len(h.speeds)
    # pause and let real time pass — no new commands
    r.pause()
    assert r.state == HiitState.PAUSED
    for t in (3, 4, 5):
        clk.set(t)
        r.tick()  # ignored while paused
    assert len(h.speeds) == n_before  # speed held, nothing emitted
    # resume at t=5 (paused 3s); logical elapsed should resume near 2->3
    r.resume()
    assert r.state == HiitState.RUNNING
    clk.set(6)  # logical elapsed = 6 - 3(pause shift) = 3
    r.tick()
    assert h.speeds[-1] == 3  # continued, did NOT jump to 6


# --------------------------
# Abort
# --------------------------
def test_abort_zeros_speed_and_stops():
    h = Harness()
    clk = FakeClock(0)
    r = h.make(_ramp_proto(40, 100, 2), clk)
    r.start(0)
    clk.set(3)
    r.tick()
    assert h.speeds[-1] > 0
    assert r.abort() is True
    assert r.state == HiitState.ABORTED
    assert h.speeds[-1] == 0
    assert h.stop == 1


def test_abort_from_paused():
    h = Harness()
    clk = FakeClock(0)
    r = h.make(_ramp_proto(40, 100, 2), clk)
    r.start(0)
    clk.set(3)
    r.tick()
    r.pause()
    assert r.abort() is True
    assert r.state == HiitState.ABORTED


# --------------------------
# Progress reporting
# --------------------------
def test_progress_fields():
    h = Harness()
    clk = FakeClock(0)
    r = h.make(_ramp_proto(10, 5, 1), clk)  # ramp 10s? no: 0->10 over 10s, hold 5 -> total 15
    r.start(0)
    clk.set(4)
    r.tick()
    p = r.current_progress()
    assert p.state == HiitState.RUNNING
    assert p.stage_index == 0
    assert p.stage_count == 1
    assert p.target_speed == 10
    assert p.phase == "ramp"
    assert p.stage_total_s == pytest.approx(15.0)  # ramp_time 10 + hold 5
    assert 0 < p.total_elapsed_s <= p.stage_total_s


def test_release_on_finish_flag():
    h = Harness()
    clk = FakeClock(0)
    r = h.make(_ramp_proto(5, 0, 5), clk, release_on_finish=True)
    r.start(0)
    clk.set(1.0)
    r.tick()
    assert r.state == HiitState.COMPLETE
    assert h.release == 1


def test_reset_returns_to_idle_after_complete():
    h = Harness()
    clk = FakeClock(0)
    r = h.make(_ramp_proto(5, 0, 5), clk)
    r.start(0)
    clk.set(1.0)
    r.tick()
    assert r.state == HiitState.COMPLETE
    assert r.reset() is True
    assert r.state == HiitState.IDLE
    # can run again
    assert r.start() is True


def test_reset_rejected_while_running():
    h = Harness()
    r = h.make(_ramp_proto(5, 100, 5), FakeClock(0))
    r.start(0)
    assert r.reset() is False
