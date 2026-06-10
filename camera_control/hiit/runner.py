# Author: Andrew England (andrewengland19)
# Created: 2026-06-08
# Last updated: 2026-06-08
"""HIIT trainer state machine + ramp engine.

Pure Python. No rclpy, no Qt. All side effects go through injected callables
(the "sinks"), and time comes from an injected ``clock`` — so the whole runner
is unit-testable on an MBP with a fake clock and recording mocks.

States: IDLE -> RUNNING -> (PAUSED) -> COMPLETE / ABORTED.
(LOADING is a transient UI/controller state during file import; the runner is
constructed with an already-validated protocol and begins in IDLE.)

Per stage the belt RAMPs from the current commanded speed to the stage target at
the stage's own ramp_rate (cm/s^2; 0 = instant jump), then HOLDs the target for
``duration`` seconds. Speeds are emitted as integer cm/s, de-duplicated, and
clamped to [MIN_SPEED, MAX_SPEED] (the device clamps again, authoritatively).
"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass
from typing import Callable, Optional

from .protocol import MAX_SPEED, MIN_SPEED, HiitProtocol, ResolvedStage, _ramp_time


class HiitState(enum.Enum):
    IDLE = "idle"
    LOADING = "loading"   # owned by the controller during import; not used internally
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETE = "complete"
    ABORTED = "aborted"


@dataclass
class HiitProgress:
    state: HiitState
    stage_index: int        # 0-based; -1 when idle/no stage
    stage_count: int
    stage_label: str
    phase: str              # "idle" | "ramp" | "hold" | "done"
    commanded_speed: int    # last integer speed emitted
    target_speed: int
    stage_elapsed_s: float
    stage_total_s: float    # ramp_time + hold duration for this stage
    total_elapsed_s: float
    total_estimated_s: float


def _noop(*_args, **_kwargs) -> None:
    pass


def _clamp_speed(value: float) -> int:
    return int(max(MIN_SPEED, min(MAX_SPEED, value)))


class HiitRunner:
    def __init__(
        self,
        protocol: HiitProtocol,
        *,
        set_speed: Callable[[int], None],
        take_control: Callable[[], None] = _noop,
        run_belt: Callable[[], None] = _noop,
        stop_belt: Callable[[], None] = _noop,
        release_control: Callable[[], None] = _noop,
        on_state_change: Optional[Callable[[HiitState, HiitState], None]] = None,
        on_progress: Optional[Callable[[HiitProgress], None]] = None,
        on_stage_change: Optional[Callable[[int, ResolvedStage], None]] = None,
        clock: Callable[[], float] = time.monotonic,
        tick_interval_s: float = 0.1,
        release_on_finish: bool = False,
    ) -> None:
        if not protocol.stages:
            raise ValueError("protocol has no stages")
        self.protocol = protocol
        self._set_speed = set_speed
        self._take_control = take_control
        self._run_belt = run_belt
        self._stop_belt = stop_belt
        self._release_control = release_control
        self._on_state_change = on_state_change
        self._on_progress = on_progress
        self._on_stage_change = on_stage_change  # fired when a new stage is entered
        self._clock = clock
        self.tick_interval_s = tick_interval_s
        self.release_on_finish = release_on_finish

        self._state = HiitState.IDLE
        self._stage_index = -1
        self._current_commanded = 0
        self._last_sent: Optional[int] = None
        self._stage_start = 0.0
        self._pause_started = 0.0
        self._prior_time = 0.0          # summed stage_total of completed stages
        # current-stage cached segment params
        self._seg_prev = 0
        self._seg_target = 0
        self._ramp_rate = 0.0
        self._ramp_time = 0.0
        self._hold_end = 0.0

    # -------- public API --------
    @property
    def state(self) -> HiitState:
        return self._state

    @property
    def stage_index(self) -> int:
        return self._stage_index

    def start(self, initial_speed: int = 0) -> bool:
        """Begin the protocol from IDLE. Returns False if not startable."""
        if self._state != HiitState.IDLE:
            return False
        self._current_commanded = _clamp_speed(initial_speed)
        self._last_sent = None
        self._prior_time = 0.0
        self._take_control()
        self._run_belt()
        self._set_state(HiitState.RUNNING)
        self._enter_stage(0, start_time=self._clock())
        self._service(self._clock())
        return True

    def pause(self) -> bool:
        if self._state != HiitState.RUNNING:
            return False
        self._pause_started = self._clock()
        self._set_state(HiitState.PAUSED)   # holds current speed; no command (Decision 1)
        self._notify_progress(self._pause_started)
        return True

    def resume(self) -> bool:
        if self._state != HiitState.PAUSED:
            return False
        now = self._clock()
        self._stage_start += (now - self._pause_started)  # keep logical elapsed continuous
        self._set_state(HiitState.RUNNING)
        self._service(now)
        return True

    def abort(self) -> bool:
        if self._state not in (HiitState.RUNNING, HiitState.PAUSED):
            return False
        self._emit(0, force=True)
        self._stop_belt()
        if self.release_on_finish:
            self._release_control()
        self._set_state(HiitState.ABORTED)
        self._notify_progress(self._clock())
        return True

    def reset(self) -> bool:
        """Return to IDLE so the (same) protocol can be re-run."""
        if self._state == HiitState.RUNNING:
            return False
        self._state = HiitState.IDLE
        self._stage_index = -1
        self._last_sent = None
        self._current_commanded = 0
        self._prior_time = 0.0
        return True

    def tick(self) -> None:
        """Drive the schedule forward. Called by a QTimer (or fake loop)."""
        if self._state != HiitState.RUNNING:
            return
        self._service(self._clock())

    def current_progress(self) -> HiitProgress:
        return self._build_progress(self._clock())

    # -------- internals --------
    def _set_state(self, new: HiitState) -> None:
        old = self._state
        if old == new:
            return
        self._state = new
        if self._on_state_change is not None:
            self._on_state_change(old, new)

    def _enter_stage(self, index: int, start_time: float) -> None:
        stage = self.protocol.stages[index]
        self._stage_index = index
        self._seg_prev = self._current_commanded
        self._seg_target = stage.speed
        self._ramp_rate = stage.ramp_rate
        self._ramp_time = _ramp_time(self._seg_prev, self._seg_target, stage.ramp_rate)
        self._hold_end = self._ramp_time + stage.duration
        self._stage_start = start_time
        if self._on_stage_change is not None:
            self._on_stage_change(index, stage)

    def _service(self, now: float) -> None:
        stages = self.protocol.stages
        # Advance through any stage(s) whose total time has elapsed (handles long
        # ticks and zero-length stages by carrying leftover logical time forward).
        while True:
            elapsed = now - self._stage_start
            if elapsed >= self._hold_end:
                self._emit(self._seg_target)
                self._current_commanded = self._seg_target
                nxt = self._stage_index + 1
                if nxt >= len(stages):
                    self._complete(now)
                    return
                self._prior_time += self._hold_end
                self._enter_stage(nxt, start_time=self._stage_start + self._hold_end)
                continue
            self._apply_ramp(now)
            self._notify_progress(now)
            return

    def _apply_ramp(self, now: float) -> None:
        elapsed = now - self._stage_start
        if self._ramp_time <= 0 or elapsed >= self._ramp_time:
            cmd = self._seg_target
        else:
            if self._seg_target >= self._seg_prev:
                desired = self._seg_prev + self._ramp_rate * elapsed
                cmd = min(round(desired), self._seg_target)   # never overshoot up
            else:
                desired = self._seg_prev - self._ramp_rate * elapsed
                cmd = max(round(desired), self._seg_target)   # never overshoot down
        self._emit(cmd)

    def _emit(self, value: float, force: bool = False) -> None:
        cmd = _clamp_speed(value)
        if force or cmd != self._last_sent:
            self._last_sent = cmd
            self._set_speed(cmd)

    def _complete(self, now: float) -> None:
        self._stop_belt()
        if self.release_on_finish:
            self._release_control()
        self._prior_time += self._hold_end
        self._set_state(HiitState.COMPLETE)
        self._notify_progress(now)

    def _notify_progress(self, now: float) -> None:
        if self._on_progress is not None:
            self._on_progress(self._build_progress(now))

    def _build_progress(self, now: float) -> HiitProgress:
        stages = self.protocol.stages
        if self._stage_index < 0 or self._state in (HiitState.IDLE,):
            return HiitProgress(
                state=self._state, stage_index=-1, stage_count=len(stages),
                stage_label="", phase="idle", commanded_speed=self._last_sent or 0,
                target_speed=0, stage_elapsed_s=0.0, stage_total_s=0.0,
                total_elapsed_s=0.0, total_estimated_s=self.protocol.estimated_total_s,
            )
        stage = stages[self._stage_index]
        if self._state == HiitState.COMPLETE:
            elapsed = self._hold_end
            phase = "done"
        elif self._state == HiitState.PAUSED:
            elapsed = min(self._pause_started - self._stage_start, self._hold_end)
            phase = "hold" if elapsed >= self._ramp_time else "ramp"
        else:
            elapsed = max(0.0, min(now - self._stage_start, self._hold_end))
            phase = "hold" if elapsed >= self._ramp_time else "ramp"
        return HiitProgress(
            state=self._state,
            stage_index=self._stage_index,
            stage_count=len(stages),
            stage_label=stage.label,
            phase=phase,
            commanded_speed=self._last_sent or 0,
            target_speed=self._seg_target,
            stage_elapsed_s=elapsed,
            stage_total_s=self._hold_end,
            total_elapsed_s=self._prior_time + elapsed,
            total_estimated_s=self.protocol.estimated_total_s,
        )
