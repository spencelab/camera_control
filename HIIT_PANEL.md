# HIIT Trainer — Treadmill Tab Extension

**Branch:** `tmill-hiit-yaml` · **Author:** Andrew England (andrewengland19) · **Status:** feature-complete, VM-verified against the fake treadmill host; pending real-rig validation.

A YAML-driven progressive HIIT trainer added to the **Treadmill** tab of `camera_control`. It drives the existing treadmill command pipeline through configurable workout protocols (ramp rates, target speeds, hold durations, interval loops), captures a per-stage timeline for aligning gait/stride data to indexed speeds, and adds an interactive regimen builder and live visualizations.

---

## 1. Design principles

1. **Purely additive — nothing existing is modified in behavior.** The entire feature lives in a new `camera_control/camera_control/hiit/` package. The only change to the pre-existing application file `camera_control.py` is **+56 lines / −0** (a guarded import, three small additions to `TreadmillPanel`, and mounting the panel on the Treadmill tab). `git diff main --stat` shows `camera_control.py` as the *only* modified pre-existing file.
2. **Fail-safe / invisible when absent.** The import and construction are wrapped in `try/except` (`HIIT_AVAILABLE`). If the package or any dependency is missing, the panel is simply not added and the cockpit is byte-for-byte unchanged. Lab members who don't use HIIT are unaffected.
3. **Manual control is never degraded.** When no protocol is running, the manual treadmill controls behave exactly as before. The trainer is a software layer *on top of* the existing pipeline; it takes exclusive control only while a protocol runs and cleanly hands back afterward.
4. **No new runtime dependencies.** Logic uses only the Python stdlib + PyYAML (already used by `camera_control`); the GUI uses only PySide6 + `QtGui.QPainter` (no QtCharts), so the VM's package set is unchanged.
5. **One source of truth.** Every protocol — imported, built, or manually ramped — is validated and executed through the same schema (`protocol_from_dict`) and the same state machine (`HiitRunner`).
6. **Decoupled for testing.** The schema, state machine, run-log, and serialization are pure Python (no rclpy, no Qt) and unit-tested on a machine without ROS. The GUI is offscreen-smoke-tested. Total: **74 tests** (60 logic + 14 GUI) green under PySide6; 57 pass on a ROS/Qt-less machine (GUI suites skip).

---

## 2. Package layout

```
camera_control/camera_control/hiit/
  protocol.py     # schema, validation, loop expansion, loader, ramp builder, plot/serialize helpers   (pure)
  runner.py       # HiitRunner state machine + ramp engine                                              (pure)
  runlog.py       # per-stage run-log timeline                                                          (pure)
  settings.py     # QSettings wrapper (paths + window geometry)                                         (Qt, no rclpy)
  graphs.py       # ProfilePlot, PhaseScrubber, TelemetryPlot + detachable dialogs                      (Qt, no rclpy)
  builder.py      # RegimenBuilderDialog                                                                (Qt, no rclpy)
  panel.py        # HiitPanel — the widget mounted on the Treadmill tab                                 (Qt, no rclpy)
  controller.py   # HiitController — glue between runner and live GUI/ROS                               (Qt, no rclpy)
  tests/          # pytest: protocol, runner, runlog, graphs, builder, panel smoke
camera_control/configs/hiit_protocols/example_hiit.yaml
```

`protocol.py`, `runner.py`, `runlog.py` import **neither rclpy nor Qt**. `controller.py` imports Qt but **not rclpy** — the `CameraControlRos` node and the manual `TreadmillPanel` are injected, which is what keeps the whole package importable and testable off-rig.

---

## 3. Integration with the existing system

### 3.1 Mount point
`MainWindow.__init__` constructs a `HiitController` and `HiitPanel` (guarded) and adds the panel (titled **"Automated Speed Controller"**) beneath the existing `TreadmillPanel` on the Treadmill tab. No new tab, no change to tab ordering.

### 3.2 Command pipeline
All belt commands route through the existing choke point **`TreadmillPanel.set_speed()`** (reusing its `[0,100]` clamp and spinbox/UI sync). Lifecycle actions use the existing `CameraControlRos.treadmill_trigger_async(...)`. The trainer adds **no new ROS publishers, subscribers, or services** — it consumes the `treadmill_control` interface already present:

| Used | Type |
|---|---|
| `/treadmill_host/set_speed` | `treadmill_control/srv/SetSpeed` |
| `/treadmill_host/{take_control,run,stop,release_control}` | `std_srvs/srv/Trigger` |
| `/treadmill_host/status` (read) | `treadmill_control/msg/TreadmillStatus` |

The device clamps speed to `max(0, min(max_speed_cm_s, …))` (default 100) — the trainer's static ceiling of 100 cm/s matches that default.

### 3.3 Mutual exclusion (manual ↔ trainer)
While a protocol is `RUNNING`/`PAUSED`, the controller sets `TreadmillPanel._hiit_lock = True` and calls `TreadmillPanel.set_manual_enabled(False)` to disable the manual command widgets; a one-line guard in `TreadmillPanel.handle_key` suppresses the treadmill hotkeys. On `COMPLETE`/`ABORTED` both revert — clean handoff. Default state is unlocked, so manual driving needs zero HIIT interaction.

---

## 4. Features

### 4.1 Phased regimens (YAML)
Protocols are authored in YAML (see schema §5). On import the regimen is validated, flattened (loops expanded), and its estimated duration computed; the status chip turns green. **Run Protocol** executes it.

### 4.2 Execution engine (`HiitRunner`)
States: `IDLE → RUNNING → (PAUSED) → COMPLETE / ABORTED`. Per stage the belt **ramps** from the current commanded speed to the stage target at the stage's own `ramp_rate` (cm/s²; `0` = instant jump), then **holds** the target for `duration` seconds. Speeds are emitted as integer cm/s, de-duplicated, and clamped. Driven by a 10 Hz `QTimer` on the Qt thread (consistent with the app's existing single-threaded ROS spin). **Pause holds the current belt speed** and freezes the schedule; resume continues mid-stage. **Abort** zeroes speed and stops the belt. Time and all side-effects are injectable, making the engine fully unit-testable with a fake clock.

### 4.3 Manual Ramp Protocol
A stepwise graded ramp (`target` / `step` / `every`) authored inline via spinboxes and run with one click. It lives in its own **"Manual Ramp (no regimen needed)"** sub-group and is fully **standalone** — operators can use it without importing or running any regimen, making it a general treadmill-control convenience independent of the HIIT workflow. It is synthesized into a normal protocol and executed on the same engine, so it inherits the lockout, progress, scrubber, and run-log. The spinboxes are seeded from an imported regimen's `target/step/every` values when one is loaded.

A **Gentle Stop** button sits beside Run Ramp and is **always enabled**: it eases the belt from its current speed to 0 at a fixed hardcoded deceleration (`GENTLE_STOP_DECEL_CM_S2 = 4.0` cm/s², ≈9 s from 36 cm/s). It works whether or not a protocol is running — pressing it during a run takes over and finalizes the interrupted run's log as `gentle_stopped`. (Distinct from **Abort**, which is an immediate stop.) The gentle stop itself does not write a run-log.

### 4.4 Interactive regimen builder (`RegimenBuilderDialog`)
"➕ Create New Regimen" opens a form-based editor: header metadata + a tree of **run steps** and **"repeat ×N" groups** (the minimum that reproduces `example_hiit.yaml`), with add/edit/duplicate/delete/reorder. Edits validate live through the real schema (showing the same path-aware error messages the loader produces) and display computed stage count + duration. **Save** writes a schema-correct YAML (clean header comment + body) to a location chosen on the fly; **Save & Load** also activates it. Non-coders can author protocols without touching YAML; the output is identical to a hand-written file.

### 4.5 Live visualizations
- **Speed-vs-time profile graph** (detachable window, toggled by a checkbox): the whole planned speed curve with a live "you-are-here" cursor.
- **Commanded-vs-reported telemetry scope** (detachable window): rolling plot of commanded vs `reported_speed_cm_s` from `/treadmill_host/status` over a configurable window (default 90 s). Works during runs *and* manual driving — effectively a treadmill oscilloscope for verifying tracking.
- **Colored phase scrubber** (inline): a proportional, color-coded timeline of the protocol — green (slow) → red (fast), **dark red at/above 48 cm/s** — with the current stage outlined and a moving position marker.

Both graphs are independent, non-modal windows (movable beside the cockpit for multitasking); their geometry persists, and closing a window keeps its checkbox in sync. All plots are `QPainter`-drawn (no QtCharts dependency).

### 4.6 Per-stage run-log timeline
On every run the trainer writes `hiit_run_<timestamp>_<name>.yaml` recording, per stage: index, label, target speed, ramp rate, provenance, and **wall-clock + monotonic** start/end times and durations, plus the outcome and estimated/actual totals. Wall-clock timestamps (chrony-synced on the rig) let gait/stride data be aligned to protocol stages and indexed speeds offline; monotonic timestamps give drift-free durations. This writes a single YAML file and has **zero coupling** to the camera recording path or any ROS interface.

### 4.7 On-the-fly settings
Regimen-save directory and run-log directory are stored via `QSettings` and changeable at any time (default `~/hiit_protocols` and `~/hiit_runs`), so the PI can route artifacts wherever preferred without code changes.

---

## 5. YAML schema (matches `configs/hiit_protocols/example_hiit.yaml`)

```yaml
protocol_name: <str, required>
date: <str/date, optional>            # display only
description: <str, optional>
target: <int>   # \
step:   <int>   #  > seeds for the manual Ramp spinboxes (phased runner ignores)
every:  <int>   # /
defaults:
  ramp_rate_cm_s2: <float>            # fallback ramp rate; required iff any step omits ramp_rate
steps:                                # executed top to bottom
  - type: run
    speed: <int 0..100>              # target speed, cm/s
    duration: <number >= 0>          # HOLD time at target (excludes ramp time), seconds
    ramp_rate: <float >= 0>          # cm/s^2; 0 = instant jump; omit -> defaults.ramp_rate_cm_s2
    label: <str, optional>
  - type: loop
    count: <int >= 1>
    steps: [ ... ]                    # same syntax; nesting allowed
```

Validation raises descriptive, path-located errors (e.g. `steps[3]: 'speed' 120 out of range [0, 100] cm/s`). Speeds outside `[0,100]` are rejected at load (not silently clamped). Guards cap loop expansion (10 000 stages) and nesting depth (10).

---

## 6. Footprint & verification

- **Pre-existing files changed:** `camera_control.py` only, **+56 / −0**. Everything else is new files.
- **Tests:** `pytest camera_control/hiit/tests/` — 74 pass under PySide6 (60 pure logic + 14 offscreen GUI); 57 pass with 3 GUI suites skipped where PySide6 is absent.
- **VM verified:** runs in the real cockpit against `ros2 run treadmill_control treadmill_host --fake`; protocol execution, manual lockout/handoff, run-log output, and the builder/graphs all confirmed.

### Run / verify
```bash
# workspace built & sourced; PySide6 + libxcb-cursor0 installed (see VM notes)
ros2 run treadmill_control treadmill_host --fake --ros-args -p enabled:=true -p auto_connect:=true
cd ~/ros2_ws/src/camera_control && python3 camera_control/camera_control.py
# Treadmill tab -> HIIT panel -> Import example_hiit.yaml -> Run Protocol
```

---

## 7. Extension points

- New per-step schema fields (e.g. gait-capture annotation flags) are additive in `protocol.py` and surface automatically in the run-log.
- Additional telemetry channels can subscribe to `TreadmillPanel.status_changed` without new ROS plumbing.
- The run-log is the integration seam for offline gait analysis; it intentionally avoids coupling acquisition to analysis.
```
