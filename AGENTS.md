# AGENTS.md — camera_control

*Drafted from reading the 2026-08-25 source snapshot.*

## Role

Operator GUI and system orchestration (PySide6 + rclpy, plain Python —
**not a colcon package**, no `setup.py`/`package.xml` at all; lives
under `~/ros2_ws/src/camera_control` only so its ROS dependencies build,
but is run directly as a script). Selects rig profiles, launches/stops
camera nodes, controls trigger output, shows status, integrates
treadmill controls, does coordinated shutdown, records a telemetry
rosbag alongside every session.

## Run

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/.venv_gui/bin/activate     # PySide6 lives here, not in package.xml anywhere
cd ~/ros2_ws/src/camera_control
python3 camera_control/camera_control.py
```
The top-of-file docstring references an old filename
(`camera_control_with_metadata.py`) — stale, ignore it; the real
entry point is `camera_control/camera_control/camera_control.py`'s
`main()`.

## Structure

One large `MainWindow` script (`camera_control.py`, ~3400 lines) plus
sibling modules: `telemetry_recorder.py`, `processing_panel.py`
(processing/upload tab — 3800 lines), `utilities_panel.py`, a `hiit/`
subpackage (progressive treadmill-workout automation, guarded import,
has its own `pytest` suite — see below), and `multicam_sync_audit.py`
(standalone frame-alignment audit tool, no rclpy).

## Cross-repo ROS interfaces this code actually calls

- **cambuffer_recorder_ng** (per camera): `~/get_status`, `~/apply_settings`,
  `~/start_recording`/`~/stop_recording` (`std_srvs/Trigger`),
  `~/dump_buffer`. Discovers camera nodes at runtime by scanning for
  anything exposing a `.../get_status` service of the right type.
- **triggerbox_ros2**: `/triggerbox_host/enable_output`,
  `/triggerbox_host/disable_output` (`std_srvs/Trigger`). Notably: the
  RAM-buffer dump flow (`dump_ram_buffer()`, ~line 2352) disables the
  triggerbox before dumping and re-enables after — a workaround for
  cambuffer_recorder_ng's current camera-stop-during-dump behavior. If
  the ping-pong (continue_acquisition) feature lands there, this
  workaround can eventually be removed — but don't remove it in the
  same change that adds ping-pong.
- **treadmill_control**: `/treadmill_host/{connect,detect_port,
  disconnect,take_control,release_control,run,stop,set_speed}`,
  subscribes `/treadmill_host/status`. Guarded import — GUI runs fine
  without treadmill_control installed.

## Config

`configs/rigs.yaml` — rig presets (which cameras, ssh commands to launch
remote camera hosts, triggerbox/treadmill telemetry requirements,
shutdown sequence). `configs/processing.yaml` (+ several personal
variants) — processing/upload pipeline config, host-routing syntax
(`cam1@cam1` = ssh, `cam1@local` = same machine). `configs/hiit_protocols/`
— YAML workout regimens for the HIIT panel.

## Known gaps / stale content

- README's own TODO list is only partly current — item 1 ("Select
  System button") looks already implemented (comment at ~line 1195
  contradicts the TODO); items 2-4 are genuinely open.
- No in-code TODO/FIXME markers exist — open items live only in README
  prose.
- HIIT panel is explicitly self-described as "VM-verified against the
  fake treadmill host; pending real-rig validation" — don't treat it as
  confirmed on real hardware.

## Testing

`pytest camera_control/hiit/tests/` — HIIT panel only (pure-logic +
offscreen-GUI tests). No test coverage exists for the main window,
telemetry recorder, or processing panel.

See the workspace-level CLAUDE.md (`~/ros2_ws/CLAUDE.md`) for the
architecture-repo imports and testing-level policy (ADR 0007) — this
file stays repo-specific.
