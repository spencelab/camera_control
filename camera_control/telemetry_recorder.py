#!/usr/bin/env python3
"""Session-scoped ROS 2 telemetry recording for camera_control.

The recorder intentionally runs ``ros2 bag record`` in a separate process so a
bag/storage failure cannot take down the camera GUI or camera acquisition nodes.
"""

from __future__ import annotations

import hashlib
import os
import shutil
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Sequence

from PySide6 import QtCore


POLICIES = {"required", "optional", "off"}


@dataclass(frozen=True)
class TelemetryTopicSpec:
    name: str
    expected_type: str
    device: str
    requirement: str


@dataclass(frozen=True)
class TelemetryPlan:
    topics: tuple[TelemetryTopicSpec, ...]
    missing_required_devices: tuple[str, ...]
    missing_optional_devices: tuple[str, ...]
    type_mismatches: tuple[str, ...]

    @property
    def topic_names(self) -> tuple[str, ...]:
        return tuple(spec.name for spec in self.topics)


class SessionTelemetryRecorder(QtCore.QObject):
    """Own one telemetry rosbag process for one recording session."""

    state_changed = QtCore.Signal(str)
    log_line = QtCore.Signal(str)
    stopped = QtCore.Signal(bool, str)
    _storage_preset_supported: Optional[bool] = None

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        self._process: Optional[subprocess.Popen] = None
        self._stdout_handle = None
        self._state = "idle"
        self._session_dir: Optional[Path] = None
        self._rosbag_dir: Optional[Path] = None
        self._bag_dir: Optional[Path] = None
        self._manifest_path: Optional[Path] = None
        self._plan: Optional[TelemetryPlan] = None
        self._command: list[str] = []
        self._start_wall_ns = 0
        self._start_monotonic_ns = 0
        self._stop_started = False
        self._expected_stop = False
        self._finalize_lock = threading.Lock()

        self._monitor_timer = QtCore.QTimer(self)
        self._monitor_timer.setInterval(1000)
        self._monitor_timer.timeout.connect(self._poll_process)

    @property
    def state(self) -> str:
        return self._state

    @property
    def bag_dir(self) -> Optional[Path]:
        return self._bag_dir

    def is_active(self) -> bool:
        return self._state in {"starting", "recording", "stopping"}

    def is_recording(self) -> bool:
        return self._state == "recording" and self._process is not None and self._process.poll() is None

    def _set_state(self, state: str) -> None:
        self._state = state
        self.state_changed.emit(state)

    @staticmethod
    def normalize_policy(value: object, default: str = "off") -> str:
        # PyYAML 1.1 treats an unquoted ``off`` as False. Accept that spelling
        # defensively so a rig cannot accidentally re-enable an inferred device.
        if value is False:
            return "off"
        text = str(default if value is None else value).strip().lower()
        return text if text in POLICIES else default

    @staticmethod
    def _topic_specs(
        camera_nodes: Sequence[str],
        treadmill_policy: str,
        triggerbox_policy: str,
        camera_events: bool,
    ) -> list[TelemetryTopicSpec]:
        specs: list[TelemetryTopicSpec] = []

        treadmill_policy = SessionTelemetryRecorder.normalize_policy(treadmill_policy)
        if treadmill_policy != "off":
            specs.extend([
                TelemetryTopicSpec(
                    "/treadmill_host/status",
                    "treadmill_control/msg/TreadmillStatus",
                    "treadmill",
                    treadmill_policy,
                ),
                TelemetryTopicSpec(
                    "/treadmill_host/events",
                    "treadmill_control/msg/TreadmillEvent",
                    "treadmill",
                    treadmill_policy,
                ),
            ])

        triggerbox_policy = SessionTelemetryRecorder.normalize_policy(triggerbox_policy)
        if triggerbox_policy != "off":
            specs.extend([
                TelemetryTopicSpec(
                    "/triggerbox_host/raw_measurements",
                    "triggerbox_ros2_interfaces/msg/TriggerClockMeasurement",
                    "triggerbox",
                    triggerbox_policy,
                ),
                TelemetryTopicSpec(
                    "/triggerbox_host/time_model",
                    "triggerbox_ros2_interfaces/msg/TriggerClockModel",
                    "triggerbox",
                    triggerbox_policy,
                ),
                TelemetryTopicSpec(
                    "/triggerbox_host/expected_framerate",
                    "std_msgs/msg/Float32",
                    "triggerbox",
                    triggerbox_policy,
                ),
                TelemetryTopicSpec(
                    "/triggerbox_host/output_enabled",
                    "std_msgs/msg/Bool",
                    "triggerbox",
                    triggerbox_policy,
                ),
            ])

        if camera_events:
            for node in camera_nodes:
                clean = str(node).strip().strip("/")
                if clean:
                    specs.append(
                        TelemetryTopicSpec(
                            f"/{clean}/recording_event",
                            "std_msgs/msg/String",
                            f"camera:{clean}",
                            "optional",
                        )
                    )
        return specs

    @classmethod
    def resolve_plan(
        cls,
        ros_node,
        camera_nodes: Sequence[str],
        treadmill_policy: str,
        triggerbox_policy: str,
        camera_events: bool = True,
    ) -> TelemetryPlan:
        specs = cls._topic_specs(camera_nodes, treadmill_policy, triggerbox_policy, camera_events)
        graph = {name: tuple(types) for name, types in ros_node.get_topic_names_and_types()}

        type_mismatches: list[str] = []
        device_present: dict[str, bool] = {}
        device_requirement: dict[str, str] = {}

        for spec in specs:
            if spec.device.startswith("camera:"):
                continue
            device_requirement[spec.device] = spec.requirement
            actual_types = graph.get(spec.name, ())
            matches = spec.expected_type in actual_types
            device_present[spec.device] = device_present.get(spec.device, False) or matches
            if actual_types and not matches:
                type_mismatches.append(
                    f"{spec.name}: expected {spec.expected_type}, found {', '.join(actual_types)}"
                )

        missing_required = sorted(
            device for device, requirement in device_requirement.items()
            if requirement == "required" and not device_present.get(device, False)
        )
        missing_optional = sorted(
            device for device, requirement in device_requirement.items()
            if requirement == "optional" and not device_present.get(device, False)
        )

        # Explicitly include all configured topics, even if not yet visible. rosbag2
        # discovery can subscribe when a publisher appears a moment later.
        return TelemetryPlan(
            topics=tuple(specs),
            missing_required_devices=tuple(missing_required),
            missing_optional_devices=tuple(missing_optional),
            type_mismatches=tuple(type_mismatches),
        )

    @staticmethod
    def _next_bag_dir(rosbag_dir: Path) -> Path:
        candidate = rosbag_dir / "telemetry_001"
        index = 1
        while candidate.exists():
            index += 1
            candidate = rosbag_dir / f"telemetry_{index:03d}"
        return candidate

    @staticmethod
    def _command_supports(option: str) -> bool:
        if option == "--storage-preset-profile" and SessionTelemetryRecorder._storage_preset_supported is not None:
            return SessionTelemetryRecorder._storage_preset_supported
        try:
            proc = subprocess.run(
                ["ros2", "bag", "record", "--help"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=8,
                check=False,
            )
            supported = option in (proc.stdout or "")
        except Exception:
            supported = False
        if option == "--storage-preset-profile":
            SessionTelemetryRecorder._storage_preset_supported = supported
        return supported

    def _capture_chrony(self, prefix: str) -> None:
        if self._rosbag_dir is None:
            return
        if shutil.which("chronyc") is None:
            (self._rosbag_dir / f"chrony_{prefix}.txt").write_text(
                "chronyc not installed or not on PATH\n", encoding="utf-8"
            )
            return
        chunks: list[str] = []
        for argv in (["chronyc", "tracking"], ["chronyc", "sources", "-v"]):
            try:
                proc = subprocess.run(
                    argv,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=8,
                    check=False,
                )
                chunks.append(f"$ {' '.join(argv)}\n{proc.stdout or ''}\n")
            except Exception as exc:
                chunks.append(f"$ {' '.join(argv)}\nFAILED: {exc}\n")
        (self._rosbag_dir / f"chrony_{prefix}.txt").write_text("\n".join(chunks), encoding="utf-8")

    @staticmethod
    def _yaml_scalar(value: object) -> str:
        text = str(value)
        return '"' + text.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'

    def _write_manifest(self, *, stop_wall_ns: int = 0, exit_code: Optional[int] = None, result: str = "active") -> None:
        if self._manifest_path is None or self._plan is None or self._bag_dir is None:
            return
        start_dt = datetime.fromtimestamp(self._start_wall_ns / 1e9, tz=timezone.utc) if self._start_wall_ns else None
        stop_dt = datetime.fromtimestamp(stop_wall_ns / 1e9, tz=timezone.utc) if stop_wall_ns else None
        lines = [
            "telemetry_recording:",
            "  schema_version: 1",
            f"  result: {self._yaml_scalar(result)}",
            f"  bag_dir: {self._yaml_scalar(str(self._bag_dir))}",
            "  storage_id: \"mcap\"",
            f"  start_utc: {self._yaml_scalar(start_dt.isoformat() if start_dt else '')}",
            f"  start_utc_ns: {self._start_wall_ns}",
            f"  start_monotonic_ns: {self._start_monotonic_ns}",
            f"  stop_utc: {self._yaml_scalar(stop_dt.isoformat() if stop_dt else '')}",
            f"  stop_utc_ns: {stop_wall_ns}",
            f"  exit_code: {'' if exit_code is None else exit_code}",
            "  command:",
        ]
        for token in self._command:
            lines.append(f"    - {self._yaml_scalar(token)}")
        lines.append("  topics:")
        for spec in self._plan.topics:
            lines.extend([
                f"    - name: {self._yaml_scalar(spec.name)}",
                f"      type: {self._yaml_scalar(spec.expected_type)}",
                f"      device: {self._yaml_scalar(spec.device)}",
                f"      requirement: {self._yaml_scalar(spec.requirement)}",
            ])
        lines.append("  missing_required_devices:")
        for item in self._plan.missing_required_devices:
            lines.append(f"    - {self._yaml_scalar(item)}")
        if not self._plan.missing_required_devices:
            lines.append("    []")
        lines.append("  missing_optional_devices:")
        for item in self._plan.missing_optional_devices:
            lines.append(f"    - {self._yaml_scalar(item)}")
        if not self._plan.missing_optional_devices:
            lines.append("    []")
        lines.append("  type_mismatches:")
        for item in self._plan.type_mismatches:
            lines.append(f"    - {self._yaml_scalar(item)}")
        if not self._plan.type_mismatches:
            lines.append("    []")
        self._manifest_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def start(self, session_dir: Path, plan: TelemetryPlan) -> tuple[bool, str]:
        if self.is_active():
            return False, "telemetry recorder is already active"
        if not plan.topics:
            self._set_state("skipped")
            return True, "telemetry skipped: rig has no configured telemetry topics"

        self._session_dir = Path(session_dir).expanduser().resolve()
        self._rosbag_dir = self._session_dir / "rosbag"
        self._rosbag_dir.mkdir(parents=True, exist_ok=True)
        # Any prior upload verification belongs to an older state of this
        # session. Starting a new bag must invalidate it immediately.
        try:
            (self._session_dir / ".SESSION_UPLOAD_VERIFY_OK").unlink()
        except FileNotFoundError:
            pass
        self._bag_dir = self._next_bag_dir(self._rosbag_dir)
        self._manifest_path = self._rosbag_dir / "telemetry_recording.yaml"
        self._plan = plan
        self._start_wall_ns = time.time_ns()
        self._start_monotonic_ns = time.monotonic_ns()
        self._stop_started = False
        self._expected_stop = False

        for marker in ("TELEMETRY_COMPLETE", "TELEMETRY_INCOMPLETE"):
            try:
                (self._rosbag_dir / marker).unlink()
            except FileNotFoundError:
                pass
        (self._rosbag_dir / "TELEMETRY_ACTIVE").write_text(
            f"pid=pending\nstarted_utc_ns={self._start_wall_ns}\n", encoding="utf-8"
        )
        self._capture_chrony("start")

        cmd = ["ros2", "bag", "record", "-s", "mcap", "-o", str(self._bag_dir)]
        if self._command_supports("--storage-preset-profile"):
            cmd.extend(["--storage-preset-profile", "zstd_fast"])
        cmd.append("--topics")
        cmd.extend(plan.topic_names)
        self._command = cmd
        self._write_manifest(result="starting")

        stdout_path = self._rosbag_dir / "rosbag_record.log"
        try:
            self._stdout_handle = stdout_path.open("a", encoding="utf-8", buffering=1)
            self._set_state("starting")
            self._process = subprocess.Popen(
                cmd,
                stdout=self._stdout_handle,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        except Exception as exc:
            self._set_state("failed")
            self._write_manifest(stop_wall_ns=time.time_ns(), result=f"start_failed: {exc}")
            try:
                (self._rosbag_dir / "TELEMETRY_ACTIVE").unlink()
            except FileNotFoundError:
                pass
            (self._rosbag_dir / "TELEMETRY_INCOMPLETE").write_text(f"start failed: {exc}\n", encoding="utf-8")
            if self._stdout_handle is not None:
                self._stdout_handle.close()
                self._stdout_handle = None
            return False, str(exc)

        # Give ros2 bag a brief chance to fail fast on a missing plugin/CLI option.
        time.sleep(0.35)
        rc = self._process.poll()
        if rc is not None:
            message = f"ros2 bag record exited during startup with code {rc}; see {stdout_path}"
            self._finalize(False, message, exit_code=rc)
            return False, message

        (self._rosbag_dir / "TELEMETRY_ACTIVE").write_text(
            f"pid={self._process.pid}\nstarted_utc_ns={self._start_wall_ns}\nbag_dir={self._bag_dir}\n",
            encoding="utf-8",
        )
        self._write_manifest(result="recording")
        self._set_state("recording")
        self._monitor_timer.start()
        return True, f"recording {len(plan.topics)} telemetry topic(s) to {self._bag_dir}"

    def _poll_process(self) -> None:
        proc = self._process
        if proc is None:
            return
        rc = proc.poll()
        if rc is None:
            return
        self._monitor_timer.stop()
        if self._expected_stop or self._stop_started:
            return
        message = f"telemetry recorder exited unexpectedly with code {rc}"
        self._finalize(False, message, exit_code=rc)
        self.log_line.emit(message)
        self.stopped.emit(False, message)

    def stop_async(self, reason: str = "recording stopped") -> None:
        if self._state in {"idle", "skipped", "complete", "failed"}:
            self.stopped.emit(self._state in {"idle", "skipped", "complete"}, f"telemetry already {self._state}")
            return
        if self._stop_started:
            return
        self._stop_started = True
        self._expected_stop = True
        self._monitor_timer.stop()
        self._set_state("stopping")
        thread = threading.Thread(target=self._stop_worker, args=(reason,), daemon=True)
        thread.start()

    def _stop_worker(self, reason: str) -> None:
        proc = self._process
        if proc is None:
            self._finalize(False, "telemetry process missing during stop", exit_code=None)
            self.stopped.emit(False, "telemetry process missing during stop")
            return

        shutdown_method = "already_exited"
        try:
            if proc.poll() is None:
                shutdown_method = "sigint"
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    shutdown_method = "sigterm"
                    os.killpg(proc.pid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        shutdown_method = "sigkill"
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait(timeout=2)
            rc = proc.returncode
            clean = rc in (0, 130, -signal.SIGINT)
            detail = f"{reason}; shutdown={shutdown_method}; exit_code={rc}"
            self._finalize(clean, detail, exit_code=rc)
            self.stopped.emit(clean, detail)
        except Exception as exc:
            self._finalize(False, f"stop failed: {exc}", exit_code=proc.poll())
            self.stopped.emit(False, f"telemetry stop failed: {exc}")

    def _bag_info_ok(self) -> tuple[bool, str]:
        if self._bag_dir is None or not self._bag_dir.exists():
            return False, "bag directory missing"
        metadata = self._bag_dir / "metadata.yaml"
        if not metadata.is_file():
            return False, "metadata.yaml missing"
        try:
            proc = subprocess.run(
                ["ros2", "bag", "info", str(self._bag_dir)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=20,
                check=False,
            )
            if self._rosbag_dir is not None:
                (self._rosbag_dir / "rosbag_info.txt").write_text(proc.stdout or "", encoding="utf-8")
            return proc.returncode == 0, (proc.stdout or "").strip()
        except Exception as exc:
            return False, str(exc)

    def _write_checksums(self) -> None:
        if self._rosbag_dir is None:
            return
        targets: list[Path] = []
        for name in ("telemetry_recording.yaml", "chrony_start.txt", "chrony_stop.txt", "rosbag_info.txt"):
            p = self._rosbag_dir / name
            if p.is_file():
                targets.append(p)
        if self._bag_dir is not None and self._bag_dir.exists():
            targets.extend(sorted(p for p in self._bag_dir.rglob("*") if p.is_file()))

        lines: list[str] = []
        for path in targets:
            digest = hashlib.sha256()
            with path.open("rb") as handle:
                for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                    digest.update(chunk)
            lines.append(f"{digest.hexdigest()}  {path.relative_to(self._rosbag_dir)}")
        (self._rosbag_dir / "SHA256SUMS").write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")

    def _finalize(self, clean_process: bool, detail: str, exit_code: Optional[int]) -> None:
        with self._finalize_lock:
            stop_wall_ns = time.time_ns()
            self._capture_chrony("stop")
            bag_ok, bag_info = self._bag_info_ok()
            success = bool(clean_process and bag_ok)
            result = "complete" if success else f"incomplete: {detail}; bag_info={bag_info}"
            self._write_manifest(stop_wall_ns=stop_wall_ns, exit_code=exit_code, result=result)
            self._write_checksums()

            if self._rosbag_dir is not None:
                try:
                    (self._rosbag_dir / "TELEMETRY_ACTIVE").unlink()
                except FileNotFoundError:
                    pass
                marker = "TELEMETRY_COMPLETE" if success else "TELEMETRY_INCOMPLETE"
                (self._rosbag_dir / marker).write_text(
                    f"completed_utc_ns={stop_wall_ns}\n{detail}\nbag_info_ok={bag_ok}\n",
                    encoding="utf-8",
                )

            if self._stdout_handle is not None:
                try:
                    self._stdout_handle.close()
                except Exception:
                    pass
                self._stdout_handle = None
            self._process = None
            self._set_state("complete" if success else "failed")
