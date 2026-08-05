"""Processing tab for camera_control.

MERB pilot tools:
  - scan local ~/camera_sessions for sessions
  - ask cam1-cam5 to extract first-frame PNG thumbnails from matching .cbrraw files
  - ask camera hosts to convert raw rolling files to MP4 in-place
  - verify MP4 frame counts against raw audit output
  - delete verified raw binaries
  - upload processed camera files directly from camera hosts to storage
  - upload tmill session-level files to the same storage session directory

This module intentionally avoids rclpy. It is just PySide6 + subprocess so the
main camera cockpit does not grow another tentacle.
"""

from __future__ import annotations

import os
import signal
import shlex
import shutil
import socket
import subprocess
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from PySide6 import QtCore, QtWidgets


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _default_processing_config_path() -> Path:
    return Path(os.environ.get(
        "CAMERA_CONTROL_PROCESSING_YAML",
        str(_repo_root() / "configs" / "processing.yaml"),
    )).expanduser()


def _load_processing_config() -> Dict[str, Any]:
    """Load optional processing.yaml, returning permissive defaults if absent."""
    defaults: Dict[str, Any] = {
        "processing": {
            "local_sessions_root": str(Path.home() / "camera_sessions"),
            "remote_sessions_root": "/home/spencelab/camera_sessions",
            "cameras": ["cam1", "cam2", "cam3", "cam4", "cam5"],
            "processed_subdir": "processed",
            "thumbnails_subdir": "thumbnails",
            "manifest_name": "processing_manifest.tsv",
            "conversion": {
                "fps": 5.0,
                "r_gain": 1.23,
                "g_gain": 1.00,
                "b_gain": 1.60,
                "gamma": 1.0,
                "audit_threshold_frames": 1.5,
            },
            "upload": {
                "host": "gpu2",
                "user": "spencelab",
                "port": "",
                "root": "/zfstank3/storage/camera_sessions_uploads",
                "verify": "size",
                "max_parallel_uploads": 1,
            },
            "rosbag": {
                "subdir": "rosbag",
                "upload_from": "tmill",
            },
        }
    }

    path = _default_processing_config_path()
    if not path.exists():
        return defaults

    try:
        import yaml  # type: ignore
        loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if not isinstance(loaded, dict):
            return defaults

        def deep_update(dst: Dict[str, Any], src: Dict[str, Any]) -> Dict[str, Any]:
            for key, value in src.items():
                if isinstance(value, dict) and isinstance(dst.get(key), dict):
                    deep_update(dst[key], value)
                else:
                    dst[key] = value
            return dst

        return deep_update(defaults, loaded)
    except Exception:
        return defaults


def _tail(text: str, n: int = 8) -> str:
    lines = [line for line in (text or "").splitlines() if line.strip()]
    return " | ".join(lines[-n:]) if lines else "no output"


def _q(value: object) -> str:
    return shlex.quote(str(value))


@dataclass(frozen=True)
class CameraSpec:
    cam: str
    host: str


def _is_local_host(host: str) -> bool:
    text = str(host or "").strip().lower()
    local_names = {
        "",
        "local",
        "localhost",
        "127.0.0.1",
        "::1",
        socket.gethostname().lower(),
        socket.getfqdn().lower(),
    }
    return text in local_names


def _camera_spec(token: str) -> CameraSpec:
    """Parse camera tokens used in processing.yaml / GUI.

    Supported forms:
      cam1             -> camera dir cam1 on host cam1, via ssh
      cam1@cam1        -> camera dir cam1 on host cam1, via ssh
      cam1@local       -> camera dir cam1 on this machine, no ssh/scp
      cam1=local       -> same as cam1@local
      cam1@ros2test    -> camera dir cam1 on host ros2test
    """
    text = str(token or "").strip()
    if not text:
        raise ValueError("empty camera token")
    if "@" in text:
        cam, host = text.split("@", 1)
    elif "=" in text:
        cam, host = text.split("=", 1)
    else:
        cam, host = text, text
    cam = cam.strip().lstrip("/")
    host = host.strip()
    if not cam:
        raise ValueError(f"bad camera token: {token!r}")
    return CameraSpec(cam=cam, host=host or cam)


def _camera_specs(tokens: List[str]) -> List[CameraSpec]:
    specs: List[CameraSpec] = []
    seen = set()
    for token in tokens:
        if not str(token).strip():
            continue
        spec = _camera_spec(token)
        key = (spec.cam, spec.host)
        if key not in seen:
            specs.append(spec)
            seen.add(key)
    return specs


@dataclass(frozen=True)
class ThumbnailJob:
    session: str
    cam: str
    host: str
    local_thumb: Path


class ThumbnailWorker(QtCore.QObject):
    log = QtCore.Signal(str)
    progress = QtCore.Signal(int, int)
    finished = QtCore.Signal(int, int)

    def __init__(
        self,
        *,
        base_dir: Path,
        remote_sessions_root: str,
        sessions: List[str],
        cameras: List[str],
        r_gain: float,
        g_gain: float,
        b_gain: float,
        gamma: float,
        parent: Optional[QtCore.QObject] = None,
    ):
        super().__init__(parent)
        self.base_dir = base_dir.expanduser()
        self.remote_sessions_root = remote_sessions_root.rstrip("/")
        self.sessions = sessions
        self.cameras = cameras
        self.camera_specs = _camera_specs(cameras)
        self.r_gain = r_gain
        self.g_gain = g_gain
        self.b_gain = b_gain
        self.gamma = gamma
        self._cancelled = False

    @QtCore.Slot()
    def cancel(self) -> None:
        self._cancelled = True

    @QtCore.Slot()
    def run(self) -> None:
        jobs = [
            ThumbnailJob(
                session=session,
                cam=spec.cam,
                host=spec.host,
                local_thumb=self.base_dir / session / "thumbnails" / f"{spec.cam}_first.png",
            )
            for session in self.sessions
            for spec in self.camera_specs
        ]
        total = len(jobs)
        done = 0
        ok = 0
        self.log.emit(f"Processing: thumbnail scan starting for {len(self.sessions)} sessions x {len(self.camera_specs)} cameras")

        for job in jobs:
            if self._cancelled:
                self.log.emit("Processing: cancelled")
                break
            if self._process_one(job):
                ok += 1
            done += 1
            self.progress.emit(done, total)

        self.log.emit(f"Processing: thumbnails complete: {ok}/{done} copied")
        self.finished.emit(ok, done)

    def _run_local(self, argv: List[str], *, timeout_s: int = 60) -> subprocess.CompletedProcess:
        return subprocess.run(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout_s,
            check=False,
        )

    def _remote_script(self, session: str, cam: str) -> str:
        # The camera node records into that host's local camera_sessions tree.
        r = shlex.quote(f"{self.r_gain:.6g}")
        g = shlex.quote(f"{self.g_gain:.6g}")
        b = shlex.quote(f"{self.b_gain:.6g}")
        gamma = shlex.quote(f"{self.gamma:.6g}")

        return f"""
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ROOT={_q(self.remote_sessions_root)}
SESSION={_q(session)}
CAM={_q(cam)}
DIR="$ROOT/$SESSION/$CAM"
if [[ ! -d "$DIR" ]]; then
  echo "NO_SESSION_DIR $DIR"
  exit 20
fi
RAW=$(find "$DIR" -maxdepth 1 -type f -name '*_0000.cbrraw' | sort | head -n 1)
if [[ -z "${{RAW}}" ]]; then
  echo "NO_CBRRAW $DIR"
  exit 21
fi
META="${{RAW%_0000.cbrraw}}.metadata.yaml"
if [[ ! -f "$META" ]]; then
  META_BY_CAM=$(find "$DIR" -maxdepth 1 -type f -name "{cam}_*.metadata.yaml" | sort | head -n 1)
  if [[ -n "$META_BY_CAM" ]]; then
    META="$META_BY_CAM"
  fi
fi
if [[ ! -f "$META" ]]; then
  META_ANY=$(find "$DIR" -maxdepth 1 -type f -name '*.metadata.yaml' | sort | head -n 1)
  if [[ -n "$META_ANY" ]]; then
    META="$META_ANY"
  fi
fi
PNG="$DIR/{cam}_first.png"
if [[ ! -f "$META" ]]; then
  META=none
fi
ros2 run cambuffer_recorder_ng raw_rolling_to_mp4 "$RAW" "$PNG" 1 0 "$META" {r} {g} {b} {gamma}
echo "$PNG"
""".strip()

    def _process_one(self, job: ThumbnailJob) -> bool:
        job.local_thumb.parent.mkdir(parents=True, exist_ok=True)
        remote_script = self._remote_script(job.session, job.cam)
        self.log.emit(f"Processing: {job.session}/{job.cam}: extracting first frame on {job.host}")

        try:
            if _is_local_host(job.host):
                proc = self._run_local(["bash", "-lc", remote_script], timeout_s=120)
            else:
                proc = self._run_local(
                    ["ssh", "-T", f"spencelab@{job.host}", remote_script],
                    timeout_s=120,
                )
        except subprocess.TimeoutExpired:
            self.log.emit(f"Processing: {job.session}/{job.cam}: TIMEOUT during frame extraction")
            return False

        out = (proc.stdout or "").strip()
        if proc.returncode != 0:
            # Missing sessions are expected when tmill has session.yaml but that camera did not record.
            self.log.emit(f"Processing: {job.session}/{job.cam}: skip/fail rc={proc.returncode}: {_tail(out, 4)}")
            return False

        remote_png = out.splitlines()[-1].strip() if out else f"{self.remote_sessions_root}/{job.session}/{job.cam}/{job.cam}_first.png"
        self.log.emit(f"Processing: {job.session}/{job.cam}: copying thumbnail to {job.local_thumb}")
        if _is_local_host(job.host):
            try:
                shutil.copy2(Path(remote_png).expanduser(), job.local_thumb)
            except Exception as exc:
                self.log.emit(f"Processing: {job.session}/{job.cam}: local thumbnail copy failed: {exc}")
                return False
            return True

        try:
            scp = self._run_local(
                ["scp", "-q", f"spencelab@{job.host}:{remote_png}", str(job.local_thumb)],
                timeout_s=60,
            )
        except subprocess.TimeoutExpired:
            self.log.emit(f"Processing: {job.session}/{job.cam}: TIMEOUT during scp")
            return False

        if scp.returncode != 0:
            self.log.emit(f"Processing: {job.session}/{job.cam}: scp failed rc={scp.returncode}: {_tail(scp.stdout, 4)}")
            return False

        return True


@dataclass(frozen=True)
class PipelineJob:
    session: str
    cam: str
    host: str


class PipelineWorker(QtCore.QObject):
    log = QtCore.Signal(str)
    status = QtCore.Signal(str)
    progress = QtCore.Signal(int, int)
    finished = QtCore.Signal(str, int, int)

    def __init__(
        self,
        *,
        action: str,
        base_dir: Path,
        remote_sessions_root: str,
        sessions: List[str],
        cameras: List[str],
        fps: float,
        r_gain: float,
        g_gain: float,
        b_gain: float,
        gamma: float,
        audit_threshold_frames: float,
        processed_subdir: str,
        thumbnails_subdir: str,
        manifest_name: str,
        upload_user: str,
        upload_host: str,
        upload_port: str,
        upload_root: str,
        parent: Optional[QtCore.QObject] = None,
    ):
        super().__init__(parent)
        self.action = action
        self.base_dir = base_dir.expanduser()
        self.remote_sessions_root = remote_sessions_root.rstrip("/")
        self.sessions = sessions
        self.cameras = cameras
        self.camera_specs = _camera_specs(cameras)
        self.fps = float(fps)
        self.r_gain = float(r_gain)
        self.g_gain = float(g_gain)
        self.b_gain = float(b_gain)
        self.gamma = float(gamma)
        self.audit_threshold_frames = float(audit_threshold_frames)
        self.processed_subdir = processed_subdir.strip("/") or "processed"
        self.thumbnails_subdir = thumbnails_subdir.strip("/") or "thumbnails"
        self.manifest_name = manifest_name or "processing_manifest.tsv"
        self.upload_user = upload_user.strip() or "spencelab"
        self.upload_host = upload_host.strip() or "gpu2"
        self.upload_port = str(upload_port or "").strip()
        self.upload_root = upload_root.rstrip("/")
        self._cancelled = False
        self._current_proc: Optional[subprocess.Popen[str]] = None

    @QtCore.Slot()
    def cancel(self) -> None:
        self._cancelled = True
        proc = self._current_proc
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                self.log.emit(f"Processing: sent SIGTERM to active process group pid={proc.pid}")
            except Exception as exc:
                self.log.emit(f"Processing: could not terminate active process pid={proc.pid}: {exc}")

    @QtCore.Slot()
    def run(self) -> None:
        action_plan = self._expand_action(self.action)
        jobs = [PipelineJob(session=s, cam=spec.cam, host=spec.host) for s in self.sessions for spec in self.camera_specs]
        per_cam_steps = [a for a in action_plan if a not in ("upload_session", "delete_session_local")]
        session_upload_passes = 2 if "upload_session" in action_plan else 0
        session_delete_passes = 1 if "delete_session_local" in action_plan else 0
        total = len(self.sessions) * (session_upload_passes + session_delete_passes) + len(jobs) * len(per_cam_steps)
        total = max(1, total)
        done = 0
        ok = 0

        self.log.emit(f"Processing: action '{self.action}' starting for {len(self.sessions)} sessions x {len(self.camera_specs)} cameras")
        for session in self.sessions:
            if self._cancelled:
                break
            if "upload_session" in action_plan:
                if self._upload_session_level_files(session):
                    ok += 1
                done += 1
                self.progress.emit(done, total)

            for job in [j for j in jobs if j.session == session]:
                for step in per_cam_steps:
                    if self._cancelled:
                        break
                    if self._run_camera_step(job, step):
                        ok += 1
                    done += 1
                    self.progress.emit(done, total)
                if self._cancelled:
                    break

            # Upload tmill session-level files again after camera steps so the
            # storage copy gets the updated processing_manifest.tsv entries.
            if (not self._cancelled) and "upload_session" in action_plan:
                if self._upload_session_level_files(session):
                    ok += 1
                done += 1
                self.progress.emit(done, total)

            # Final cleanup: remove local/control-machine session copy after
            # all selected camera-host cleanup steps have run.
            if (not self._cancelled) and "delete_session_local" in action_plan:
                if self._delete_local_session_tree(session):
                    ok += 1
                done += 1
                self.progress.emit(done, total)

        if self._cancelled:
            self.log.emit(f"Processing: action '{self.action}' cancelled")
        self.log.emit(f"Processing: action '{self.action}' complete: {ok}/{done} step(s) OK")
        self.finished.emit(self.action, ok, done)

    def _expand_action(self, action: str) -> List[str]:
        if action == "process":
            return ["process"]
        if action == "info":
            return ["info"]
        if action == "audit":
            return ["audit"]
        if action == "info_audit":
            return ["info", "audit"]
        if action == "verify":
            return ["verify"]
        if action == "delete_raws":
            return ["delete_raws"]
        if action == "upload":
            return ["upload_session", "upload"]
        if action == "verify_upload":
            return ["verify_upload"]
        if action == "delete_uploaded_local":
            return ["delete_uploaded_local"]
        if action == "delete_uploaded_session_local":
            return ["delete_camera_session_local", "delete_session_local"]
        if action == "process_verify":
            return ["process", "verify"]
        if action == "upload_verify":
            return ["upload_session", "upload", "verify_upload"]
        if action == "process_verify_upload":
            return ["process", "verify", "upload_session", "upload", "verify_upload"]
        return [action]

    def _run_local(self, argv: List[str], *, timeout_s: int = 600) -> subprocess.CompletedProcess:
        return subprocess.run(
            argv,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout_s,
            check=False,
        )

    def _ssh(self, host: str, script: str, *, timeout_s: int = 3600) -> subprocess.CompletedProcess:
        if _is_local_host(host):
            return self._run_local(["bash", "-lc", script], timeout_s=timeout_s)
        return self._run_local(["ssh", "-T", f"spencelab@{host}", script], timeout_s=timeout_s)

    def _ssh_streaming(self, host: str, script: str, *, label: str) -> subprocess.CompletedProcess:
        if _is_local_host(host):
            argv = ["bash", "-lc", script]
        else:
            argv = ["ssh", "-T", f"spencelab@{host}", script]
        return self._run_streaming(argv, label=label)

    def _run_streaming(self, argv: List[str], *, label: str) -> subprocess.CompletedProcess:
        lines: List[str] = []
        proc: Optional[subprocess.Popen[str]] = None
        try:
            proc = subprocess.Popen(
                argv,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            self._current_proc = proc
            assert proc.stdout is not None
            for raw_line in proc.stdout:
                line = raw_line.rstrip()
                lines.append(line)
                if line:
                    self.log.emit(f"Processing: {label}: {line}")
                    if "[raw2mp4]" in line or "VERIFY" in line or "FRAME_" in line:
                        self.status.emit(f"{label}: {line[:220]}")
                if self._cancelled and proc.poll() is None:
                    try:
                        os.killpg(proc.pid, signal.SIGTERM)
                    except Exception:
                        proc.terminate()
            rc = proc.wait()
            return subprocess.CompletedProcess(argv, rc, "\n".join(lines))
        finally:
            if proc is not None and self._current_proc is proc:
                self._current_proc = None

    def _storage_spec(self) -> str:
        return f"{self.upload_user}@{self.upload_host}"

    def _storage_ssh_argv(self) -> List[str]:
        return ["-p", self.upload_port] if self.upload_port else []

    def _storage_rsync_argv(self) -> List[str]:
        return ["-e", f"ssh -p {self.upload_port}"] if self.upload_port else []

    def _storage_shell_vars(self) -> str:
        return f"""
STORAGE={_q(self._storage_spec())}
STORAGE_PORT={_q(self.upload_port)}
SSH_ARGS=()
if [[ -n "$STORAGE_PORT" ]]; then
  SSH_ARGS=(-p "$STORAGE_PORT")
fi
RSYNC_RSH="ssh"
if [[ -n "$STORAGE_PORT" ]]; then
  RSYNC_RSH="ssh -p $STORAGE_PORT"
fi
""".strip()

    def _remote_base_snippet(self, job: PipelineJob) -> str:
        return f"""
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
SESSION={_q(job.session)}
CAM={_q(job.cam)}
ROOT={_q(self.remote_sessions_root)}
PROCESSED_SUBDIR={_q(self.processed_subdir)}
DIR="$ROOT/$SESSION/$CAM"
if [[ ! -d "$DIR" ]]; then
  echo "NO_SESSION_DIR $DIR"
  exit 20
fi
RAW=$(find "$DIR" -maxdepth 1 -type f -name '*_0000.cbrraw' | sort | head -n 1)
if [[ -z "$RAW" ]]; then
  echo "NO_CBRRAW $DIR"
  exit 21
fi
PREFIX="${{RAW%_0000.cbrraw}}"
BASE="$(basename "$PREFIX")"
META="${{PREFIX}}.metadata.yaml"
if [[ ! -f "$META" ]]; then
  META_BY_CAM=$(find "$DIR" -maxdepth 1 -type f -name "${{CAM}}_*.metadata.yaml" | sort | head -n 1)
  if [[ -n "$META_BY_CAM" ]]; then
    META="$META_BY_CAM"
  fi
fi
if [[ ! -f "$META" ]]; then
  META_ANY=$(find "$DIR" -maxdepth 1 -type f -name '*.metadata.yaml' | sort | head -n 1)
  if [[ -n "$META_ANY" ]]; then
    META="$META_ANY"
  fi
fi
PROC_DIR="$DIR/$PROCESSED_SUBDIR"
mkdir -p "$PROC_DIR"
MP4="$PROC_DIR/${{BASE}}.mp4"
AUDIT="$PROC_DIR/${{BASE}}.audit.csv"
AUDIT_STDOUT="$PROC_DIR/${{BASE}}.audit.stdout.txt"
INFO="$PROC_DIR/${{BASE}}.raw_info.txt"
CONVERT_STDOUT="$PROC_DIR/${{BASE}}.convert.stdout.txt"
CONVERT_ENV="$PROC_DIR/${{BASE}}.convert.env"
VERIFY="$PROC_DIR/${{BASE}}.verify.env"
if [[ ! -f "$META" ]]; then
  META=none
else
  echo "METADATA_USING $META"
fi
""".strip()

    def _processed_base_snippet(self, job: PipelineJob) -> str:
        """Snippet for steps that operate on processed outputs after raws may be deleted."""
        return f"""
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
SESSION={_q(job.session)}
CAM={_q(job.cam)}
ROOT={_q(self.remote_sessions_root)}
PROCESSED_SUBDIR={_q(self.processed_subdir)}
DIR="$ROOT/$SESSION/$CAM"
if [[ ! -d "$DIR" ]]; then
  echo "NO_SESSION_DIR $DIR"
  exit 20
fi
PROC_DIR="$DIR/$PROCESSED_SUBDIR"
if [[ ! -d "$PROC_DIR" ]]; then
  echo "NO_PROCESSED_DIR $PROC_DIR"
  exit 22
fi
BASE=""
VERIFY_OK=$(find "$PROC_DIR" -maxdepth 1 -type f -name '*.VERIFY_OK' | sort | head -n 1)
if [[ -n "$VERIFY_OK" ]]; then
  BASE="$(basename "${{VERIFY_OK%.VERIFY_OK}}")"
fi
if [[ -z "$BASE" ]]; then
  PROCESS_ENV=$(find "$PROC_DIR" -maxdepth 1 -type f -name '*.process.env' | sort | head -n 1)
  if [[ -n "$PROCESS_ENV" ]]; then
    BASE="$(basename "${{PROCESS_ENV%.process.env}}")"
  fi
fi
if [[ -z "$BASE" ]]; then
  MP4_FIRST=$(find "$PROC_DIR" -maxdepth 1 -type f -name '*.mp4' | sort | head -n 1)
  if [[ -n "$MP4_FIRST" ]]; then
    BASE="$(basename "${{MP4_FIRST%.mp4}}")"
  fi
fi
if [[ -z "$BASE" ]]; then
  echo "NO_PROCESSED_BASE $PROC_DIR"
  exit 23
fi
RAW=""
PREFIX="$DIR/$BASE"
META="$PROC_DIR/${{BASE}}.metadata.yaml"
MP4="$PROC_DIR/${{BASE}}.mp4"
AUDIT="$PROC_DIR/${{BASE}}.audit.csv"
AUDIT_STDOUT="$PROC_DIR/${{BASE}}.audit.stdout.txt"
INFO="$PROC_DIR/${{BASE}}.raw_info.txt"
CONVERT_STDOUT="$PROC_DIR/${{BASE}}.convert.stdout.txt"
CONVERT_ENV="$PROC_DIR/${{BASE}}.convert.env"
VERIFY="$PROC_DIR/${{BASE}}.verify.env"
echo "PROCESSED_BASE_USING $BASE"
""".strip()

    def _camera_session_snippet(self, job: PipelineJob) -> str:
        """Snippet for final cleanup of a camera's local session directory.

        This intentionally does not require raws or processed outputs. It is
        used after upload verification, and must also work if processed/ has
        already been removed by Delete local uploaded files.
        """
        return f"""
set -eo pipefail
SESSION={_q(job.session)}
CAM={_q(job.cam)}
ROOT={_q(self.remote_sessions_root)}
PROCESSED_SUBDIR={_q(self.processed_subdir)}
DIR="$ROOT/$SESSION/$CAM"
PROC_DIR="$DIR/$PROCESSED_SUBDIR"
if [[ ! -d "$DIR" ]]; then
  echo "NO_SESSION_DIR $DIR"
  exit 20
fi
""".strip()

    def _script_delete_camera_session_local(self, job: PipelineJob) -> str:
        return self._camera_session_snippet(job) + "\n\n" + """

RAW_COUNT=$(find "$DIR" -maxdepth 1 -type f -name '*.cbrraw' | wc -l)
if [[ "$RAW_COUNT" != "0" ]]; then
  echo "REFUSING_DELETE_CAMERA_SESSION_WITH_RAW_FILES $DIR raw_count=$RAW_COUNT"
  echo "Run Delete verified raws first."
  exit 80
fi

# If processed/ still exists, require upload verification before deleting it.
# If processed/ is already gone, this step is allowed so leftover metadata/png
# crumbs can be removed after Delete local uploaded files.
if [[ -d "$PROC_DIR" ]]; then
  UPLOAD_OK=$(find "$PROC_DIR" -maxdepth 1 -type f -name '*.UPLOAD_VERIFY_OK' | sort | head -n 1)
  if [[ -z "$UPLOAD_OK" ]]; then
    echo "REFUSING_DELETE_CAMERA_SESSION_WITHOUT_UPLOAD_VERIFY_OK $PROC_DIR"
    exit 81
  fi
fi

BYTES=$(du -sb "$DIR" | awk '{print $1}')
FILES=$(find "$DIR" -type f | wc -l)
PARENT=$(dirname "$DIR")
TOMBSTONE="$PARENT/${CAM}.LOCAL_CAMERA_SESSION_DELETED"
cat > "$TOMBSTONE" <<EOF
SESSION=$SESSION
CAM=$CAM
DIR=$DIR
BYTES=$BYTES
FILES=$FILES
DELETED_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
rm -rf "$DIR"
echo "LOCAL_CAMERA_SESSION_DELETED $CAM bytes=$BYTES files=$FILES dir=$DIR"
""".strip()

    def _script_info(self, job: PipelineJob) -> str:
        return self._remote_base_snippet(job) + "\n\n" + """

ros2 run cambuffer_recorder_ng raw_rolling_info "$RAW" > "$INFO" 2>&1
echo "RAW_INFO_WRITTEN $INFO"
""".strip()

    def _script_audit(self, job: PipelineJob) -> str:
        return self._remote_base_snippet(job) + "\n\n" + f"""

AUDIT_RC=0
ros2 run cambuffer_recorder_ng raw_rolling_audit "$RAW" "$AUDIT" {_q(self.fps)} {_q(self.audit_threshold_frames)} 0 > "$AUDIT_STDOUT" 2>&1 || AUDIT_RC=$?
if [[ "$AUDIT_RC" != "0" && "$AUDIT_RC" != "3" ]]; then
  cat "$AUDIT_STDOUT"
  exit "$AUDIT_RC"
fi
cat > "$PROC_DIR/${{BASE}}.audit.env" <<EOF
SESSION=$SESSION
CAM=$CAM
RAW=$RAW
AUDIT=$AUDIT
AUDIT_STDOUT=$AUDIT_STDOUT
AUDIT_RC=$AUDIT_RC
FPS={self.fps}
AUDIT_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
echo "AUDIT_WRITTEN $AUDIT audit_rc=$AUDIT_RC"
""".strip()

    def _script_process(self, job: PipelineJob) -> str:
        return self._remote_base_snippet(job) + "\n\n" + f"""

CONVERT_RC=0
set +e
ros2 run cambuffer_recorder_ng raw_rolling_to_mp4 "$RAW" "$MP4" 0 {_q(self.fps)} "$META" {_q(self.r_gain)} {_q(self.g_gain)} {_q(self.b_gain)} {_q(self.gamma)} 2>&1 | tee "$CONVERT_STDOUT"
CONVERT_RC=${{PIPESTATUS[0]}}
set -e
if [[ "$CONVERT_RC" != "0" ]]; then
  exit "$CONVERT_RC"
fi
CONVERT_WRITTEN_FRAMES=$(grep -Eo '\\[raw2mp4\\] wrote [0-9]+ frames' "$CONVERT_STDOUT" | tail -n 1 | awk '{{print $3}}')
cat > "$CONVERT_ENV" <<EOF
SESSION=$SESSION
CAM=$CAM
RAW=$RAW
MP4=$MP4
CONVERT_STDOUT=$CONVERT_STDOUT
CONVERT_RC=$CONVERT_RC
CONVERT_WRITTEN_FRAMES=$CONVERT_WRITTEN_FRAMES
CONVERT_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
AUDIT_RC=0
ros2 run cambuffer_recorder_ng raw_rolling_audit "$RAW" "$AUDIT" {_q(self.fps)} {_q(self.audit_threshold_frames)} 0 > "$AUDIT_STDOUT" 2>&1 || AUDIT_RC=$?
if [[ "$AUDIT_RC" != "0" && "$AUDIT_RC" != "3" ]]; then
  cat "$AUDIT_STDOUT"
  exit "$AUDIT_RC"
fi
ros2 run cambuffer_recorder_ng raw_rolling_info "$RAW" > "$INFO" 2>&1 || true
if [[ "$META" != "none" ]]; then
  cp -f "$META" "$PROC_DIR/${{BASE}}.metadata.yaml"
fi
cat > "$PROC_DIR/${{BASE}}.process.env" <<EOF
SESSION=$SESSION
CAM=$CAM
RAW=$RAW
MP4=$MP4
AUDIT=$AUDIT
AUDIT_RC=$AUDIT_RC
CONVERT_STDOUT=$CONVERT_STDOUT
CONVERT_WRITTEN_FRAMES=$CONVERT_WRITTEN_FRAMES
FPS={self.fps}
PROCESSED_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
echo "PROCESSED $MP4 frames=${{CONVERT_WRITTEN_FRAMES:-unknown}} audit_rc=$AUDIT_RC"
""".strip()

    def _script_verify(self, job: PipelineJob) -> str:
        return self._remote_base_snippet(job) + "\n\n" + """

if [[ ! -s "$MP4" ]]; then
  echo "MISSING_MP4 $MP4"
  exit 30
fi
if [[ ! -s "$AUDIT" ]]; then
  echo "MISSING_AUDIT $AUDIT"
  exit 31
fi
RAW_FRAMES=$(grep -E '^# total_frames:' "$AUDIT" | tail -n 1 | awk '{print $3}')
if [[ -z "$RAW_FRAMES" ]]; then
  echo "FRAME_COUNT_UNKNOWN raw=$RAW_FRAMES"
  exit 32
fi
CONVERT_WRITTEN=""
if [[ -s "$CONVERT_ENV" ]]; then
  CONVERT_WRITTEN=$(grep -E '^CONVERT_WRITTEN_FRAMES=' "$CONVERT_ENV" | tail -n 1 | cut -d= -f2-)
fi
if [[ -z "$CONVERT_WRITTEN" && -s "$CONVERT_STDOUT" ]]; then
  CONVERT_WRITTEN=$(grep -Eo '\\[raw2mp4\\] wrote [0-9]+ frames' "$CONVERT_STDOUT" | tail -n 1 | awk '{print $3}')
fi
MP4_FRAMES=$(ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames -of default=nokey=1:noprint_wrappers=1 "$MP4" | tail -n 1)
MP4_PACKETS=$(ffprobe -v error -count_packets -select_streams v:0 -show_entries stream=nb_read_packets -of default=nokey=1:noprint_wrappers=1 "$MP4" | tail -n 1)
MP4_NB_FRAMES=$(ffprobe -v error -select_streams v:0 -show_entries stream=nb_frames -of default=nokey=1:noprint_wrappers=1 "$MP4" | tail -n 1)
FRAME_OK=0
VERIFY_WARNING=""
if [[ "$MP4_FRAMES" == "$RAW_FRAMES" ]]; then
  FRAME_OK=1
elif [[ "$MP4_PACKETS" == "$RAW_FRAMES" ]]; then
  FRAME_OK=1
  VERIFY_WARNING="nb_read_packets_matched_raw_frames mp4_frames=$MP4_FRAMES packets=$MP4_PACKETS"
elif [[ -n "$CONVERT_WRITTEN" && "$CONVERT_WRITTEN" == "$RAW_FRAMES" ]]; then
  FRAME_OK=1
  VERIFY_WARNING="converter_matched_raw_frames ffprobe_frames=$MP4_FRAMES packets=$MP4_PACKETS convert=$CONVERT_WRITTEN"
elif [[ "$MP4_FRAMES" =~ ^[0-9]+$ && "$RAW_FRAMES" =~ ^[0-9]+$ ]]; then
  DELTA=$((MP4_FRAMES - RAW_FRAMES))
  if (( DELTA < 0 )); then DELTA=$(( -DELTA )); fi
  if (( DELTA <= 1 )); then
    FRAME_OK=1
    VERIFY_WARNING="ffprobe_off_by_${DELTA}_accepted raw=$RAW_FRAMES mp4=$MP4_FRAMES packets=$MP4_PACKETS convert=${CONVERT_WRITTEN:-unknown}"
  fi
fi
if [[ "$FRAME_OK" != "1" ]]; then
  echo "FRAME_MISMATCH raw=$RAW_FRAMES mp4=$MP4_FRAMES packets=$MP4_PACKETS nb_frames=$MP4_NB_FRAMES convert=${CONVERT_WRITTEN:-unknown}"
  exit 33
fi
META_CHECK="$META"
if [[ "$META_CHECK" == "none" ]]; then
  META_CHECK="$PROC_DIR/${BASE}.metadata.yaml"
fi
if [[ ! -s "$META_CHECK" ]]; then
  echo "MISSING_METADATA $META_CHECK"
  exit 34
fi
PROC_METADATA="$PROC_DIR/${BASE}.metadata.yaml"
if [[ "$META_CHECK" != "$PROC_METADATA" ]]; then
  cp -f "$META_CHECK" "$PROC_METADATA"
  META_CHECK="$PROC_METADATA"
fi
MISSING=0
for KEY in mode camera.width camera.height camera.fps camera.pixel_format output.kind; do
  if ! grep -q "$KEY" "$META_CHECK"; then
    echo "MISSING_METADATA_KEY $KEY"
    MISSING=1
  fi
done
if [[ "$MISSING" != "0" ]]; then
  exit 35
fi
cat > "$VERIFY" <<EOF
VERIFY_OK=1
RAW_FRAMES=$RAW_FRAMES
MP4_FRAMES=$MP4_FRAMES
MP4_PACKETS=$MP4_PACKETS
MP4_NB_FRAMES=$MP4_NB_FRAMES
CONVERT_WRITTEN_FRAMES=$CONVERT_WRITTEN
VERIFY_WARNING=$VERIFY_WARNING
MP4_SIZE_BYTES=$(stat -c %s "$MP4")
AUDIT_SIZE_BYTES=$(stat -c %s "$AUDIT")
METADATA=$META_CHECK
VERIFIED_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
touch "$PROC_DIR/${BASE}.VERIFY_OK"
if [[ -n "$VERIFY_WARNING" ]]; then
  echo "VERIFY_OK_WITH_WARNING $CAM raw_frames=$RAW_FRAMES mp4_frames=$MP4_FRAMES packets=$MP4_PACKETS convert=${CONVERT_WRITTEN:-unknown} warning=$VERIFY_WARNING"
else
  echo "VERIFY_OK $CAM raw_frames=$RAW_FRAMES mp4_frames=$MP4_FRAMES packets=$MP4_PACKETS convert=${CONVERT_WRITTEN:-unknown}"
fi
""".strip()

    def _script_delete_raws(self, job: PipelineJob) -> str:
        return self._remote_base_snippet(job) + "\n\n" + """

if [[ ! -f "$PROC_DIR/${BASE}.VERIFY_OK" ]]; then
  echo "REFUSING_DELETE_RAW_WITHOUT_VERIFY_OK $PROC_DIR/${BASE}.VERIFY_OK"
  exit 40
fi
COUNT=$(find "$DIR" -maxdepth 1 -type f -name "${BASE}_*.cbrraw" | wc -l)
find "$DIR" -maxdepth 1 -type f -name "${BASE}_*.cbrraw" -delete
touch "$PROC_DIR/${BASE}.RAW_DELETED"
echo "RAW_DELETED $CAM count=$COUNT"
""".strip()

    def _script_upload(self, job: PipelineJob) -> str:
        upload_root = self.upload_root
        return self._processed_base_snippet(job) + "\n\n" + f"""

{self._storage_shell_vars()}
if [[ ! -f "$PROC_DIR/${{BASE}}.VERIFY_OK" ]]; then
  echo "REFUSING_UPLOAD_WITHOUT_VERIFY_OK $PROC_DIR/${{BASE}}.VERIFY_OK"
  exit 50
fi
DEST={_q(upload_root)}/$SESSION/$CAM/processed
ssh "${{SSH_ARGS[@]}}" "$STORAGE" "mkdir -p '$DEST'"
touch "$PROC_DIR/${{BASE}}.UPLOADED"
rsync -a --partial -e "$RSYNC_RSH" "$PROC_DIR/" "$STORAGE:$DEST/"
echo "UPLOADED $CAM to $STORAGE:$DEST"
""".strip()

    def _script_verify_upload(self, job: PipelineJob) -> str:
        upload_root = self.upload_root
        return self._processed_base_snippet(job) + "\n\n" + f"""

{self._storage_shell_vars()}
DEST={_q(upload_root)}/$SESSION/$CAM/processed
LOCAL_LIST=$(mktemp)
REMOTE_LIST=$(mktemp)
(cd "$PROC_DIR" && find . -type f ! -name '*.UPLOAD_VERIFY_OK' ! -name '*.upload_sizes.tsv' -printf '%P\t%s\n' | sort) > "$LOCAL_LIST"
ssh "${{SSH_ARGS[@]}}" "$STORAGE" "cd '$DEST' && find . -type f -printf '%P\t%s\n' | sort" > "$REMOTE_LIST"
if ! diff -u "$LOCAL_LIST" "$REMOTE_LIST"; then
  echo "UPLOAD_SIZE_VERIFY_FAILED $CAM"
  exit 60
fi
cp "$LOCAL_LIST" "$PROC_DIR/${{BASE}}.upload_sizes.tsv"
touch "$PROC_DIR/${{BASE}}.UPLOAD_VERIFY_OK"
echo "UPLOAD_VERIFY_OK $CAM"
""".strip()

    def _script_delete_uploaded_local(self, job: PipelineJob) -> str:
        return self._processed_base_snippet(job) + "\n\n" + """

if [[ ! -f "$PROC_DIR/${BASE}.UPLOAD_VERIFY_OK" ]]; then
  echo "REFUSING_DELETE_PROCESSED_WITHOUT_UPLOAD_VERIFY_OK $PROC_DIR/${BASE}.UPLOAD_VERIFY_OK"
  exit 70
fi
BYTES=$(du -sb "$PROC_DIR" | awk '{print $1}')
rm -rf "$PROC_DIR"
echo "LOCAL_PROCESSED_DELETED $CAM bytes=$BYTES"
""".strip()

    def _script_for_step(self, job: PipelineJob, step: str) -> str:
        if step == "process":
            return self._script_process(job)
        if step == "info":
            return self._script_info(job)
        if step == "audit":
            return self._script_audit(job)
        if step == "verify":
            return self._script_verify(job)
        if step == "delete_raws":
            return self._script_delete_raws(job)
        if step == "upload":
            return self._script_upload(job)
        if step == "verify_upload":
            return self._script_verify_upload(job)
        if step == "delete_uploaded_local":
            return self._script_delete_uploaded_local(job)
        if step == "delete_camera_session_local":
            return self._script_delete_camera_session_local(job)
        raise ValueError(f"unknown processing step: {step}")

    def _append_manifest(self, session: str, cam: str, step: str, ok: bool, detail: str) -> None:
        path = self.base_dir / session / self.manifest_name
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists():
            path.write_text("timestamp_utc\tsession\tcamera\tstep\tok\tdetail\n", encoding="utf-8")
        clean_detail = " ".join(str(detail).split())[:800]
        with path.open("a", encoding="utf-8") as f:
            f.write(
                f"{datetime.utcnow().isoformat(timespec='seconds')}Z\t{session}\t{cam}\t{step}\t{int(ok)}\t{clean_detail}\n"
            )

    def _run_camera_step(self, job: PipelineJob, step: str) -> bool:
        self.log.emit(f"Processing: {job.session}/{job.cam}: {step} on {job.host}")
        script = self._script_for_step(job, step)
        try:
            proc = self._ssh_streaming(job.host, script, label=f"{job.session}/{job.cam} {step}")
        except subprocess.TimeoutExpired:
            msg = f"TIMEOUT during {step}"
            self.log.emit(f"Processing: {job.session}/{job.cam}: {msg}")
            self._append_manifest(job.session, job.cam, step, False, msg)
            return False

        out = proc.stdout or ""
        ok = proc.returncode == 0
        if ok:
            self.log.emit(f"Processing: {job.session}/{job.cam}: {step} OK: {_tail(out, 4)}")
        else:
            self.log.emit(f"Processing: {job.session}/{job.cam}: {step} FAIL rc={proc.returncode}: {_tail(out, 8)}")
        self._append_manifest(job.session, job.cam, step, ok, _tail(out, 8))
        return ok

    def _upload_session_level_files(self, session: str) -> bool:
        local_dir = self.base_dir / session
        if not local_dir.is_dir():
            msg = f"NO_LOCAL_SESSION_DIR {local_dir}"
            self.log.emit(f"Processing: {session}/tmill: upload_session FAIL: {msg}")
            self._append_manifest(session, "tmill", "upload_session", False, msg)
            return False

        storage = self._storage_spec()
        remote_dir = f"{self.upload_root}/{session}"
        upload_items: List[Path] = []
        for name in ["session.yaml", self.manifest_name]:
            p = local_dir / name
            if p.exists():
                upload_items.append(p)
        for subdir in [self.thumbnails_subdir, "rosbag"]:
            p = local_dir / subdir
            if p.exists():
                upload_items.append(p)

        if not upload_items:
            msg = f"NO_SESSION_LEVEL_FILES {local_dir}"
            self.log.emit(f"Processing: {session}/tmill: upload_session skipped: {msg}")
            self._append_manifest(session, "tmill", "upload_session", False, msg)
            return False

        try:
            mkdir = self._run_local(["ssh"] + self._storage_ssh_argv() + [storage, "mkdir", "-p", remote_dir], timeout_s=60)
            if mkdir.returncode != 0:
                raise RuntimeError(_tail(mkdir.stdout, 4))
            argv = ["rsync", "-a", "--partial"] + self._storage_rsync_argv() + [str(p) for p in upload_items] + [f"{storage}:{remote_dir}/"]
            proc = self._run_local(argv, timeout_s=1800)
        except Exception as exc:
            msg = str(exc)
            self.log.emit(f"Processing: {session}/tmill: upload_session FAIL: {msg}")
            self._append_manifest(session, "tmill", "upload_session", False, msg)
            return False

        ok = proc.returncode == 0
        if ok:
            self.log.emit(f"Processing: {session}/tmill: upload_session OK to {storage}:{remote_dir}")
        else:
            self.log.emit(f"Processing: {session}/tmill: upload_session FAIL rc={proc.returncode}: {_tail(proc.stdout, 8)}")
        self._append_manifest(session, "tmill", "upload_session", ok, _tail(proc.stdout, 8))
        return ok


    def _delete_local_session_tree(self, session: str) -> bool:
        """Delete the controller/local copy of an uploaded session.

        This refuses to remove local raw binaries. The raw cleanup path remains
        Delete verified raws, so the final session cleanup cannot accidentally
        discard unverified raw acquisition files.
        """
        local_dir = self.base_dir / session
        storage = self._storage_spec()
        remote_dir = f"{self.upload_root}/{session}"

        if not local_dir.exists():
            msg = f"LOCAL_SESSION_ALREADY_GONE {local_dir}"
            self.log.emit(f"Processing: {session}/tmill: delete_session_local OK: {msg}")
            self._append_manifest(session, "tmill", "delete_session_local", True, msg)
            return True
        if not local_dir.is_dir():
            msg = f"LOCAL_SESSION_NOT_DIRECTORY {local_dir}"
            self.log.emit(f"Processing: {session}/tmill: delete_session_local FAIL: {msg}")
            self._append_manifest(session, "tmill", "delete_session_local", False, msg)
            return False

        raw_examples = sorted(local_dir.rglob("*.cbrraw"))[:5]
        if raw_examples:
            msg = "REFUSING_DELETE_SESSION_WITH_RAW_FILES " + " ".join(str(p) for p in raw_examples)
            self.log.emit(f"Processing: {session}/tmill: delete_session_local FAIL: {msg}")
            self._append_manifest(session, "tmill", "delete_session_local", False, msg)
            return False

        # Require that the session-level copy exists on storage before deleting
        # the local folder. Camera processed files are guarded by per-camera
        # UPLOAD_VERIFY_OK checks before their directories can be removed.
        check_cmd = f"test -s {_q(remote_dir + '/session.yaml')}"
        check = self._run_local(["ssh"] + self._storage_ssh_argv() + [storage, check_cmd], timeout_s=60)
        if check.returncode != 0:
            msg = f"REFUSING_DELETE_SESSION_STORAGE_MISSING {storage}:{remote_dir}/session.yaml {_tail(check.stdout, 4)}"
            self.log.emit(f"Processing: {session}/tmill: delete_session_local FAIL: {msg}")
            self._append_manifest(session, "tmill", "delete_session_local", False, msg)
            return False

        file_count = 0
        byte_count = 0
        for p in local_dir.rglob("*"):
            if p.is_file():
                file_count += 1
                try:
                    byte_count += p.stat().st_size
                except OSError:
                    pass

        report = local_dir / "LOCAL_SESSION_DELETED.txt"
        report.write_text(
            "\n".join([
                f"SESSION={session}",
                f"LOCAL_DIR={local_dir}",
                f"STORAGE={storage}:{remote_dir}",
                f"CAMERA_MAP={' '.join(self.cameras)}",
                f"BYTES={byte_count}",
                f"FILES={file_count}",
                f"DELETED_UTC={datetime.utcnow().isoformat(timespec='seconds')}Z",
                "",
            ]),
            encoding="utf-8",
        )
        self._append_manifest(
            session,
            "tmill",
            "delete_session_local",
            True,
            f"DELETE_READY bytes={byte_count} files={file_count} storage={storage}:{remote_dir}",
        )

        upload_items = [report]
        manifest = local_dir / self.manifest_name
        if manifest.exists():
            upload_items.append(manifest)
        argv = ["rsync", "-a", "--partial"] + self._storage_rsync_argv() + [str(p) for p in upload_items] + [f"{storage}:{remote_dir}/"]
        proc = self._run_local(argv, timeout_s=300)
        if proc.returncode != 0:
            msg = f"DELETE_REPORT_UPLOAD_FAILED rc={proc.returncode}: {_tail(proc.stdout, 8)}"
            self.log.emit(f"Processing: {session}/tmill: delete_session_local FAIL: {msg}")
            self._append_manifest(session, "tmill", "delete_session_local", False, msg)
            return False

        shutil.rmtree(local_dir)
        self.log.emit(f"Processing: {session}/tmill: delete_session_local OK: deleted {local_dir} bytes={byte_count} files={file_count}")
        return True


class ProcessingPanel(QtWidgets.QWidget):
    """Processing tab for pilot-day thumbnail, conversion, verification, and upload."""

    log_line = QtCore.Signal(str)

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self._thread: Optional[QtCore.QThread] = None
        self._worker: Optional[QtCore.QObject] = None
        cfg = _load_processing_config().get("processing", {})
        conv = cfg.get("conversion", {}) if isinstance(cfg.get("conversion"), dict) else {}
        upload = cfg.get("upload", {}) if isinstance(cfg.get("upload"), dict) else {}

        self.processed_subdir = str(cfg.get("processed_subdir", "processed"))
        self.thumbnails_subdir = str(cfg.get("thumbnails_subdir", "thumbnails"))
        self.manifest_name = str(cfg.get("manifest_name", "processing_manifest.tsv"))
        self.audit_threshold_frames = float(conv.get("audit_threshold_frames", 1.5))

        self.base_dir_edit = QtWidgets.QLineEdit(str(cfg.get("local_sessions_root", Path.home() / "camera_sessions")))
        self.remote_root_edit = QtWidgets.QLineEdit(str(cfg.get("remote_sessions_root", "/home/spencelab/camera_sessions")))
        cameras = cfg.get("cameras", ["cam1", "cam2", "cam3", "cam4", "cam5"])
        self.cameras_edit = QtWidgets.QLineEdit(" ".join(str(x) for x in cameras))

        self.fps_spin = self._gain_spin(0.0)
        self.fps_spin.setMaximum(10000.0)
        self.fps_spin.setSingleStep(1.0)
        self.fps_spin.setValue(float(conv.get("fps", 5.0)))
        self.r_spin = self._gain_spin(float(conv.get("r_gain", 1.23)))
        self.g_spin = self._gain_spin(float(conv.get("g_gain", 1.0)))
        self.b_spin = self._gain_spin(float(conv.get("b_gain", 1.60)))
        self.gamma_spin = self._gain_spin(float(conv.get("gamma", 1.0)))
        self.gamma_spin.setMinimum(0.05)
        self.gamma_spin.setMaximum(5.0)

        self.upload_host_edit = QtWidgets.QLineEdit(str(upload.get("host", "gpu2")))
        self.upload_user_edit = QtWidgets.QLineEdit(str(upload.get("user", "spencelab")))
        self.upload_port_edit = QtWidgets.QLineEdit(str(upload.get("port", "")))
        self.upload_port_edit.setMaximumWidth(80)
        self.upload_root_edit = QtWidgets.QLineEdit(str(upload.get("root", "/zfstank3/storage/camera_sessions_uploads")))

        self.load_config_btn = QtWidgets.QPushButton("Load processing.yaml")
        self.refresh_btn = QtWidgets.QPushButton("Refresh sessions")
        self.create_btn = QtWidgets.QPushButton("Create and copy thumbnails")
        self.process_btn = QtWidgets.QPushButton("Process raws")
        self.info_btn = QtWidgets.QPushButton("Run raw info")
        self.audit_btn = QtWidgets.QPushButton("Run raw audit")
        self.info_audit_btn = QtWidgets.QPushButton("Info + audit")
        self.verify_btn = QtWidgets.QPushButton("Verify processed")
        self.delete_raws_btn = QtWidgets.QPushButton("Delete verified raws")
        self.upload_btn = QtWidgets.QPushButton("Upload processed")
        self.verify_upload_btn = QtWidgets.QPushButton("Verify upload")
        self.delete_uploaded_local_btn = QtWidgets.QPushButton("Delete local uploaded files")
        self.delete_uploaded_session_local_btn = QtWidgets.QPushButton("Delete uploaded session copies")
        self.process_verify_btn = QtWidgets.QPushButton("Process + verify")
        self.upload_verify_btn = QtWidgets.QPushButton("Upload + verify")
        self.process_to_upload_btn = QtWidgets.QPushButton("Process + verify + upload")
        self.cancel_btn = QtWidgets.QPushButton("Cancel")
        self.cancel_btn.setEnabled(False)

        self.session_list = QtWidgets.QListWidget()
        self.session_list.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.session_list.setMinimumHeight(220)

        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(0)

        self.status_label = QtWidgets.QLabel("Ready.")
        self.status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)

        self._build_layout()

        self.load_config_btn.clicked.connect(self.load_processing_yaml)
        self.refresh_btn.clicked.connect(self.refresh_sessions)
        self.create_btn.clicked.connect(self.create_thumbnails)
        self.process_btn.clicked.connect(lambda: self.run_pipeline("process"))
        self.info_btn.clicked.connect(lambda: self.run_pipeline("info"))
        self.audit_btn.clicked.connect(lambda: self.run_pipeline("audit"))
        self.info_audit_btn.clicked.connect(lambda: self.run_pipeline("info_audit"))
        self.verify_btn.clicked.connect(lambda: self.run_pipeline("verify"))
        self.delete_raws_btn.clicked.connect(lambda: self.run_pipeline("delete_raws"))
        self.upload_btn.clicked.connect(lambda: self.run_pipeline("upload"))
        self.verify_upload_btn.clicked.connect(lambda: self.run_pipeline("verify_upload"))
        self.delete_uploaded_local_btn.clicked.connect(lambda: self.run_pipeline("delete_uploaded_local"))
        self.delete_uploaded_session_local_btn.clicked.connect(lambda: self.run_pipeline("delete_uploaded_session_local"))
        self.process_verify_btn.clicked.connect(lambda: self.run_pipeline("process_verify"))
        self.upload_verify_btn.clicked.connect(lambda: self.run_pipeline("upload_verify"))
        self.process_to_upload_btn.clicked.connect(lambda: self.run_pipeline("process_verify_upload"))
        self.cancel_btn.clicked.connect(self.cancel)
        QtCore.QTimer.singleShot(0, self.refresh_sessions)

    def _gain_spin(self, value: float) -> QtWidgets.QDoubleSpinBox:
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(0.0, 10.0)
        spin.setDecimals(3)
        spin.setSingleStep(0.05)
        spin.setValue(value)
        return spin

    def _build_layout(self) -> None:
        top = QtWidgets.QFormLayout()
        top.addRow("Local sessions root", self.base_dir_edit)
        top.addRow("Remote camera sessions root", self.remote_root_edit)
        top.addRow("Camera map", self.cameras_edit)

        wb = QtWidgets.QHBoxLayout()
        wb.addWidget(QtWidgets.QLabel("FPS"))
        wb.addWidget(self.fps_spin)
        wb.addWidget(QtWidgets.QLabel("R"))
        wb.addWidget(self.r_spin)
        wb.addWidget(QtWidgets.QLabel("G"))
        wb.addWidget(self.g_spin)
        wb.addWidget(QtWidgets.QLabel("B"))
        wb.addWidget(self.b_spin)
        wb.addWidget(QtWidgets.QLabel("Gamma"))
        wb.addWidget(self.gamma_spin)
        wb.addStretch(1)
        top.addRow("Conversion", wb)

        upload = QtWidgets.QHBoxLayout()
        upload.addWidget(QtWidgets.QLabel("user"))
        upload.addWidget(self.upload_user_edit)
        upload.addWidget(QtWidgets.QLabel("host"))
        upload.addWidget(self.upload_host_edit)
        upload.addWidget(QtWidgets.QLabel("port"))
        upload.addWidget(self.upload_port_edit)
        upload.addWidget(QtWidgets.QLabel("root"))
        upload.addWidget(self.upload_root_edit, stretch=1)
        top.addRow("Storage upload", upload)

        buttons1 = QtWidgets.QHBoxLayout()
        buttons1.addWidget(self.load_config_btn)
        buttons1.addWidget(self.refresh_btn)
        buttons1.addWidget(self.create_btn)
        buttons1.addWidget(self.process_btn)
        buttons1.addWidget(self.info_btn)
        buttons1.addWidget(self.audit_btn)
        buttons1.addWidget(self.info_audit_btn)
        buttons1.addWidget(self.verify_btn)
        buttons1.addWidget(self.delete_raws_btn)
        buttons1.addStretch(1)

        buttons2 = QtWidgets.QHBoxLayout()
        buttons2.addWidget(self.upload_btn)
        buttons2.addWidget(self.verify_upload_btn)
        buttons2.addWidget(self.delete_uploaded_local_btn)
        buttons2.addWidget(self.delete_uploaded_session_local_btn)
        buttons2.addStretch(1)

        buttons3 = QtWidgets.QHBoxLayout()
        buttons3.addWidget(self.process_verify_btn)
        buttons3.addWidget(self.upload_verify_btn)
        buttons3.addWidget(self.process_to_upload_btn)
        buttons3.addWidget(self.cancel_btn)
        buttons3.addStretch(1)

        hint = QtWidgets.QLabel(
            "Camera map examples: cam1@local for one-box testing, or cam1@cam1 cam2@cam2 ... for MERB. "
            "Heavy processing runs on each camera host. Cameras upload their processed files directly to storage. "
            "tmill uploads session.yaml, thumbnails, processing_manifest.tsv, and rosbag/ if present. "
            "Run raw info/audit writes non-destructive diagnostics into each camera processed/ folder. "
            "Delete buttons require prior VERIFY_OK / UPLOAD_VERIFY_OK sentinel files. "
            "Delete uploaded session copies removes selected local session folders after upload verification and refuses if raw binaries remain."
        )
        hint.setWordWrap(True)

        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addLayout(top)
        layout.addLayout(buttons1)
        layout.addLayout(buttons2)
        layout.addLayout(buttons3)
        layout.addWidget(hint)
        layout.addWidget(QtWidgets.QLabel("Sessions"))
        layout.addWidget(self.session_list, stretch=1)
        layout.addWidget(self.progress)
        layout.addWidget(self.status_label)
        self.setLayout(layout)

    def _base_dir(self) -> Path:
        return Path(self.base_dir_edit.text().strip() or "~/camera_sessions").expanduser()

    def _camera_names(self) -> List[str]:
        names = [x.strip() for x in self.cameras_edit.text().replace(",", " ").split()]
        return [x for x in names if x]

    def _apply_processing_config_to_widgets(self, cfg: Dict[str, Any]) -> None:
        """Overwrite visible Processing tab settings from a processing.yaml dict."""
        conv = cfg.get("conversion", {}) if isinstance(cfg.get("conversion"), dict) else {}
        upload = cfg.get("upload", {}) if isinstance(cfg.get("upload"), dict) else {}

        self.processed_subdir = str(cfg.get("processed_subdir", "processed"))
        self.thumbnails_subdir = str(cfg.get("thumbnails_subdir", "thumbnails"))
        self.manifest_name = str(cfg.get("manifest_name", "processing_manifest.tsv"))
        self.audit_threshold_frames = float(conv.get("audit_threshold_frames", 1.5))

        self.base_dir_edit.setText(str(cfg.get("local_sessions_root", Path.home() / "camera_sessions")))
        self.remote_root_edit.setText(str(cfg.get("remote_sessions_root", "/home/spencelab/camera_sessions")))

        cameras = cfg.get("cameras", ["cam1", "cam2", "cam3", "cam4", "cam5"])
        if isinstance(cameras, str):
            camera_text = cameras
        else:
            camera_text = " ".join(str(x) for x in cameras)
        self.cameras_edit.setText(camera_text)

        self.fps_spin.setValue(float(conv.get("fps", 5.0)))
        self.r_spin.setValue(float(conv.get("r_gain", 1.23)))
        self.g_spin.setValue(float(conv.get("g_gain", 1.0)))
        self.b_spin.setValue(float(conv.get("b_gain", 1.60)))
        self.gamma_spin.setValue(float(conv.get("gamma", 1.0)))

        self.upload_host_edit.setText(str(upload.get("host", "gpu2")))
        self.upload_user_edit.setText(str(upload.get("user", "spencelab")))
        self.upload_port_edit.setText(str(upload.get("port", "")))
        self.upload_root_edit.setText(str(upload.get("root", "/zfstank3/storage/camera_sessions_uploads")))

    @QtCore.Slot()
    def load_processing_yaml(self) -> None:
        path = _default_processing_config_path()
        cfg = _load_processing_config().get("processing", {})
        if not isinstance(cfg, dict):
            self.status_label.setText(f"processing config did not contain a processing: map: {path}")
            return
        self._apply_processing_config_to_widgets(cfg)
        self.refresh_sessions()
        msg = f"Processing: loaded settings from {path}"
        self.status_label.setText(msg)
        self.log_line.emit(msg)

    def _set_busy(self, busy: bool) -> None:
        for widget in [
            self.load_config_btn, self.refresh_btn, self.create_btn, self.process_btn, self.info_btn,
            self.audit_btn, self.info_audit_btn, self.verify_btn,
            self.delete_raws_btn, self.upload_btn, self.verify_upload_btn,
            self.delete_uploaded_local_btn, self.delete_uploaded_session_local_btn,
            self.process_verify_btn, self.upload_verify_btn, self.process_to_upload_btn,
        ]:
            widget.setEnabled(not busy)
        self.cancel_btn.setEnabled(busy)

    @QtCore.Slot()
    def refresh_sessions(self) -> None:
        base = self._base_dir()
        self.session_list.clear()
        sessions = []
        if base.is_dir():
            for child in sorted(base.iterdir(), key=lambda p: p.name):
                if child.is_dir() and (child / "session.yaml").exists():
                    sessions.append(child.name)
        for name in sessions:
            item = QtWidgets.QListWidgetItem(name)
            item.setSelected(True)
            self.session_list.addItem(item)
        self.status_label.setText(f"Found {len(sessions)} local sessions under {base}")

    def _selected_sessions(self) -> List[str]:
        return [item.text() for item in self.session_list.selectedItems()]

    @QtCore.Slot()
    def create_thumbnails(self) -> None:
        if self._thread is not None:
            return
        sessions = self._selected_sessions()
        if not sessions:
            self.status_label.setText("No sessions selected.")
            return
        cameras = self._camera_names()
        if not cameras:
            self.status_label.setText("No camera hosts/nodes configured.")
            return

        total = len(sessions) * len(cameras)
        self.progress.setRange(0, total)
        self.progress.setValue(0)
        self.status_label.setText(f"Creating thumbnails for {len(sessions)} sessions x {len(cameras)} cameras...")
        self._set_busy(True)

        self._thread = QtCore.QThread(self)
        self._worker = ThumbnailWorker(
            base_dir=self._base_dir(),
            remote_sessions_root=self.remote_root_edit.text().strip() or "/home/spencelab/camera_sessions",
            sessions=sessions,
            cameras=cameras,
            r_gain=float(self.r_spin.value()),
            g_gain=float(self.g_spin.value()),
            b_gain=float(self.b_spin.value()),
            gamma=float(self.gamma_spin.value()),
        )
        self._start_worker(self._worker, self._worker.run, self._worker.finished, self._on_thumbnail_finished)

    def run_pipeline(self, action: str) -> None:
        if self._thread is not None:
            return
        sessions = self._selected_sessions()
        if not sessions:
            self.status_label.setText("No sessions selected.")
            return
        cameras = self._camera_names()
        if not cameras:
            self.status_label.setText("No camera hosts/nodes configured.")
            return

        if action == "delete_uploaded_session_local":
            session_text = ", ".join(sessions[:5])
            if len(sessions) > 5:
                session_text += f", ... ({len(sessions)} total)"
            reply = QtWidgets.QMessageBox.warning(
                self,
                "Delete uploaded session copies?",
                "This permanently deletes the selected local session folder(s) and camera-host session folder(s) after upload checks.\n\n"
                "It refuses to continue if .cbrraw files remain.\n\n"
                f"Sessions: {session_text}",
                QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.Cancel,
                QtWidgets.QMessageBox.StandardButton.Cancel,
            )
            if reply != QtWidgets.QMessageBox.StandardButton.Yes:
                self.status_label.setText("Session cleanup cancelled.")
                return

        self.progress.setRange(0, max(1, len(sessions) * len(cameras)))
        self.progress.setValue(0)
        self.status_label.setText(f"Running {action} for {len(sessions)} sessions x {len(cameras)} cameras...")
        self._set_busy(True)

        self._thread = QtCore.QThread(self)
        self._worker = PipelineWorker(
            action=action,
            base_dir=self._base_dir(),
            remote_sessions_root=self.remote_root_edit.text().strip() or "/home/spencelab/camera_sessions",
            sessions=sessions,
            cameras=cameras,
            fps=float(self.fps_spin.value()),
            r_gain=float(self.r_spin.value()),
            g_gain=float(self.g_spin.value()),
            b_gain=float(self.b_spin.value()),
            gamma=float(self.gamma_spin.value()),
            audit_threshold_frames=self.audit_threshold_frames,
            processed_subdir=self.processed_subdir,
            thumbnails_subdir=self.thumbnails_subdir,
            manifest_name=self.manifest_name,
            upload_user=self.upload_user_edit.text().strip(),
            upload_host=self.upload_host_edit.text().strip(),
            upload_port=self.upload_port_edit.text().strip(),
            upload_root=self.upload_root_edit.text().strip(),
        )
        self._start_worker(self._worker, self._worker.run, self._worker.finished, self._on_pipeline_finished)

    def _start_worker(self, worker: QtCore.QObject, start_slot, finished_signal, finished_slot) -> None:
        worker.moveToThread(self._thread)
        self._thread.started.connect(start_slot)
        worker.log.connect(self.log_line.emit)  # type: ignore[attr-defined]
        if hasattr(worker, "status"):
            worker.status.connect(self.status_label.setText)  # type: ignore[attr-defined]
        worker.progress.connect(self._on_progress)  # type: ignore[attr-defined]
        finished_signal.connect(finished_slot)
        finished_signal.connect(self._thread.quit)
        self._thread.finished.connect(self._thread.deleteLater)
        self._thread.start()

    @QtCore.Slot()
    def cancel(self) -> None:
        if self._worker is not None and hasattr(self._worker, "cancel"):
            self._worker.cancel()  # type: ignore[attr-defined]
            self.cancel_btn.setEnabled(False)
            self.status_label.setText("Cancelling active local/SSH command...")

    @QtCore.Slot(int, int)
    def _on_progress(self, done: int, total: int) -> None:
        self.progress.setRange(0, max(1, total))
        self.progress.setValue(done)
        self.status_label.setText(f"Processing jobs: {done}/{total}")

    @QtCore.Slot(int, int)
    def _on_thumbnail_finished(self, ok: int, done: int) -> None:
        self.status_label.setText(f"Done: copied {ok}/{done} thumbnails.")
        self._set_busy(False)
        self._worker = None
        self._thread = None

    @QtCore.Slot(str, int, int)
    def _on_pipeline_finished(self, action: str, ok: int, done: int) -> None:
        self.status_label.setText(f"Done: {action}: {ok}/{done} step(s) OK.")
        self._set_busy(False)
        self._worker = None
        self._thread = None
