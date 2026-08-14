"""Processing tab for camera_control.

MERB pilot tools:
  - scan local ~/camera_sessions for sessions
  - ask cam1-cam5 to extract first-frame PNG thumbnails from matching .cbrraw files
  - ask camera hosts to convert raw rolling files to MP4 in-place
  - verify MP4 frame counts against raw audit output
  - delete verified raw binaries
  - permanently delete selected test sessions from tmill + configured camera hosts
  - upload processed camera files directly from camera hosts to storage
  - upload tmill session-level files to the same storage session directory

This module intentionally avoids rclpy. It is just PySide6 + subprocess so the
main camera cockpit does not grow another tentacle.
"""

from __future__ import annotations

import concurrent.futures
import os
import signal
import shlex
import shutil
import socket
import subprocess
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from PySide6 import QtCore, QtWidgets

try:
    from multicam_sync_audit import audit_session as run_multicam_sync_audit
    MULTICAM_SYNC_AUDIT_AVAILABLE = True
except Exception:
    run_multicam_sync_audit = None
    MULTICAM_SYNC_AUDIT_AVAILABLE = False


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _configs_dir() -> Path:
    return _repo_root() / "configs"


def _default_processing_config_path() -> Path:
    env = os.environ.get("CAMERA_CONTROL_PROCESSING_YAML", "").strip()
    if env:
        return Path(env).expanduser()
    return _configs_dir() / "processing.yaml"


def _processing_config_candidates() -> List[Path]:
    """Return available processing profile YAMLs, de-duplicated.

    Convention:
      configs/processing*.yaml

    Examples:
      processing.yaml
      processing_merb.yaml
      processing_ros2test.yaml
      processing_dimroom.yaml
    """
    candidates: List[Path] = []

    env = os.environ.get("CAMERA_CONTROL_PROCESSING_YAML", "").strip()
    if env:
        candidates.append(Path(env).expanduser())

    cfg_dir = _configs_dir()
    if cfg_dir.is_dir():
        candidates.extend(sorted(cfg_dir.glob("processing*.yaml")))

    candidates.append(_default_processing_config_path())

    out: List[Path] = []
    seen = set()
    for path in candidates:
        key = str(path.expanduser())
        if key not in seen:
            out.append(path.expanduser())
            seen.add(key)
    return out


def _remembered_processing_config_path() -> Path:
    settings = QtCore.QSettings("SpenceLab", "camera_control")
    remembered = str(settings.value("processing/config_path", "") or "").strip()
    if remembered:
        return Path(remembered).expanduser()
    return _default_processing_config_path()


def _load_processing_config(path: Optional[Path] = None) -> Dict[str, Any]:
    """Load optional processing YAML, returning permissive defaults if absent."""
    defaults: Dict[str, Any] = {
        "processing": {
            "local_sessions_root": str(Path.home() / "camera_sessions"),
            "remote_sessions_root": "/home/spencelab/camera_sessions",
            "cameras": ["cam1", "cam2", "cam3", "cam4", "cam5"],
            "max_parallel_cameras": 5,
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
                "max_parallel_uploads": 5,
            },
            "rosbag": {
                "subdir": "rosbag",
                "upload_from": "tmill",
            },
        }
    }

    cfg_path = (path or _default_processing_config_path()).expanduser()
    if not cfg_path.exists():
        return defaults

    try:
        import yaml  # type: ignore
        loaded = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
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

        # Thumbnails use the processing/profile RGB white-balance gains, but
        # intentionally leave gamma neutral for now. Full MP4 processing still
        # uses the GUI/profile gamma value through PipelineWorker.
        self.gamma = 1.0
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
        # Keep annotation function outside the f-string so bash/Python braces do
        # not become a brace confetti incident.
        annotate_script = r"""
annotate_thumbnail() {
  local png="$1"
  local raw="$2"
  local meta="$3"
  local r_gain="$4"
  local g_gain="$5"
  local b_gain="$6"

  python3 - "$png" "$raw" "$meta" "$r_gain" "$g_gain" "$b_gain" <<'PYANNOTATE' || true
import sys
from pathlib import Path

png_path = Path(sys.argv[1])
raw_path = Path(sys.argv[2])
meta_path = sys.argv[3]
r_gain = sys.argv[4]
g_gain = sys.argv[5]
b_gain = sys.argv[6]

try:
    import cv2  # type: ignore
    import numpy as np  # type: ignore
except Exception as exc:
    print(f"THUMBNAIL_ANNOTATE_SKIPPED missing cv2/numpy: {exc}")
    raise SystemExit(0)

img = cv2.imread(str(png_path), cv2.IMREAD_UNCHANGED)
if img is None:
    print(f"THUMBNAIL_ANNOTATE_SKIPPED could not read {png_path}")
    raise SystemExit(0)

if img.ndim == 2:
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
elif img.shape[2] == 4:
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

h, w = img.shape[:2]
banner_h = max(54, min(96, h // 9))
overlay = img.copy()
cv2.rectangle(overlay, (0, 0), (w, banner_h), (0, 0, 0), -1)
img = cv2.addWeighted(overlay, 0.62, img, 0.38, 0)

base = raw_path.name
if base.endswith("_0000.cbrraw"):
    base = base[:-len("_0000.cbrraw")]

line1 = base
line2 = f"WB R={r_gain} G={g_gain} B={b_gain}"
if meta_path and meta_path != "none":
    line2 += f"  meta={Path(meta_path).name}"

font = cv2.FONT_HERSHEY_SIMPLEX
scale1 = max(0.42, min(0.72, w / 2600.0))
scale2 = max(0.38, min(0.62, w / 3000.0))
thick = max(1, int(round(w / 1200.0)))

def put_fit(text, y, scale, color=(255, 255, 255)):
    max_width = max(80, w - 24)
    rendered = text
    while len(rendered) > 8:
        (tw, _), _ = cv2.getTextSize(rendered, font, scale, thick)
        if tw <= max_width:
            break
        rendered = rendered[:-9] + "..."
    cv2.putText(img, rendered, (12, y), font, scale, color, thick, cv2.LINE_AA)

put_fit(line1, int(banner_h * 0.43), scale1)
put_fit(line2, int(banner_h * 0.82), scale2, (220, 255, 220))

hist_w = max(180, min(320, w // 5))
hist_h = max(44, min(banner_h - 14, 78))
x0 = max(0, w - hist_w - 12)
y0 = 7
x1 = x0 + hist_w
y1 = y0 + hist_h

cv2.rectangle(img, (x0, y0), (x1, y1), (18, 18, 18), -1)
cv2.rectangle(img, (x0, y0), (x1, y1), (230, 230, 230), 1)

roi = img[banner_h:, :, :]
if roi.size == 0:
    roi = img

b, g, r = cv2.split(roi[:, :, :3])
is_mono = np.array_equal(b, g) and np.array_equal(g, r)

def draw_hist(channel, color):
    hist = cv2.calcHist([channel], [0], None, [256], [0, 256]).flatten()
    if hist.max() > 0:
        hist = hist / hist.max()
    pts = []
    for i in range(256):
        x = int(x0 + 1 + (i / 255.0) * (hist_w - 2))
        y = int(y1 - 2 - hist[i] * (hist_h - 5))
        pts.append((x, y))
    for a, bpt in zip(pts[:-1], pts[1:]):
        cv2.line(img, a, bpt, color, 1, cv2.LINE_AA)

if is_mono:
    gray = cv2.cvtColor(roi[:, :, :3], cv2.COLOR_BGR2GRAY)
    draw_hist(gray, (240, 240, 240))
    label = "mono hist"
else:
    draw_hist(b, (255, 130, 90))
    draw_hist(g, (120, 255, 120))
    draw_hist(r, (100, 130, 255))
    label = "RGB hist"

cv2.line(img, (x1 - 2, y0 + 1), (x1 - 2, y1 - 1), (255, 255, 255), 1)
cv2.putText(img, "255", (x1 - 34, y1 - 5), font, 0.35, (255, 255, 255), 1, cv2.LINE_AA)
cv2.putText(img, label, (x0 + 5, y0 + 14), font, 0.38, (255, 255, 255), 1, cv2.LINE_AA)

if cv2.imwrite(str(png_path), img):
    print(f"THUMBNAIL_ANNOTATED {png_path}")
else:
    print(f"THUMBNAIL_ANNOTATE_SKIPPED could not write {png_path}")
PYANNOTATE
}
"""
        r = shlex.quote(f"{self.r_gain:.6g}")
        g = shlex.quote(f"{self.g_gain:.6g}")
        b = shlex.quote(f"{self.b_gain:.6g}")
        gamma = "1.0"  # thumbnails: RGB WB yes, gamma neutral for now

        return annotate_script + "\n" + f"""
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
mapfile -t RAW_STARTS < <(find "$DIR" -maxdepth 1 -type f -name '*_0000.cbrraw' | sort)
if [[ "${{#RAW_STARTS[@]}}" == "0" ]]; then
  echo "NO_CBRRAW $DIR"
  exit 21
fi
RAW="${{RAW_STARTS[-1]}}"
echo "THUMBNAIL_USING $(basename "$RAW") of ${{#RAW_STARTS[@]}} raw starts"

META="${{RAW%_0000.cbrraw}}.metadata.yaml"
if [[ ! -f "$META" ]]; then
  BASE="$(basename "${{RAW%_0000.cbrraw}}")"
  SESSION_BASE="$(printf '%s\n' "$BASE" | sed -E 's/_[0-9]{{8}}T[0-9]{{6}}Z.*$//')"
  META_BY_BASE="$DIR/${{SESSION_BASE}}.metadata.yaml"
  if [[ -f "$META_BY_BASE" ]]; then
    META="$META_BY_BASE"
  fi
fi
if [[ ! -f "$META" ]]; then
  META_BY_CAM=$(find "$DIR" -maxdepth 1 -type f -name "${{CAM}}_*.metadata.yaml" ! -name '*dump[0-9]*.metadata.yaml' | sort | head -n 1)
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

echo "THUMBNAIL_WB R={r} G={g} B={b} gamma=$gamma"
ros2 run cambuffer_recorder_ng raw_rolling_to_mp4 "$RAW" "$PNG" 1 0 "$META" {r} {g} {b} "$gamma"

# Annotation is nice-to-have; never let it turn a good PNG into a failed job.
annotate_thumbnail "$PNG" "$RAW" "$META" {r} {g} {b} || true

# Keep the PNG path as the final line; _process_one copies this.
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
        max_parallel_cameras: int = 5,
        max_parallel_uploads: int = 5,
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
        self.max_parallel_cameras = max(1, int(max_parallel_cameras or 1))
        self.max_parallel_uploads = max(1, int(max_parallel_uploads or 1))
        self._cancelled = False
        self._proc_lock = threading.Lock()
        self._manifest_lock = threading.Lock()
        self._active_procs: List[subprocess.Popen[str]] = []
        self._current_proc: Optional[subprocess.Popen[str]] = None

    @QtCore.Slot()
    def cancel(self) -> None:
        self._cancelled = True
        with self._proc_lock:
            procs = list(self._active_procs)
            if self._current_proc is not None and self._current_proc not in procs:
                procs.append(self._current_proc)

        if not procs:
            self.log.emit("Processing: cancellation requested; no active process is currently registered")
            return

        for proc in procs:
            if proc.poll() is not None:
                continue
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                self.log.emit(f"Processing: sent SIGTERM to active process group pid={proc.pid}")
            except Exception as exc:
                try:
                    proc.terminate()
                    self.log.emit(f"Processing: terminate fallback sent to pid={proc.pid}")
                except Exception:
                    self.log.emit(f"Processing: could not terminate active process pid={proc.pid}: {exc}")

    def run(self) -> None:
        action_plan = self._expand_action(self.action)
        safe_eod = self.action == "process_verify_upload_delete_trim"
        trim_after_delete = self.action in {"process_verify_upload_delete_trim", "delete_uploaded_session_local"}
        jobs = [PipelineJob(session=s, cam=spec.cam, host=spec.host) for s in self.sessions for spec in self.camera_specs]
        per_cam_steps = [
            a for a in action_plan
            if a not in (
                "upload_session",
                "delete_session_local",
                "delete_session_force_local",
                "multicam_sync",
            )
        ]
        multicam_passes = 1 if "multicam_sync" in action_plan else 0
        session_upload_passes = 2 if "upload_session" in action_plan else 0
        session_delete_passes = 1 if "delete_session_local" in action_plan else 0
        force_session_delete_passes = 1 if "delete_session_force_local" in action_plan else 0
        safe_cleanup_passes = len(self.sessions) * (len(self.camera_specs) + 1) if safe_eod else 0
        trim_specs = self._unique_trim_specs() if trim_after_delete else []
        total = (
            len(self.sessions)
            * (session_upload_passes + session_delete_passes + force_session_delete_passes + multicam_passes)
            + len(jobs) * len(per_cam_steps)
            + safe_cleanup_passes
            + len(trim_specs)
        )
        total = max(1, total)
        done = 0
        ok = 0
        any_camera_session_deleted = False

        self.log.emit(
            f"Processing: action '{self.action}' starting for {len(self.sessions)} sessions x "
            f"{len(self.camera_specs)} cameras; parallel cameras={self.max_parallel_cameras}, "
            f"parallel uploads={self.max_parallel_uploads}"
        )

        for session in self.sessions:
            if self._cancelled:
                break

            session_jobs = [j for j in jobs if j.session == session]
            force_remote_delete_ok = True
            uploaded_camera_cleanup_ok = True
            safe_eod_chain_ok = True

            if "upload_session" in action_plan:
                upload_ok = self._upload_session_level_files(session)
                if upload_ok:
                    ok += 1
                elif safe_eod:
                    safe_eod_chain_ok = False
                done += 1
                self.progress.emit(done, total)

            for step in per_cam_steps:
                if self._cancelled:
                    break
                done_before = done
                ok_before = ok
                done, ok = self._run_camera_step_group(session, session_jobs, step, done, ok, total)
                attempted = done - done_before
                succeeded = ok - ok_before
                step_group_ok = attempted == len(session_jobs) and succeeded == attempted
                if safe_eod and not step_group_ok:
                    safe_eod_chain_ok = False
                if step == "delete_session_force":
                    force_remote_delete_ok = step_group_ok
                if step == "delete_camera_session_local":
                    uploaded_camera_cleanup_ok = step_group_ok
                    if succeeded > 0:
                        any_camera_session_deleted = True

            if (not self._cancelled) and "multicam_sync" in action_plan:
                multicam_ok = self._run_multicam_sync_audit(session)
                if multicam_ok:
                    ok += 1
                elif safe_eod:
                    safe_eod_chain_ok = False
                done += 1
                self.progress.emit(done, total)

            if (not self._cancelled) and "upload_session" in action_plan:
                upload_ok = self._upload_session_level_files(session)
                if upload_ok:
                    ok += 1
                elif safe_eod:
                    safe_eod_chain_ok = False
                done += 1
                self.progress.emit(done, total)

            if safe_eod and not self._cancelled:
                if safe_eod_chain_ok:
                    self.status.emit(f"{session}: archive chain passed; deleting verified local copies")
                    done_before = done
                    ok_before = ok
                    done, ok = self._run_camera_step_group(
                        session, session_jobs, "delete_camera_session_local", done, ok, total
                    )
                    attempted = done - done_before
                    succeeded = ok - ok_before
                    uploaded_camera_cleanup_ok = attempted == len(session_jobs) and succeeded == attempted
                    if succeeded > 0:
                        any_camera_session_deleted = True

                    if uploaded_camera_cleanup_ok:
                        if self._delete_local_session_tree(session):
                            ok += 1
                    else:
                        msg = (
                            "KEEPING_TMILL_SESSION_CAMERA_CLEANUP_FAILED "
                            "one or more camera-host session folders were not deleted"
                        )
                        self.log.emit(f"Processing: {session}/tmill: delete_session_local FAIL: {msg}")
                        self.status.emit(f"{session}: camera cleanup failed; tmill copy kept so cleanup can be retried")
                        self._append_manifest(session, "tmill", "delete_session_local", False, msg)
                    done += 1
                    self.progress.emit(done, total)
                else:
                    msg = (
                        "SAFE_EOD_DELETE_SKIPPED prior process/verify/upload/multicam step failed; "
                        "no local session data deleted"
                    )
                    self.log.emit(f"Processing: {session}: {msg}")
                    self.status.emit(f"{session}: verification chain failed; local data kept")
                    for _ in session_jobs:
                        done += 1
                        self.progress.emit(done, total)
                    done += 1
                    self.progress.emit(done, total)

            if (not self._cancelled) and "delete_session_local" in action_plan:
                if "delete_camera_session_local" in action_plan and not uploaded_camera_cleanup_ok:
                    msg = (
                        "KEEPING_TMILL_SESSION_CAMERA_CLEANUP_FAILED "
                        "one or more camera-host session folders were not deleted"
                    )
                    self.log.emit(f"Processing: {session}/tmill: delete_session_local FAIL: {msg}")
                    self.status.emit(f"{session}: camera cleanup failed; tmill copy kept so cleanup can be retried")
                    self._append_manifest(session, "tmill", "delete_session_local", False, msg)
                elif self._delete_local_session_tree(session):
                    ok += 1
                done += 1
                self.progress.emit(done, total)

            if (not self._cancelled) and "delete_session_force_local" in action_plan:
                if force_remote_delete_ok:
                    if self._delete_local_session_force(session):
                        ok += 1
                else:
                    self.log.emit(
                        f"Processing: {session}/tmill: delete_session_force_local FAIL: "
                        "keeping tmill session because one or more configured camera-host deletes failed"
                    )
                    self.status.emit(
                        f"{session}: camera-host delete failure; tmill copy kept so the deletion can be retried"
                    )
                done += 1
                self.progress.emit(done, total)

        if trim_after_delete and not self._cancelled:
            done, ok = self._run_trim_group(
                trim_specs,
                any_camera_session_deleted=any_camera_session_deleted,
                done=done,
                ok=ok,
                total=total,
            )

        if self._cancelled:
            self.log.emit(f"Processing: action '{self.action}' cancelled")
        self.log.emit(f"Processing: action '{self.action}' complete: {ok}/{done} step(s) OK")
        self.finished.emit(self.action, ok, done)

    def _unique_trim_specs(self) -> List[CameraSpec]:
        """Return one representative camera per unique host for post-delete TRIM."""
        out: List[CameraSpec] = []
        seen: set[str] = set()
        for spec in self.camera_specs:
            key = str(spec.host or "").strip().lower()
            if key in seen:
                continue
            seen.add(key)
            out.append(spec)
        return out

    def _trim_camera_host(self, spec: CameraSpec) -> bool:
        """Best-effort batch TRIM of the filesystem holding camera_sessions.

        This runs only after verified local camera-session deletion. It never
        participates in the archive/deletion safety gate. The sudo call is
        non-interactive by design, so a host without a narrowly configured
        NOPASSWD fstrim rule fails visibly instead of hanging the GUI.
        """
        root = self.remote_sessions_root
        script = f"""
set -eo pipefail
ROOT={_q(root)}
if ! command -v findmnt >/dev/null 2>&1; then
  echo "TRIM_FAILED host={spec.host} reason=findmnt_not_found"
  exit 90
fi
if ! command -v fstrim >/dev/null 2>&1; then
  echo "TRIM_FAILED host={spec.host} reason=fstrim_not_found"
  exit 91
fi
MOUNT=$(findmnt -n -o TARGET -T "$ROOT" | head -n 1)
if [[ -z "$MOUNT" ]]; then
  echo "TRIM_FAILED host={spec.host} reason=could_not_resolve_mount root=$ROOT"
  exit 92
fi
echo "TRIM_START host={spec.host} mount=$MOUNT root=$ROOT"
if OUTPUT=$(sudo -n fstrim -v "$MOUNT" 2>&1); then
  echo "$OUTPUT"
  echo "TRIM_OK host={spec.host} mount=$MOUNT"
else
  RC=$?
  echo "$OUTPUT"
  echo "TRIM_FAILED host={spec.host} mount=$MOUNT rc=$RC note=archive_delete_already_complete"
  exit "$RC"
fi
""".strip()
        self.log.emit(f"Processing: {spec.host}: trimming filesystem containing {root}")
        try:
            proc = self._ssh_streaming(spec.host, script, label=f"{spec.host} fstrim")
        except subprocess.TimeoutExpired:
            self.log.emit(
                f"Processing: {spec.host}: TRIM_FAILED timeout; archive/delete results are unaffected"
            )
            return False
        out = proc.stdout or ""
        if proc.returncode == 0:
            self.log.emit(f"Processing: {spec.host}: TRIM OK: {_tail(out, 4)}")
            return True
        self.log.emit(
            f"Processing: {spec.host}: TRIM FAILED rc={proc.returncode}: {_tail(out, 8)} "
            "(archive/delete results are unaffected)"
        )
        return False

    def _run_trim_group(
        self,
        specs: List[CameraSpec],
        *,
        any_camera_session_deleted: bool,
        done: int,
        ok: int,
        total: int,
    ) -> tuple[int, int]:
        """Run one post-delete fstrim per unique camera host, in parallel.

        TRIM remains maintenance only: failure never changes whether verified
        archive deletion was allowed. Waiting for the parallel group gives the
        GUI and persistent receipt a definitive result for every host.
        """
        if not specs:
            return done, ok

        if not any_camera_session_deleted:
            for spec in specs:
                self.log.emit(
                    f"Processing: {spec.host}: TRIM_SKIPPED no camera session was deleted during this action"
                )
                done += 1
                self.progress.emit(done, total)
            return done, ok

        max_workers = max(1, min(len(specs), self.max_parallel_cameras))
        self.status.emit(f"TRIM: launching {len(specs)} camera host(s), parallel={max_workers}")
        self.log.emit(f"Processing: TRIM launching {len(specs)} camera host(s), parallel={max_workers}")

        executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=max_workers,
            thread_name_prefix="processing_fstrim",
        )
        futures: Dict[concurrent.futures.Future[bool], CameraSpec] = {}
        try:
            for spec in specs:
                if self._cancelled:
                    break
                futures[executor.submit(self._trim_camera_host, spec)] = spec

            for future in concurrent.futures.as_completed(futures):
                spec = futures[future]
                try:
                    trim_ok = bool(future.result())
                except Exception as exc:
                    trim_ok = False
                    self.log.emit(
                        f"Processing: {spec.host}: TRIM_FAILED exception={exc} "
                        "note=archive_delete_already_complete"
                    )
                if trim_ok:
                    ok += 1
                done += 1
                self.progress.emit(done, total)
                if self._cancelled:
                    break
        finally:
            executor.shutdown(wait=True, cancel_futures=True)

        return done, ok

    def _parallel_limit_for_step(self, step: str) -> int:
        if step in ("upload", "verify_upload"):
            return self.max_parallel_uploads
        return self.max_parallel_cameras

    def _run_camera_step_group(
        self,
        session: str,
        jobs: List[PipelineJob],
        step: str,
        done: int,
        ok: int,
        total: int,
    ) -> tuple[int, int]:
        if not jobs:
            return done, ok

        max_workers = max(1, min(len(jobs), self._parallel_limit_for_step(step)))
        self.status.emit(f"{session}: {step}: launching {len(jobs)} camera job(s), parallel={max_workers}")
        self.log.emit(f"Processing: {session}: {step} launching {len(jobs)} camera job(s), parallel={max_workers}")

        if max_workers == 1:
            for job in jobs:
                if self._cancelled:
                    break
                if self._run_camera_step(job, step):
                    ok += 1
                done += 1
                self.progress.emit(done, total)
            return done, ok

        executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=max_workers,
            thread_name_prefix=f"processing_{step}",
        )
        futures: Dict[concurrent.futures.Future[bool], PipelineJob] = {}
        try:
            for job in jobs:
                if self._cancelled:
                    break
                futures[executor.submit(self._run_camera_step, job, step)] = job

            for future in concurrent.futures.as_completed(futures):
                job = futures[future]
                try:
                    step_ok = bool(future.result())
                except Exception as exc:
                    step_ok = False
                    msg = f"EXCEPTION during {step}: {exc}"
                    self.log.emit(f"Processing: {job.session}/{job.cam}: {step} FAIL: {msg}")
                    self._append_manifest(job.session, job.cam, step, False, msg)

                if step_ok:
                    ok += 1
                done += 1
                self.progress.emit(done, total)

                if self._cancelled:
                    break
        finally:
            executor.shutdown(wait=True, cancel_futures=True)

        return done, ok

    def _expand_action(self, action: str) -> List[str]:
        if action == "process":
            return ["process", "multicam_sync"]
        if action == "info":
            return ["info"]
        if action == "audit":
            return ["audit"]
        if action == "info_audit":
            return ["info", "audit", "multicam_sync"]
        if action == "multicam_sync":
            # Standalone GUI sync audit: generate/refresh the per-camera raw
            # info + audit CSV prerequisites before reconstructing alignment.
            return ["info", "audit", "multicam_sync"]
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
        if action == "delete_sessions":
            return ["delete_session_force", "delete_session_force_local"]
        if action == "process_verify":
            return ["process", "verify", "multicam_sync"]
        if action == "upload_verify":
            return ["upload_session", "upload", "verify_upload"]
        if action == "process_verify_upload":
            return ["process", "verify", "upload_session", "upload", "verify_upload", "multicam_sync"]
        if action == "process_verify_upload_delete_trim":
            # Cleanup and TRIM are intentionally handled as post-verification
            # phases in run(), after the final session-level upload pass.
            return ["process", "verify", "upload_session", "upload", "verify_upload", "multicam_sync"]
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
            with self._proc_lock:
                self._active_procs.append(proc)
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
            if proc is not None:
                with self._proc_lock:
                    try:
                        self._active_procs.remove(proc)
                    except ValueError:
                        pass
                    if self._current_proc is proc:
                        self._current_proc = self._active_procs[-1] if self._active_procs else None

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

    def _remote_common_snippet(self, job: PipelineJob) -> str:
        """Shared shell setup for camera steps that operate on raw start files.

        A single camera/session folder may contain multiple independent raw starts:
          - rolling:  BASE_0000.cbrraw, BASE_0001.cbrraw, ...
          - RAM dump: BASE_dump000001_HHMMSS_0000.cbrraw

        We enumerate every *_0000.cbrraw and treat each prefix as one unit.
        raw_rolling_to_mp4/raw_rolling_audit will follow numbered rolling chunks
        from that start prefix automatically.
        """
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
mkdir -p "$PROC_DIR"

metadata_for_prefix() {{
  local prefix="$1"
  local base
  base="$(basename "$prefix")"
  local meta="${{prefix}}.metadata.yaml"

  # RAM dumps usually have exact per-dump metadata.
  if [[ -f "$meta" ]]; then
    echo "$meta"
    return 0
  fi

  # Rolling runs usually have one camera/session metadata file without the
  # per-start timestamp suffix. Strip _YYYYMMDDTHHMMSSZ... to find it.
  local session_base
  session_base="$(printf '%s\n' "$base" | sed -E 's/_[0-9]{{8}}T[0-9]{{6}}Z.*$//')"
  meta="$DIR/${{session_base}}.metadata.yaml"
  if [[ -f "$meta" ]]; then
    echo "$meta"
    return 0
  fi

  # Prefer non-dump camera/session metadata before dump-specific metadata.
  meta=$(find "$DIR" -maxdepth 1 -type f -name "${{CAM}}_*.metadata.yaml" ! -name '*dump[0-9]*.metadata.yaml' | sort | head -n 1)
  if [[ -n "$meta" ]]; then
    echo "$meta"
    return 0
  fi

  meta=$(find "$DIR" -maxdepth 1 -type f -name '*.metadata.yaml' | sort | head -n 1)
  if [[ -n "$meta" ]]; then
    echo "$meta"
    return 0
  fi

  echo none
}}

audit_context_for_meta() {{
  local meta="$1"
  python3 - "$meta" <<'PYAUDITCTX'
import math
import re
import sys
from pathlib import Path

meta = sys.argv[1]
if meta == "none":
    print("0\tunknown\tmetadata_missing")
    raise SystemExit(0)

path = Path(meta)
if not path.is_file():
    print("0\tunknown\tmetadata_missing")
    raise SystemExit(0)

sections = {{}}
try:
    import yaml  # type: ignore
    doc = yaml.safe_load(path.read_text(encoding="utf-8")) or {{}}
    root = doc.get("cambuffer_recorder_ng", doc) if isinstance(doc, dict) else {{}}
    if isinstance(root, dict):
        for name in ("effective_settings", "requested_settings"):
            value = root.get(name)
            if isinstance(value, dict):
                sections[name] = value
except Exception:
    # The recorder writes these settings as a flat key/value map nested under
    # effective_settings/requested_settings.  This tiny fallback parser keeps
    # audit cadence recovery working even on a host without PyYAML.
    current = None
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = re.match(r"^\\s{{2}}(effective_settings|requested_settings):\\s*$", line)
        if m:
            current = m.group(1)
            sections.setdefault(current, {{}})
            continue
        if current is None:
            continue
        m = re.match(r"^\\s{{4}}([^:]+):\\s*(.*?)\\s*$", line)
        if m:
            sections[current][m.group(1).strip()] = m.group(2).strip().strip('"')
        elif line and not line.startswith("    "):
            current = None

def boolish(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        s = v.strip().lower()
        if s in ("true", "yes", "on", "1"):
            return True
        if s in ("false", "no", "off", "0"):
            return False
    return None

for section_name in ("effective_settings", "requested_settings"):
    settings = sections.get(section_name, {{}})
    if not isinstance(settings, dict):
        continue
    hw = boolish(settings.get("camera.hardware_trigger"))
    keys = (
        ("camera.expected_hardware_fps", "camera.fps", "fps")
        if hw is True
        else ("camera.fps", "fps", "camera.expected_hardware_fps")
    )
    for key in keys:
        try:
            fps = float(settings.get(key))
        except (TypeError, ValueError):
            continue
        if math.isfinite(fps) and fps > 0:
            hw_text = "true" if hw is True else "false" if hw is False else "unknown"
            print(f"{{fps:g}}\t{{hw_text}}\t{{section_name}}.{{key}}")
            raise SystemExit(0)

print("0\tunknown\tmetadata_keys_missing")
PYAUDITCTX
}}

set_current_raw() {{
  RAW="$1"
  PREFIX="${{RAW%_0000.cbrraw}}"
  BASE="$(basename "$PREFIX")"
  META="$(metadata_for_prefix "$PREFIX")"
  MP4="$PROC_DIR/${{BASE}}.mp4"
  AUDIT="$PROC_DIR/${{BASE}}.audit.csv"
  AUDIT_STDOUT="$PROC_DIR/${{BASE}}.audit.stdout.txt"
  INFO="$PROC_DIR/${{BASE}}.raw_info.txt"
  CONVERT_STDOUT="$PROC_DIR/${{BASE}}.convert.stdout.txt"
  CONVERT_ENV="$PROC_DIR/${{BASE}}.convert.env"
  VERIFY="$PROC_DIR/${{BASE}}.verify.env"
  if [[ "$META" != "none" ]]; then
    echo "METADATA_USING $META"
  fi
  IFS=$'\t' read -r AUDIT_FPS HARDWARE_TRIGGER AUDIT_FPS_SOURCE < <(audit_context_for_meta "$META")
  if [[ -z "$AUDIT_FPS" || "$AUDIT_FPS" == "0" ]]; then
    echo "AUDIT_EXPECTED_FPS_UNAVAILABLE base=$BASE meta=$META source=$AUDIT_FPS_SOURCE"
    exit 24
  fi
  echo "AUDIT_CONTEXT base=$BASE hardware_trigger=$HARDWARE_TRIGGER expected_fps=$AUDIT_FPS source=$AUDIT_FPS_SOURCE playback_fps={self.fps}"
}}

mapfile -t RAW_STARTS < <(find "$DIR" -maxdepth 1 -type f -name '*_0000.cbrraw' | sort)
if [[ "${{#RAW_STARTS[@]}}" == "0" ]]; then
  echo "NO_CBRRAW $DIR"
  exit 21
fi
echo "RAW_START_COUNT $CAM ${{#RAW_STARTS[@]}}"
""".strip()

    def _processed_dir_snippet(self, job: PipelineJob, *, require_processed: bool = True) -> str:
        """Shared shell setup for steps that operate on processed/ as a whole."""
        require_line = """
if [[ ! -d "$PROC_DIR" ]]; then
  echo "NO_PROCESSED_DIR $PROC_DIR"
  exit 22
fi
""" if require_processed else ""
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
{require_line}
""".strip()

    # Backward-compatible name for older call sites/patches.
    def _processed_base_snippet(self, job: PipelineJob) -> str:
        return self._processed_dir_snippet(job, require_processed=True) + "\n\n" + """
mapfile -t PROCESSED_BASES < <(find "$PROC_DIR" -maxdepth 1 -type f -name '*.mp4' -printf '%f\n' | sed 's/\\.mp4$//' | sort)
if [[ "${#PROCESSED_BASES[@]}" == "0" ]]; then
  echo "NO_PROCESSED_BASE $PROC_DIR"
  exit 23
fi
BASE="${PROCESSED_BASES[0]}"
RAW=""
PREFIX="$DIR/$BASE"
META="$PROC_DIR/${BASE}.metadata.yaml"
MP4="$PROC_DIR/${BASE}.mp4"
AUDIT="$PROC_DIR/${BASE}.audit.csv"
AUDIT_STDOUT="$PROC_DIR/${BASE}.audit.stdout.txt"
INFO="$PROC_DIR/${BASE}.raw_info.txt"
CONVERT_STDOUT="$PROC_DIR/${BASE}.convert.stdout.txt"
CONVERT_ENV="$PROC_DIR/${BASE}.convert.env"
VERIFY="$PROC_DIR/${BASE}.verify.env"
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

    def _script_delete_session_force(self, job: PipelineJob) -> str:
        """Permanently delete one selected session on a camera host.

        This intentionally ignores processing/upload sentinel state because it
        is for disposable test sessions. The path guards are deliberately
        strict: SESSION must be one basename and its canonical target must be a
        direct child of remote_sessions_root.
        """
        return f"""
set -eo pipefail
SESSION={_q(job.session)}
ROOT={_q(self.remote_sessions_root)}

case "$SESSION" in
  ""|"."|".."|*/*)
    echo "REFUSING_DANGEROUS_SESSION_NAME session=$SESSION"
    exit 90
    ;;
esac

ROOT_ABS=$(realpath -m -- "$ROOT")
DIR_ABS=$(realpath -m -- "$ROOT/$SESSION")
PARENT_ABS=$(dirname -- "$DIR_ABS")

if [[ -z "$ROOT_ABS" || "$ROOT_ABS" == "/" || "$PARENT_ABS" != "$ROOT_ABS" || "$DIR_ABS" == "$ROOT_ABS" ]]; then
  echo "REFUSING_DANGEROUS_SESSION_PATH root=$ROOT_ABS target=$DIR_ABS parent=$PARENT_ABS"
  exit 91
fi

if [[ ! -e "$DIR_ABS" ]]; then
  echo "SESSION_ALREADY_ABSENT $DIR_ABS"
  exit 0
fi

BYTES=$(du -sb -- "$DIR_ABS" 2>/dev/null | awk '{{print $1}}' || true)
FILES=$(find "$DIR_ABS" -type f 2>/dev/null | wc -l || true)
rm -rf --one-file-system -- "$DIR_ABS"

if [[ -e "$DIR_ABS" ]]; then
  echo "SESSION_DELETE_FAILED target=$DIR_ABS"
  exit 92
fi

echo "SESSION_DELETED target=$DIR_ABS bytes=${{BYTES:-unknown}} files=${{FILES:-unknown}}"
""".strip()

    def _script_delete_camera_session_local(self, job: PipelineJob) -> str:
        upload_root = self.upload_root
        return self._camera_session_snippet(job) + "\n\n" + f"""
{self._storage_shell_vars()}
UPLOAD_ROOT={_q(upload_root)}
REMOTE_SESSION="$UPLOAD_ROOT/$SESSION"
REMOTE_PROC="$REMOTE_SESSION/$CAM/$PROCESSED_SUBDIR"
LOCAL_UPLOAD_MARKER="$DIR/${{CAM}}.PROCESSED_UPLOAD_VERIFIED"

LOCAL_PROCESSED_UPLOAD_OK=0
if [[ -f "$PROC_DIR/.UPLOAD_VERIFY_OK" || -f "$LOCAL_UPLOAD_MARKER" ]]; then
  LOCAL_PROCESSED_UPLOAD_OK=1
fi

REMOTE_SESSION_OK=0
if ssh "${{SSH_ARGS[@]}}" "$STORAGE" "test -f '$REMOTE_SESSION/.SESSION_UPLOAD_VERIFY_OK'"; then
  REMOTE_SESSION_OK=1
fi

if [[ -d "$PROC_DIR" && "$LOCAL_PROCESSED_UPLOAD_OK" != "1" ]]; then
  echo "REFUSING_DELETE_CAMERA_SESSION_WITH_UNVERIFIED_PROCESSED_DIR $PROC_DIR"
  echo "Run Verify upload first, then Delete local uploaded files or Delete uploaded session copies."
  exit 81
fi

mapfile -t RAW_STARTS < <(find "$DIR" -maxdepth 1 -type f -name '*_0000.cbrraw' | sort)
RAW_START_COUNT=${{#RAW_STARTS[@]}}
TOTAL_RAW_FILES=$(find "$DIR" -maxdepth 1 -type f -name '*.cbrraw' | wc -l)

if [[ "$RAW_START_COUNT" != "0" ]]; then
  for RAW0 in "${{RAW_STARTS[@]}}"; do
    BASE="$(basename "${{RAW0%_0000.cbrraw}}")"
    LOCAL_VERIFY_OK=0
    REMOTE_VERIFY_OK=0

    if [[ -f "$PROC_DIR/${{BASE}}.VERIFY_OK" && -s "$PROC_DIR/${{BASE}}.mp4" ]]; then
      LOCAL_VERIFY_OK=1
    fi

    if [[ "$REMOTE_SESSION_OK" == "1" ]]; then
      if ssh "${{SSH_ARGS[@]}}" "$STORAGE" "test -f '$REMOTE_PROC/$BASE.VERIFY_OK' && test -s '$REMOTE_PROC/$BASE.mp4'"; then
        REMOTE_VERIFY_OK=1
      fi
    fi

    if [[ "$LOCAL_VERIFY_OK" != "1" && "$REMOTE_VERIFY_OK" != "1" ]]; then
      echo "REFUSING_DELETE_CAMERA_SESSION_RAW_NOT_VERIFIED base=$BASE local=$LOCAL_VERIFY_OK remote=$REMOTE_VERIFY_OK dir=$DIR"
      echo "Need local processed/${{BASE}}.VERIFY_OK or verified uploaded copy at $STORAGE:$REMOTE_PROC/${{BASE}}.VERIFY_OK"
      exit 80
    fi
  done

  TOTAL_DELETED=0
  for RAW0 in "${{RAW_STARTS[@]}}"; do
    BASE="$(basename "${{RAW0%_0000.cbrraw}}")"
    COUNT=$(find "$DIR" -maxdepth 1 -type f -name "${{BASE}}_*.cbrraw" | wc -l)
    find "$DIR" -maxdepth 1 -type f -name "${{BASE}}_*.cbrraw" -delete
    TOTAL_DELETED=$((TOTAL_DELETED + COUNT))
    echo "RAW_DELETED_DURING_CAMERA_SESSION_DELETE $CAM base=$BASE count=$COUNT"
  done
else
  TOTAL_DELETED=0
fi

REMAINING_RAW_COUNT=$(find "$DIR" -maxdepth 1 -type f -name '*.cbrraw' | wc -l)
if [[ "$REMAINING_RAW_COUNT" != "0" ]]; then
  echo "REFUSING_DELETE_CAMERA_SESSION_RAW_FILES_REMAIN $DIR raw_count=$REMAINING_RAW_COUNT"
  exit 82
fi

if [[ ! -d "$PROC_DIR" && "$LOCAL_PROCESSED_UPLOAD_OK" != "1" && "$REMOTE_SESSION_OK" != "1" ]]; then
  echo "REFUSING_DELETE_CAMERA_SESSION_WITHOUT_UPLOAD_EVIDENCE dir=$DIR storage=$STORAGE:$REMOTE_SESSION"
  exit 83
fi

BYTES=$(du -sb "$DIR" | awk '{{print $1}}')
FILES=$(find "$DIR" -type f | wc -l)
PARENT=$(dirname "$DIR")
TOMBSTONE="$PARENT/${{CAM}}.LOCAL_CAMERA_SESSION_DELETED"
cat > "$TOMBSTONE" <<EOF
SESSION=$SESSION
CAM=$CAM
DIR=$DIR
BYTES=$BYTES
FILES=$FILES
RAW_STARTS=$RAW_START_COUNT
RAW_FILES_BEFORE=$TOTAL_RAW_FILES
RAW_FILES_DELETED=$TOTAL_DELETED
LOCAL_PROCESSED_UPLOAD_OK=$LOCAL_PROCESSED_UPLOAD_OK
REMOTE_SESSION_OK=$REMOTE_SESSION_OK
STORAGE=$STORAGE
REMOTE_PROC=$REMOTE_PROC
DELETED_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
rm -rf "$DIR"
echo "LOCAL_CAMERA_SESSION_DELETED $CAM bytes=$BYTES files=$FILES raw_deleted=$TOTAL_DELETED dir=$DIR"
""".strip()

    def _script_info(self, job: PipelineJob) -> str:
        return self._remote_common_snippet(job) + "\n\n" + """
for RAW0 in "${RAW_STARTS[@]}"; do
  set_current_raw "$RAW0"
  echo "INFO_START $BASE"
  ros2 run cambuffer_recorder_ng raw_rolling_info "$RAW" > "$INFO" 2>&1
  echo "RAW_INFO_WRITTEN $INFO"
done
""".strip()

    def _script_audit(self, job: PipelineJob) -> str:
        return self._remote_common_snippet(job) + "\n\n" + f"""
for RAW0 in "${{RAW_STARTS[@]}}"; do
  set_current_raw "$RAW0"
  echo "AUDIT_START $BASE"
  AUDIT_RC=0
  ros2 run cambuffer_recorder_ng raw_rolling_audit "$RAW" "$AUDIT" "$AUDIT_FPS" {_q(self.audit_threshold_frames)} 0 > "$AUDIT_STDOUT" 2>&1 || AUDIT_RC=$?
  if [[ "$AUDIT_RC" != "0" && "$AUDIT_RC" != "3" ]]; then
    cat "$AUDIT_STDOUT"
    exit "$AUDIT_RC"
  fi
  if [[ "$META" != "none" ]]; then
    cp -f "$META" "$PROC_DIR/${{BASE}}.metadata.yaml"
  fi
  cat > "$PROC_DIR/${{BASE}}.audit.env" <<EOF
SESSION=$SESSION
CAM=$CAM
BASE=$BASE
RAW=$RAW
AUDIT=$AUDIT
AUDIT_STDOUT=$AUDIT_STDOUT
AUDIT_RC=$AUDIT_RC
PLAYBACK_FPS={self.fps}
AUDIT_EXPECTED_FPS=$AUDIT_FPS
HARDWARE_TRIGGER=$HARDWARE_TRIGGER
AUDIT_FPS_SOURCE=$AUDIT_FPS_SOURCE
AUDIT_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
  echo "AUDIT_WRITTEN $AUDIT audit_rc=$AUDIT_RC expected_fps=$AUDIT_FPS hardware_trigger=$HARDWARE_TRIGGER"
done
""".strip()

    def _script_process(self, job: PipelineJob) -> str:
        return self._remote_common_snippet(job) + "\n\n" + f"""
for RAW0 in "${{RAW_STARTS[@]}}"; do
  set_current_raw "$RAW0"
  echo "PROCESS_START $BASE"
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
BASE=$BASE
RAW=$RAW
MP4=$MP4
CONVERT_STDOUT=$CONVERT_STDOUT
CONVERT_RC=$CONVERT_RC
CONVERT_WRITTEN_FRAMES=$CONVERT_WRITTEN_FRAMES
CONVERT_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
  AUDIT_RC=0
  ros2 run cambuffer_recorder_ng raw_rolling_audit "$RAW" "$AUDIT" "$AUDIT_FPS" {_q(self.audit_threshold_frames)} 0 > "$AUDIT_STDOUT" 2>&1 || AUDIT_RC=$?
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
BASE=$BASE
RAW=$RAW
MP4=$MP4
AUDIT=$AUDIT
AUDIT_RC=$AUDIT_RC
CONVERT_STDOUT=$CONVERT_STDOUT
CONVERT_WRITTEN_FRAMES=$CONVERT_WRITTEN_FRAMES
FPS={self.fps}
PLAYBACK_FPS={self.fps}
AUDIT_EXPECTED_FPS=$AUDIT_FPS
HARDWARE_TRIGGER=$HARDWARE_TRIGGER
AUDIT_FPS_SOURCE=$AUDIT_FPS_SOURCE
PROCESSED_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
  echo "PROCESSED $MP4 frames=${{CONVERT_WRITTEN_FRAMES:-unknown}} audit_rc=$AUDIT_RC audit_expected_fps=$AUDIT_FPS"
done
""".strip()

    def _script_verify(self, job: PipelineJob) -> str:
        return self._processed_dir_snippet(job, require_processed=True) + "\n\n" + """
mapfile -t PROCESSED_BASES < <(find "$PROC_DIR" -maxdepth 1 -type f -name '*.mp4' -printf '%f\n' | sed 's/\\.mp4$//' | sort)
if [[ "${#PROCESSED_BASES[@]}" == "0" ]]; then
  echo "NO_MP4_TO_VERIFY $PROC_DIR"
  exit 30
fi
for BASE in "${PROCESSED_BASES[@]}"; do
  MP4="$PROC_DIR/${BASE}.mp4"
  AUDIT="$PROC_DIR/${BASE}.audit.csv"
  CONVERT_STDOUT="$PROC_DIR/${BASE}.convert.stdout.txt"
  CONVERT_ENV="$PROC_DIR/${BASE}.convert.env"
  META_CHECK="$PROC_DIR/${BASE}.metadata.yaml"
  VERIFY="$PROC_DIR/${BASE}.verify.env"
  echo "VERIFY_START $BASE"

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
    echo "FRAME_COUNT_UNKNOWN raw=$RAW_FRAMES base=$BASE"
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
    echo "FRAME_MISMATCH base=$BASE raw=$RAW_FRAMES mp4=$MP4_FRAMES packets=$MP4_PACKETS nb_frames=$MP4_NB_FRAMES convert=${CONVERT_WRITTEN:-unknown}"
    exit 33
  fi
  if [[ ! -s "$META_CHECK" ]]; then
    echo "MISSING_METADATA $META_CHECK"
    exit 34
  fi

  MISSING=0
  for KEY in mode camera.width camera.height camera.fps camera.pixel_format output.kind; do
    if ! grep -q "$KEY" "$META_CHECK"; then
      echo "MISSING_METADATA_KEY $KEY base=$BASE"
      MISSING=1
    fi
  done
  if [[ "$MISSING" != "0" ]]; then
    exit 35
  fi

  cat > "$VERIFY" <<EOF
VERIFY_OK=1
BASE=$BASE
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
    echo "VERIFY_OK_WITH_WARNING $CAM base=$BASE raw_frames=$RAW_FRAMES mp4_frames=$MP4_FRAMES packets=$MP4_PACKETS convert=${CONVERT_WRITTEN:-unknown} warning=$VERIFY_WARNING"
  else
    echo "VERIFY_OK $CAM base=$BASE raw_frames=$RAW_FRAMES mp4_frames=$MP4_FRAMES packets=$MP4_PACKETS convert=${CONVERT_WRITTEN:-unknown}"
  fi
done
""".strip()

    def _script_delete_raws(self, job: PipelineJob) -> str:
        return self._remote_common_snippet(job) + "\n\n" + """
TOTAL_DELETED=0
for RAW0 in "${RAW_STARTS[@]}"; do
  set_current_raw "$RAW0"
  if [[ ! -f "$PROC_DIR/${BASE}.VERIFY_OK" ]]; then
    echo "REFUSING_DELETE_RAW_WITHOUT_VERIFY_OK $PROC_DIR/${BASE}.VERIFY_OK"
    exit 40
  fi
  COUNT=$(find "$DIR" -maxdepth 1 -type f -name "${BASE}_*.cbrraw" | wc -l)
  find "$DIR" -maxdepth 1 -type f -name "${BASE}_*.cbrraw" -delete
  touch "$PROC_DIR/${BASE}.RAW_DELETED"
  TOTAL_DELETED=$((TOTAL_DELETED + COUNT))
  echo "RAW_DELETED $CAM base=$BASE count=$COUNT"
done
echo "RAW_DELETE_COMPLETE $CAM total_count=$TOTAL_DELETED"
""".strip()

    def _script_upload(self, job: PipelineJob) -> str:
        upload_root = self.upload_root
        return self._processed_dir_snippet(job, require_processed=True) + "\n\n" + f"""
{self._storage_shell_vars()}
mapfile -t MP4S < <(find "$PROC_DIR" -maxdepth 1 -type f -name '*.mp4' | sort)
if [[ "${{#MP4S[@]}}" != "0" ]]; then
  for MP4 in "${{MP4S[@]}}"; do
    BASE="$(basename "${{MP4%.mp4}}")"
    if [[ ! -f "$PROC_DIR/${{BASE}}.VERIFY_OK" ]]; then
      echo "REFUSING_UPLOAD_WITHOUT_VERIFY_OK $PROC_DIR/${{BASE}}.VERIFY_OK"
      exit 50
    fi
  done
fi
CAM_DEST={_q(upload_root)}/$SESSION/$CAM
DEST="$CAM_DEST/processed"
ssh "${{SSH_ARGS[@]}}" "$STORAGE" "mkdir -p '$DEST'"

# A fresh upload invalidates any older local upload-verification gate.
rm -f "$PROC_DIR/.UPLOAD_VERIFY_OK" "$PROC_DIR/.upload_sizes.tsv"
touch "$PROC_DIR/.UPLOADED"
rsync -a --partial -e "$RSYNC_RSH" "$PROC_DIR/" "$STORAGE:$DEST/"

# Acquisition/dump metadata lives beside processed/, not inside it. It is
# required archive state, so upload every camera-root metadata YAML. The
# first-frame thumbnail is optional because not every recording produces one.
mapfile -t ROOT_METADATA < <(find "$DIR" -maxdepth 1 -type f -name '*.metadata.yaml' | sort)
if [[ "${{#ROOT_METADATA[@]}}" == "0" ]]; then
  echo "REFUSING_UPLOAD_WITHOUT_ROOT_METADATA $DIR/*.metadata.yaml"
  exit 51
fi
rsync -a --partial -e "$RSYNC_RSH" "${{ROOT_METADATA[@]}}" "$STORAGE:$CAM_DEST/"
ROOT_THUMB="$DIR/${{CAM}}_first.png"
if [[ -f "$ROOT_THUMB" ]]; then
  rsync -a --partial -e "$RSYNC_RSH" "$ROOT_THUMB" "$STORAGE:$CAM_DEST/"
fi

echo "UPLOADED $CAM to $STORAGE:$CAM_DEST processed_files=$(find "$PROC_DIR" -maxdepth 1 -type f | wc -l) root_metadata=${{#ROOT_METADATA[@]}} thumbnail=$([[ -f "$ROOT_THUMB" ]] && echo yes || echo no)"
""".strip()

    def _script_verify_upload(self, job: PipelineJob) -> str:
        upload_root = self.upload_root
        return self._processed_dir_snippet(job, require_processed=True) + "\n\n" + f"""
{self._storage_shell_vars()}
CAM_DEST={_q(upload_root)}/$SESSION/$CAM
DEST="$CAM_DEST/processed"
LOCAL_LIST=$(mktemp)
REMOTE_LIST=$(mktemp)
trap 'rm -f "$LOCAL_LIST" "$REMOTE_LIST"' EXIT

# Verify processed payload plus camera-root scientific sidecars as one archive
# contract. Prefix processed entries so root files cannot collide by name.
(
  cd "$DIR"
  find "$PROCESSED_SUBDIR" -type f \\
    ! -name '*.UPLOAD_VERIFY_OK' ! -name '.UPLOAD_VERIFY_OK' \\
    ! -name '*.upload_sizes.tsv' ! -name '.upload_sizes.tsv' \\
    -printf '%P\t%s\n' | sed 's#^#processed/#'
  find . -maxdepth 1 -type f -name '*.metadata.yaml' -printf '%P\t%s\n'
  if [[ -f "${{CAM}}_first.png" ]]; then
    stat -c '%n\t%s' "${{CAM}}_first.png"
  fi
) | sort > "$LOCAL_LIST"

if ! grep -qE '^[^/]+\\.metadata\\.yaml[[:space:]]' "$LOCAL_LIST"; then
  echo "UPLOAD_VERIFY_MISSING_LOCAL_ROOT_METADATA $DIR/*.metadata.yaml"
  exit 59
fi

ssh "${{SSH_ARGS[@]}}" "$STORAGE" "
  set -eo pipefail
  cd '$CAM_DEST'
  {{
    find processed -type f \\
      ! -name '*.UPLOAD_VERIFY_OK' ! -name '.UPLOAD_VERIFY_OK' \\
      ! -name '*.upload_sizes.tsv' ! -name '.upload_sizes.tsv' \\
      -printf '%P\t%s\n' | sed 's#^#processed/#'
    find . -maxdepth 1 -type f -name '*.metadata.yaml' -printf '%P\t%s\n'
    if [[ -f '${{CAM}}_first.png' ]]; then
      stat -c '%n\t%s' '${{CAM}}_first.png'
    fi
  }} | sort
" > "$REMOTE_LIST"

if ! diff -u "$LOCAL_LIST" "$REMOTE_LIST"; then
  echo "UPLOAD_SIZE_VERIFY_FAILED $CAM (processed payload and/or root sidecars differ)"
  exit 60
fi
cp "$LOCAL_LIST" "$PROC_DIR/.upload_sizes.tsv"
touch "$PROC_DIR/.UPLOAD_VERIFY_OK"
if ! rsync -a --partial -e "$RSYNC_RSH" "$PROC_DIR/.UPLOAD_VERIFY_OK" "$PROC_DIR/.upload_sizes.tsv" "$STORAGE:$DEST/"; then
  rm -f "$PROC_DIR/.UPLOAD_VERIFY_OK"
  echo "UPLOAD_VERIFY_SENTINEL_SYNC_FAILED $CAM to $STORAGE:$DEST"
  exit 61
fi
echo "UPLOAD_VERIFY_OK $CAM files=$(wc -l < "$LOCAL_LIST") including_root_sidecars=1"
""".strip()

    def _script_delete_uploaded_local(self, job: PipelineJob) -> str:
        return self._processed_dir_snippet(job, require_processed=True) + "\n\n" + """
if [[ ! -f "$PROC_DIR/.UPLOAD_VERIFY_OK" ]]; then
  echo "REFUSING_DELETE_PROCESSED_WITHOUT_UPLOAD_VERIFY_OK $PROC_DIR/.UPLOAD_VERIFY_OK"
  exit 70
fi
BYTES=$(du -sb "$PROC_DIR" | awk '{print $1}')
MARKER="$DIR/${CAM}.PROCESSED_UPLOAD_VERIFIED"
{
  echo "SESSION=$SESSION"
  echo "CAM=$CAM"
  echo "PROC_DIR=$PROC_DIR"
  echo "BYTES=$BYTES"
  echo "VERIFIED_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  find "$PROC_DIR" -maxdepth 1 -type f -name '*.VERIFY_OK' -printf 'VERIFY_OK=%f\n' | sort
} > "$MARKER"
rm -rf "$PROC_DIR"
echo "LOCAL_PROCESSED_DELETED $CAM bytes=$BYTES marker=$MARKER"
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
        if step == "delete_session_force":
            return self._script_delete_session_force(job)
        raise ValueError(f"unknown processing step: {step}")

    def _append_manifest(self, session: str, cam: str, step: str, ok: bool, detail: str) -> None:
        path = self.base_dir / session / self.manifest_name
        clean_detail = " ".join(str(detail).split())[:800]
        line = f"{datetime.utcnow().isoformat(timespec='seconds')}Z\t{session}\t{cam}\t{step}\t{int(ok)}\t{clean_detail}\n"
        with self._manifest_lock:
            path.parent.mkdir(parents=True, exist_ok=True)
            if not path.exists():
                path.write_text("timestamp_utc\tsession\tcamera\tstep\tok\tdetail\n", encoding="utf-8")
            with path.open("a", encoding="utf-8") as f:
                f.write(line)

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

    def _run_multicam_sync_audit(self, session: str) -> bool:
        local_dir = self.base_dir / session
        if not MULTICAM_SYNC_AUDIT_AVAILABLE or run_multicam_sync_audit is None:
            msg = "MULTICAM_SYNC_AUDIT_UNAVAILABLE import failed"
            self.log.emit(f"Processing: {session}/multicam: FAIL: {msg}")
            self._append_manifest(session, "multicam", "multicam_sync", False, msg)
            return False
        if not local_dir.is_dir():
            msg = f"NO_LOCAL_SESSION_DIR {local_dir}"
            self.log.emit(f"Processing: {session}/multicam: FAIL: {msg}")
            self._append_manifest(session, "multicam", "multicam_sync", False, msg)
            return False

        camera_hosts = {spec.cam: spec.host for spec in self.camera_specs}
        self.status.emit(f"{session}: auditing multi-camera trigger alignment...")
        self.log.emit(
            f"Processing: {session}/multicam: collecting per-camera audit inputs and reconstructing shared trigger axis"
        )
        try:
            result = run_multicam_sync_audit(
                local_dir,
                expected_cameras=[spec.cam for spec in self.camera_specs],
                camera_hosts=camera_hosts,
                remote_sessions_root=self.remote_sessions_root,
                processed_subdir=self.processed_subdir,
                collect=True,
                log=lambda text: self.log.emit(f"Processing: {session}/multicam: {text}"),
            )
        except Exception as exc:
            msg = f"MULTICAM_SYNC_EXCEPTION {exc}"
            self.log.emit(f"Processing: {session}/multicam: FAIL: {msg}")
            self._append_manifest(session, "multicam", "multicam_sync", False, msg)
            return False

        detail = (
            f"{result.headline}; valid={result.valid_frames}/{result.total_common_frames} "
            f"({result.valid_percent:.3f}%); report={result.report_path}"
        )
        self._append_manifest(session, "multicam", "multicam_sync", result.ok, detail)
        if result.ok:
            self.log.emit(f"Processing: {session}/multicam: OK: {detail}")
            self.status.emit(f"{session}: {result.headline} ({result.valid_percent:.3f}% valid)")
            return True

        errors = "; ".join(result.errors[:4]) if result.errors else result.status
        self.log.emit(f"Processing: {session}/multicam: NOT PASSING: {result.headline}: {errors}")
        self.status.emit(f"{session}: {result.headline}: {errors}")
        return False

    def _upload_session_level_files(self, session: str) -> bool:
        local_dir = self.base_dir / session
        if not local_dir.is_dir():
            msg = f"NO_LOCAL_SESSION_DIR {local_dir}"
            self.log.emit(f"Processing: {session}/tmill: upload_session FAIL: {msg}")
            self._append_manifest(session, "tmill", "upload_session", False, msg)
            return False

        active_marker = local_dir / "rosbag" / "TELEMETRY_ACTIVE"
        if active_marker.exists():
            msg = f"REFUSING_UPLOAD_ACTIVE_TELEMETRY {active_marker}"
            self.log.emit(f"Processing: {session}/tmill: upload_session FAIL: {msg}")
            self._append_manifest(session, "tmill", "upload_session", False, msg)
            return False

        # Invalidate any older verification before attempting a fresh upload.
        # A failed or interrupted upload must leave local deletion blocked.
        try:
            (local_dir / ".SESSION_UPLOAD_VERIFY_OK").unlink()
        except FileNotFoundError:
            pass

        storage = self._storage_spec()
        remote_dir = f"{self.upload_root}/{session}"
        upload_items: List[Path] = []
        for name in ["session.yaml", self.manifest_name]:
            p = local_dir / name
            if p.exists():
                upload_items.append(p)
        for subdir in [self.thumbnails_subdir, "rosbag", "multicam_sync"]:
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
        if not ok:
            self.log.emit(f"Processing: {session}/tmill: upload_session FAIL rc={proc.returncode}: {_tail(proc.stdout, 8)}")
            self._append_manifest(session, "tmill", "upload_session", False, _tail(proc.stdout, 8))
            return False

        # Verify every session-level file by relative path and byte size. This is
        # intentionally stronger than merely checking remote session.yaml because
        # the rosbag is scientifically important and local deletion must never
        # outrun a partial rsync.
        ignored_names = {".SESSION_UPLOAD_VERIFY_OK", ".session_upload_sizes.tsv"}
        local_sizes: Dict[str, int] = {}
        for item in upload_items:
            if item.is_file():
                if item.name not in ignored_names:
                    local_sizes[str(item.relative_to(local_dir))] = item.stat().st_size
                continue
            for child in item.rglob("*"):
                if child.is_file() and child.name not in ignored_names:
                    local_sizes[str(child.relative_to(local_dir))] = child.stat().st_size

        roots = " ".join(_q(item.name) for item in upload_items)
        remote_list_cmd = (
            f"cd {_q(remote_dir)} && "
            f"find {roots} -type f ! -name '.SESSION_UPLOAD_VERIFY_OK' "
            f"! -name '.session_upload_sizes.tsv' -printf '%p\\t%s\\n' | sort"
        )
        remote_list = self._run_local(
            ["ssh"] + self._storage_ssh_argv() + [storage, remote_list_cmd],
            timeout_s=120,
        )
        if remote_list.returncode != 0:
            msg = f"SESSION_LEVEL_VERIFY_LIST_FAILED {_tail(remote_list.stdout, 8)}"
            self.log.emit(f"Processing: {session}/tmill: upload_session FAIL: {msg}")
            self._append_manifest(session, "tmill", "upload_session", False, msg)
            return False

        remote_sizes: Dict[str, int] = {}
        try:
            for line in (remote_list.stdout or "").splitlines():
                rel, size = line.rsplit("\t", 1)
                remote_sizes[rel.lstrip("./")] = int(size)
        except Exception as exc:
            msg = f"SESSION_LEVEL_VERIFY_PARSE_FAILED {exc}"
            self.log.emit(f"Processing: {session}/tmill: upload_session FAIL: {msg}")
            self._append_manifest(session, "tmill", "upload_session", False, msg)
            return False

        missing = sorted(set(local_sizes) - set(remote_sizes))[:10]
        mismatched = sorted(
            key for key in set(local_sizes) & set(remote_sizes)
            if local_sizes[key] != remote_sizes[key]
        )[:10]
        if missing or mismatched:
            msg = (
                f"SESSION_LEVEL_VERIFY_MISMATCH local={len(local_sizes)} remote={len(remote_sizes)} "
                f"missing={missing} size_mismatch={mismatched}"
            )
            self.log.emit(f"Processing: {session}/tmill: upload_session FAIL: {msg}")
            self._append_manifest(session, "tmill", "upload_session", False, msg)
            return False

        msg = f"SESSION_UPLOAD_VERIFY_OK files={len(local_sizes)} to {storage}:{remote_dir}"
        self._append_manifest(session, "tmill", "upload_session", True, msg)

        sentinel = local_dir / ".SESSION_UPLOAD_VERIFY_OK"
        sentinel.write_text(
            f"verified_utc={datetime.utcnow().isoformat(timespec='seconds')}Z\nfiles={len(local_sizes)}\n",
            encoding="utf-8",
        )
        # _append_manifest above adds the final success line after the main rsync.
        # Push that updated manifest plus the verification sentinel as the last
        # atomic-ish bookkeeping step.
        final_items = [sentinel]
        manifest = local_dir / self.manifest_name
        if manifest.is_file():
            final_items.insert(0, manifest)
        final_sync = self._run_local(
            ["rsync", "-a", "--partial"]
            + self._storage_rsync_argv()
            + [str(path) for path in final_items]
            + [f"{storage}:{remote_dir}/"],
            timeout_s=300,
        )
        if final_sync.returncode != 0:
            # A partial final rsync could have copied the sentinel before failing.
            # Remove both copies best-effort so local deletion stays hard-blocked.
            try:
                sentinel.unlink()
            except FileNotFoundError:
                pass
            remove_remote = f"rm -f {_q(remote_dir + '/.SESSION_UPLOAD_VERIFY_OK')}"
            self._run_local(
                ["ssh"] + self._storage_ssh_argv() + [storage, remove_remote],
                timeout_s=60,
            )
            msg = f"SESSION_LEVEL_REMOTE_SENTINEL_FAILED {_tail(final_sync.stdout, 4)}"
            self.log.emit(f"Processing: {session}/tmill: upload_session FAIL: {msg}")
            self._append_manifest(session, "tmill", "upload_session", False, msg)
            return False

        self.log.emit(f"Processing: {session}/tmill: upload_session OK: {msg}")
        return True


    def _delete_local_session_force(self, session: str) -> bool:
        """Permanently delete one selected tmill session with strict path guards."""
        if not session or session in {".", ".."} or "/" in session or "\\" in session:
            self.log.emit(f"Processing: {session}/tmill: delete_session_force_local FAIL: unsafe session name")
            return False

        try:
            root = self.base_dir.expanduser().resolve()
            target = (root / session).resolve()
        except Exception as exc:
            self.log.emit(
                f"Processing: {session}/tmill: delete_session_force_local FAIL: path resolution failed: {exc}"
            )
            return False

        if root == Path("/") or target == root or target.parent != root:
            self.log.emit(
                f"Processing: {session}/tmill: delete_session_force_local FAIL: "
                f"REFUSING_DANGEROUS_SESSION_PATH root={root} target={target}"
            )
            return False

        if not target.exists():
            self.log.emit(f"Processing: {session}/tmill: delete_session_force_local OK: already absent {target}")
            return True

        try:
            byte_count = sum(p.stat().st_size for p in target.rglob("*") if p.is_file())
            file_count = sum(1 for p in target.rglob("*") if p.is_file())
        except Exception:
            byte_count = -1
            file_count = -1

        try:
            shutil.rmtree(target)
        except Exception as exc:
            self.log.emit(
                f"Processing: {session}/tmill: delete_session_force_local FAIL: could not delete {target}: {exc}"
            )
            return False

        if target.exists():
            self.log.emit(
                f"Processing: {session}/tmill: delete_session_force_local FAIL: target still exists {target}"
            )
            return False

        self.log.emit(
            f"Processing: {session}/tmill: delete_session_force_local OK: "
            f"deleted {target} bytes={byte_count} files={file_count}"
        )
        return True

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

        active_marker = local_dir / "rosbag" / "TELEMETRY_ACTIVE"
        if active_marker.exists():
            msg = f"REFUSING_DELETE_ACTIVE_TELEMETRY {active_marker}"
            self.log.emit(f"Processing: {session}/tmill: delete_session_local FAIL: {msg}")
            self._append_manifest(session, "tmill", "delete_session_local", False, msg)
            return False

        local_session_verify = local_dir / ".SESSION_UPLOAD_VERIFY_OK"
        if not local_session_verify.is_file():
            msg = f"REFUSING_DELETE_SESSION_WITHOUT_VERIFY {local_session_verify}"
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
        check_cmd = (
            f"test -s {_q(remote_dir + '/session.yaml')} && "
            f"test -f {_q(remote_dir + '/.SESSION_UPLOAD_VERIFY_OK')}"
        )
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
        self._persistent_pipeline_log_path: Optional[Path] = None
        self._last_eod_log_path: Optional[Path] = None
        self._persistent_pipeline_log_write_failed = False
        self._last_progress_done = 0
        self._last_progress_total = 0
        self.settings = QtCore.QSettings("SpenceLab", "camera_control")
        self.processing_config_path = _remembered_processing_config_path()
        cfg = _load_processing_config(self.processing_config_path).get("processing", {})
        conv = cfg.get("conversion", {}) if isinstance(cfg.get("conversion"), dict) else {}
        upload = cfg.get("upload", {}) if isinstance(cfg.get("upload"), dict) else {}

        self.processed_subdir = str(cfg.get("processed_subdir", "processed"))
        self.thumbnails_subdir = str(cfg.get("thumbnails_subdir", "thumbnails"))
        self.manifest_name = str(cfg.get("manifest_name", "processing_manifest.tsv"))
        self.audit_threshold_frames = float(conv.get("audit_threshold_frames", 1.5))
        self.max_parallel_cameras = max(1, int(cfg.get("max_parallel_cameras", 5)))
        self.max_parallel_uploads = max(1, int(upload.get("max_parallel_uploads", 5)))

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

        self.processing_profile_combo = QtWidgets.QComboBox()
        self.processing_profile_combo.setMinimumWidth(280)
        self.reload_profiles_btn = QtWidgets.QPushButton("Refresh profiles")
        self.load_config_btn = QtWidgets.QPushButton("Load selected processing YAML")
        self._populate_processing_profile_combo()

        self.refresh_btn = QtWidgets.QPushButton("Refresh sessions")
        self.create_btn = QtWidgets.QPushButton("Create and copy thumbnails")
        self.process_btn = QtWidgets.QPushButton("Process raws")
        self.info_btn = QtWidgets.QPushButton("Run raw info")
        self.audit_btn = QtWidgets.QPushButton("Run raw audit")
        self.info_audit_btn = QtWidgets.QPushButton("Info + audit")
        self.multicam_audit_btn = QtWidgets.QPushButton("Audit multi-cam sync")
        self.verify_btn = QtWidgets.QPushButton("Verify processed")
        self.delete_raws_btn = QtWidgets.QPushButton("Delete verified raws")
        self.upload_btn = QtWidgets.QPushButton("Upload processed")
        self.verify_upload_btn = QtWidgets.QPushButton("Verify upload")
        self.delete_uploaded_local_btn = QtWidgets.QPushButton("Delete local uploaded files")
        self.delete_uploaded_session_local_btn = QtWidgets.QPushButton("Delete uploaded session copies + trim")
        self.delete_sessions_btn = QtWidgets.QPushButton("DELETE SELECTED SESSIONS")
        self.delete_sessions_btn.setToolTip(
            "Permanently delete selected session folders from tmill and all configured camera hosts. "
            "No processing or upload verification is required."
        )
        self.delete_sessions_btn.setStyleSheet(
            "QPushButton { font-weight: bold; color: white; background-color: #b00020; padding: 6px 10px; }"
            "QPushButton:disabled { color: #dddddd; background-color: #777777; }"
        )
        self.process_verify_btn = QtWidgets.QPushButton("Process + verify")
        self.upload_verify_btn = QtWidgets.QPushButton("Upload + verify")
        self.process_to_upload_btn = QtWidgets.QPushButton("Process + verify + upload")
        self.end_of_day_btn = QtWidgets.QPushButton("END OF DAY: process + upload + delete + trim")
        self.end_of_day_btn.setToolTip(
            "Process, verify, upload, verify upload, run multi-camera sync audit, then delete only sessions "
            "whose entire verification chain passed. Finally run one batch TRIM per camera host."
        )
        self.end_of_day_btn.setStyleSheet(
            "QPushButton { font-weight: bold; padding: 6px 10px; }"
        )
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
        self.status_label.setMinimumWidth(0)
        self.status_label.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Ignored,
            QtWidgets.QSizePolicy.Policy.Preferred,
        )

        self._build_layout()

        self.processing_profile_combo.currentIndexChanged.connect(self._processing_profile_changed)
        self.reload_profiles_btn.clicked.connect(self.refresh_processing_profiles)
        self.load_config_btn.clicked.connect(self.load_selected_processing_yaml)
        self.refresh_btn.clicked.connect(self.refresh_sessions)
        self.create_btn.clicked.connect(self.create_thumbnails)
        self.process_btn.clicked.connect(lambda: self.run_pipeline("process"))
        self.info_btn.clicked.connect(lambda: self.run_pipeline("info"))
        self.audit_btn.clicked.connect(lambda: self.run_pipeline("audit"))
        self.info_audit_btn.clicked.connect(lambda: self.run_pipeline("info_audit"))
        self.multicam_audit_btn.clicked.connect(lambda: self.run_pipeline("multicam_sync"))
        self.verify_btn.clicked.connect(lambda: self.run_pipeline("verify"))
        self.delete_raws_btn.clicked.connect(lambda: self.run_pipeline("delete_raws"))
        self.upload_btn.clicked.connect(lambda: self.run_pipeline("upload"))
        self.verify_upload_btn.clicked.connect(lambda: self.run_pipeline("verify_upload"))
        self.delete_uploaded_local_btn.clicked.connect(lambda: self.run_pipeline("delete_uploaded_local"))
        self.delete_uploaded_session_local_btn.clicked.connect(lambda: self.run_pipeline("delete_uploaded_session_local"))
        self.delete_sessions_btn.clicked.connect(lambda: self.run_pipeline("delete_sessions"))
        self.process_verify_btn.clicked.connect(lambda: self.run_pipeline("process_verify"))
        self.upload_verify_btn.clicked.connect(lambda: self.run_pipeline("upload_verify"))
        self.process_to_upload_btn.clicked.connect(lambda: self.run_pipeline("process_verify_upload"))
        self.end_of_day_btn.clicked.connect(lambda: self.run_pipeline("process_verify_upload_delete_trim"))
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
        wb.addWidget(QtWidgets.QLabel("MP4 playback FPS"))
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

        profile = QtWidgets.QHBoxLayout()
        profile.addWidget(self.processing_profile_combo, stretch=1)
        profile.addWidget(self.load_config_btn)
        profile.addWidget(self.reload_profiles_btn)
        top.addRow("Processing YAML", profile)

        buttons1 = QtWidgets.QHBoxLayout()
        buttons1.addWidget(self.refresh_btn)
        buttons1.addWidget(self.create_btn)
        buttons1.addWidget(self.process_btn)
        buttons1.addWidget(self.info_btn)
        buttons1.addWidget(self.audit_btn)
        buttons1.addWidget(self.info_audit_btn)
        buttons1.addWidget(self.multicam_audit_btn)
        buttons1.addWidget(self.verify_btn)
        buttons1.addWidget(self.delete_raws_btn)
        buttons1.addStretch(1)

        buttons2 = QtWidgets.QHBoxLayout()
        buttons2.addWidget(self.upload_btn)
        buttons2.addWidget(self.verify_upload_btn)
        buttons2.addWidget(self.delete_uploaded_local_btn)
        buttons2.addWidget(self.delete_uploaded_session_local_btn)
        buttons2.addStretch(1)

        danger_buttons = QtWidgets.QHBoxLayout()
        danger_buttons.addWidget(self.delete_sessions_btn)
        danger_buttons.addWidget(
            QtWidgets.QLabel("Permanently removes selected sessions from tmill + configured camera hosts; storage is untouched.")
        )
        danger_buttons.addStretch(1)

        buttons3 = QtWidgets.QHBoxLayout()
        buttons3.addWidget(self.process_verify_btn)
        buttons3.addWidget(self.upload_verify_btn)
        buttons3.addWidget(self.process_to_upload_btn)
        buttons3.addWidget(self.end_of_day_btn)
        buttons3.addWidget(self.cancel_btn)
        buttons3.addStretch(1)

        hint = QtWidgets.QLabel(
            "Camera map examples: cam1@local for one-box testing, or cam1@cam1 cam2@cam2 ... for MERB. "
            "Heavy processing runs on each camera host. Cameras upload their processed files directly to storage. "
            "tmill uploads session.yaml, thumbnails, processing_manifest.tsv, rosbag/, and multicam_sync/ if present. "
            "The conversion FPS is MP4 playback speed; raw audit acquisition FPS is recovered from each recording's metadata. "
            "Run raw info/audit writes non-destructive diagnostics into each camera processed/ folder. "
            "Process and Process + verify paths also safely attempt the session-level multi-camera sync audit. "
            "Normal cleanup buttons require prior VERIFY_OK / UPLOAD_VERIFY_OK sentinel files. "
            "END OF DAY runs the full process/verify/upload/multi-cam chain and only then attempts deletion for sessions whose entire chain passed; "
            "the existing deletion sentinels are still re-checked, and camera-drive TRIM is a final best-effort maintenance step. "
            "Delete uploaded session copies + trim removes selected local session folders after upload verification, refuses if raw binaries remain, "
            "then runs one best-effort batch TRIM per camera host. "
            "DELETE SELECTED SESSIONS is intentionally destructive for disposable tests: after a prominent confirmation it deletes the "
            "selected session from tmill and every configured camera host without requiring processing/upload sentinels; storage is untouched."
        )
        hint.setWordWrap(True)

        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(12, 12, 12, 12)
        layout.addLayout(top)
        layout.addLayout(buttons1)
        layout.addLayout(buttons2)
        layout.addLayout(danger_buttons)
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

    def _profile_label(self, path: Path) -> str:
        try:
            return str(path.resolve().relative_to(_repo_root()))
        except Exception:
            return str(path)

    def _populate_processing_profile_combo(self) -> None:
        current = Path(getattr(self, "processing_config_path", _default_processing_config_path())).expanduser()
        candidates = _processing_config_candidates()
        if current not in candidates:
            candidates.insert(0, current)

        self.processing_profile_combo.blockSignals(True)
        try:
            self.processing_profile_combo.clear()
            selected_index = 0
            for i, path in enumerate(candidates):
                label = self._profile_label(path)
                if not path.exists():
                    label += "  [missing]"
                self.processing_profile_combo.addItem(label, str(path))
                if str(path) == str(current):
                    selected_index = i
            self.processing_profile_combo.setCurrentIndex(selected_index)
        finally:
            self.processing_profile_combo.blockSignals(False)

    @QtCore.Slot()
    def refresh_processing_profiles(self) -> None:
        data = self.processing_profile_combo.currentData()
        if data:
            self.processing_config_path = Path(str(data)).expanduser()
        self._populate_processing_profile_combo()
        self.status_label.setText("Refreshed processing YAML profiles.")

    @QtCore.Slot(int)
    def _processing_profile_changed(self, index: int) -> None:
        if index < 0:
            return
        self.load_selected_processing_yaml()

    def _apply_processing_config_to_widgets(self, cfg: Dict[str, Any]) -> None:
        """Overwrite visible Processing tab settings from a processing.yaml dict."""
        conv = cfg.get("conversion", {}) if isinstance(cfg.get("conversion"), dict) else {}
        upload = cfg.get("upload", {}) if isinstance(cfg.get("upload"), dict) else {}

        self.processed_subdir = str(cfg.get("processed_subdir", "processed"))
        self.thumbnails_subdir = str(cfg.get("thumbnails_subdir", "thumbnails"))
        self.manifest_name = str(cfg.get("manifest_name", "processing_manifest.tsv"))
        self.audit_threshold_frames = float(conv.get("audit_threshold_frames", 1.5))
        self.max_parallel_cameras = max(1, int(cfg.get("max_parallel_cameras", 5)))
        self.max_parallel_uploads = max(1, int(upload.get("max_parallel_uploads", 5)))

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
    def load_selected_processing_yaml(self) -> None:
        data = self.processing_profile_combo.currentData()
        path = Path(str(data)).expanduser() if data else self.processing_config_path

        cfg = _load_processing_config(path).get("processing", {})
        if not isinstance(cfg, dict):
            self.status_label.setText(f"processing config did not contain a processing: map: {path}")
            return

        self.processing_config_path = path
        self.settings.setValue("processing/config_path", str(path))
        self._apply_processing_config_to_widgets(cfg)
        self.refresh_sessions()

        msg = f"Processing: loaded profile {self._profile_label(path)}"
        self.status_label.setText(msg)
        self.log_line.emit(msg)

    # Compatibility with the first quick reload-button patch.
    @QtCore.Slot()
    def load_processing_yaml(self) -> None:
        self.load_selected_processing_yaml()


    def _set_busy(self, busy: bool) -> None:
        for widget in [
            self.processing_profile_combo, self.reload_profiles_btn, self.load_config_btn,
            self.refresh_btn, self.create_btn, self.process_btn, self.info_btn,
            self.audit_btn, self.info_audit_btn, self.multicam_audit_btn, self.verify_btn,
            self.delete_raws_btn, self.upload_btn, self.verify_upload_btn,
            self.delete_uploaded_local_btn, self.delete_uploaded_session_local_btn, self.delete_sessions_btn,
            self.process_verify_btn, self.upload_verify_btn, self.process_to_upload_btn, self.end_of_day_btn,
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

    def _end_of_day_log_dir(self) -> Path:
        """Persistent tmill receipts live outside camera_sessions so cleanup cannot remove them."""
        return Path.home() / "camera_control_logs" / "end_of_day"

    def _begin_end_of_day_log(self, action: str, sessions: List[str], cameras: List[str]) -> Path:
        """Create a durable tmill receipt before either verified cleanup path starts.

        Failure here is intentionally fatal: if camera-control is going to delete
        verified local copies, tmill must first be able to create its tiny
        persistent audit trail.
        """
        log_dir = self._end_of_day_log_dir()
        log_dir.mkdir(parents=True, exist_ok=True)

        started = datetime.now(timezone.utc)
        stamp = started.strftime("%Y%m%dT%H%M%SZ")
        if action == "delete_uploaded_session_local":
            slug = "delete_uploaded_session_copies_trim"
            title = "# SpenceLab camera-control manual verified cleanup + trim receipt"
        else:
            slug = "process_verify_upload_delete_trim"
            title = "# SpenceLab camera-control end-of-day archive/cleanup receipt"
        path = log_dir / f"{stamp}_{slug}.log"
        if path.exists():
            path = log_dir / f"{stamp}_{os.getpid()}_{slug}.log"

        upload_user = self.upload_user_edit.text().strip() or "spencelab"
        upload_host = self.upload_host_edit.text().strip() or "gpu2"
        upload_port = self.upload_port_edit.text().strip()
        upload_root = self.upload_root_edit.text().strip()
        upload_target = f"{upload_user}@{upload_host}:{upload_root}"
        if upload_port:
            upload_target += f" (ssh_port={upload_port})"

        lines = [
            title,
            f"START_UTC={started.isoformat(timespec='seconds')}",
            f"ACTION={action}",
            f"HOST={socket.gethostname()}",
            f"PID={os.getpid()}",
            f"PROCESSING_PROFILE={self.processing_config_path}",
            f"LOCAL_SESSIONS_ROOT={self._base_dir()}",
            f"REMOTE_CAMERA_SESSIONS_ROOT={self.remote_root_edit.text().strip() or '/home/spencelab/camera_sessions'}",
            f"UPLOAD_TARGET={upload_target}",
            f"SESSION_COUNT={len(sessions)}",
            f"CAMERA_COUNT={len(cameras)}",
            f"CAMERAS={' '.join(cameras)}",
        ]
        lines.extend(f"SESSION_{index:03d}={session}" for index, session in enumerate(sessions, start=1))
        lines.extend(["", "----- BEGIN PIPELINE LOG -----"])
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

        self._persistent_pipeline_log_path = path
        self._last_eod_log_path = path
        self._persistent_pipeline_log_write_failed = False
        return path

    def _append_end_of_day_log(self, text: str, *, tag: str = "LOG") -> None:
        path = self._persistent_pipeline_log_path
        if path is None or self._persistent_pipeline_log_write_failed:
            return
        try:
            now = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            message_lines = str(text).splitlines() or [""]
            with path.open("a", encoding="utf-8") as handle:
                for line in message_lines:
                    handle.write(f"{now}\t{tag}\t{line}\n")
        except Exception as exc:
            self._persistent_pipeline_log_write_failed = True
            # Do not disturb a pipeline that is already safely running merely
            # because later receipt appends failed. The initial receipt creation
            # is the hard preflight gate; subsequent failure is surfaced loudly.
            self.log_line.emit(f"Processing: END_OF_DAY_LOG_WRITE_FAILED path={path}: {exc}")

    @QtCore.Slot(str)
    def _handle_worker_log(self, text: str) -> None:
        self.log_line.emit(text)
        self._append_end_of_day_log(text)

    @QtCore.Slot(str)
    def _handle_worker_status(self, text: str) -> None:
        self._set_status_text(text)
        self._append_end_of_day_log(text, tag="STATUS")

    def _finish_end_of_day_log(self, ok: int, done: int) -> Optional[Path]:
        path = self._persistent_pipeline_log_path
        if path is None:
            return self._last_eod_log_path

        total = max(done, self._last_progress_total)
        success = ok == done == total and not self._persistent_pipeline_log_write_failed
        ended = datetime.now(timezone.utc)
        footer = [
            "----- END PIPELINE LOG -----",
            f"END_UTC={ended.isoformat(timespec='seconds')}",
            f"RESULT={'SUCCESS' if success else 'INCOMPLETE'}",
            f"STEPS_OK={ok}",
            f"STEPS_COMPLETED={done}",
            f"STEPS_TOTAL={total}",
            f"RECEIPT_LOG_WRITE_OK={0 if self._persistent_pipeline_log_write_failed else 1}",
            f"LOG_PATH={path}",
        ]
        try:
            with path.open("a", encoding="utf-8") as handle:
                handle.write("\n" + "\n".join(footer) + "\n")
        except Exception as exc:
            self.log_line.emit(f"Processing: END_OF_DAY_LOG_FINALIZE_FAILED path={path}: {exc}")
        finally:
            self._persistent_pipeline_log_path = None
        return path

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

        if action == "process_verify_upload_delete_trim":
            session_text = ", ".join(sessions[:5])
            if len(sessions) > 5:
                session_text += f", ... ({len(sessions)} total)"
            reply = QtWidgets.QMessageBox.warning(
                self,
                "End-of-day archive and cleanup?",
                "This runs PROCESS + VERIFY + UPLOAD + VERIFY UPLOAD + MULTI-CAMERA AUDIT.\n\n"
                "A selected session is deleted from tmill and the configured camera hosts ONLY if every step "
                "in that session's current run succeeds, and the existing upload/deletion sentinels pass again.\n\n"
                "After successful camera cleanup, one batch TRIM is attempted on each camera host. "
                "TRIM failure does not affect the archive or deletion result.\n\n"
                f"Sessions: {session_text}",
                QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.Cancel,
                QtWidgets.QMessageBox.StandardButton.Cancel,
            )
            if reply != QtWidgets.QMessageBox.StandardButton.Yes:
                self.status_label.setText("End-of-day archive/cleanup cancelled. Nothing was started.")
                return

            try:
                receipt_path = self._begin_end_of_day_log(action, sessions, cameras)
            except Exception as exc:
                QtWidgets.QMessageBox.critical(
                    self,
                    "Cannot create end-of-day receipt",
                    "Nothing was started because camera-control could not create the persistent tmill "
                    "end-of-day log.\n\n"
                    f"{exc}",
                )
                self.status_label.setText("End-of-day archive/cleanup not started: receipt log could not be created.")
                return
            self.log_line.emit(f"Processing: end-of-day receipt log: {receipt_path}")
            self._append_end_of_day_log(f"RECEIPT_READY path={receipt_path}", tag="CONTROL")

        if action == "delete_sessions":
            session_lines = "<br>".join(f"• {s}" for s in sessions)
            box = QtWidgets.QMessageBox(self)
            box.setIcon(QtWidgets.QMessageBox.Icon.Critical)
            box.setWindowTitle("DANGER: Permanently delete selected sessions")
            box.setTextFormat(QtCore.Qt.TextFormat.RichText)
            box.setText(
                f"<h2>PERMANENTLY DELETE {len(sessions)} SELECTED SESSION(S)?</h2>"
                "<p><b>This cannot be undone.</b></p>"
                "<p>The selected session folder(s) will be removed from <b>tmill</b> and "
                "<b>every configured camera host</b>. Raw files, MP4s, metadata, rosbag data, "
                "and any other local session contents will be deleted.</p>"
                "<p><b>The storage/upload server is NOT touched.</b></p>"
                f"<p>{session_lines}</p>"
            )
            cancel_button = box.addButton(QtWidgets.QMessageBox.StandardButton.Cancel)
            delete_label = f"DELETE {len(sessions)} SESSION" + ("S" if len(sessions) != 1 else "")
            delete_button = box.addButton(delete_label, QtWidgets.QMessageBox.ButtonRole.DestructiveRole)
            box.setDefaultButton(cancel_button)
            box.setEscapeButton(cancel_button)
            box.exec()
            if box.clickedButton() is not delete_button:
                self.status_label.setText("Session deletion cancelled. Nothing was deleted.")
                return

        if action == "delete_uploaded_session_local":
            session_text = ", ".join(sessions[:5])
            if len(sessions) > 5:
                session_text += f", ... ({len(sessions)} total)"
            reply = QtWidgets.QMessageBox.warning(
                self,
                "Delete uploaded session copies + trim?",
                "This permanently deletes the selected local session folder(s) and camera-host session folder(s) after upload checks.\n\n"
                "It will delete raw files only when local or uploaded VERIFY_OK evidence exists. After successful camera cleanup, "
                "one batch TRIM is attempted on each camera host. TRIM failure does not affect the archive or deletion result.\n\n"
                f"Sessions: {session_text}",
                QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.Cancel,
                QtWidgets.QMessageBox.StandardButton.Cancel,
            )
            if reply != QtWidgets.QMessageBox.StandardButton.Yes:
                self.status_label.setText("Session cleanup cancelled.")
                return

            try:
                receipt_path = self._begin_end_of_day_log(action, sessions, cameras)
            except Exception as exc:
                QtWidgets.QMessageBox.critical(
                    self,
                    "Cannot create cleanup receipt",
                    "Nothing was started because camera-control could not create the persistent tmill "
                    "manual-cleanup log.\n\n"
                    f"{exc}",
                )
                self.status_label.setText("Manual cleanup not started: receipt log could not be created.")
                return
            self.log_line.emit(f"Processing: manual cleanup receipt log: {receipt_path}")
            self._append_end_of_day_log(f"RECEIPT_READY path={receipt_path}", tag="CONTROL")

        self._last_progress_done = 0
        self._last_progress_total = max(1, len(sessions) * len(cameras))
        self.progress.setRange(0, self._last_progress_total)
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
            max_parallel_cameras=self.max_parallel_cameras,
            max_parallel_uploads=self.max_parallel_uploads,
        )
        self._start_worker(self._worker, self._worker.run, self._worker.finished, self._on_pipeline_finished)

    def _short_status_text(self, text: str, max_chars: int = 180) -> str:
        text = " ".join(str(text or "").split())
        if len(text) <= max_chars:
            return text
        head_chars = 70
        tail_chars = max(40, max_chars - head_chars - 5)
        return f"{text[:head_chars]} ... {text[-tail_chars:]}"

    @QtCore.Slot(str)
    def _set_status_text(self, text: str) -> None:
        full = " ".join(str(text or "").split())
        self.status_label.setToolTip(full)
        self.status_label.setText(self._short_status_text(full))

    def _start_worker(self, worker: QtCore.QObject, start_slot, finished_signal, finished_slot) -> None:
        worker.moveToThread(self._thread)
        self._thread.started.connect(start_slot)
        worker.log.connect(self._handle_worker_log)  # type: ignore[attr-defined]
        if hasattr(worker, "status"):
            worker.status.connect(self._handle_worker_status)  # type: ignore[attr-defined]
        worker.progress.connect(self._on_progress)  # type: ignore[attr-defined]
        finished_signal.connect(finished_slot)
        # The worker lives in the QThread. Do not drop the last Python
        # reference from a GUI-thread completion slot while its thread is
        # still winding down; that can destroy a QObject from the wrong
        # thread and, with PySide/Qt, can surface as a native segfault.
        # Use Qt's canonical worker-thread teardown instead.
        finished_signal.connect(worker.deleteLater)
        finished_signal.connect(self._thread.quit)
        self._thread.finished.connect(self._on_worker_thread_finished)
        self._thread.finished.connect(self._thread.deleteLater)
        self._thread.start()

    @QtCore.Slot()
    def _on_worker_thread_finished(self) -> None:
        # Only release GUI-side references after QThread has actually
        # stopped. This is also the point where a new job may safely start.
        self._worker = None
        self._thread = None
        self._set_busy(False)

    @QtCore.Slot()
    def cancel(self) -> None:
        if self._worker is not None and hasattr(self._worker, "cancel"):
            self._worker.cancel()  # type: ignore[attr-defined]
            self.cancel_btn.setEnabled(False)
            self.status_label.setText("Cancelling active local/SSH command...")

    @QtCore.Slot(int, int)
    def _on_progress(self, done: int, total: int) -> None:
        self._last_progress_done = int(done)
        self._last_progress_total = max(1, int(total))
        self.progress.setRange(0, self._last_progress_total)
        self.progress.setValue(done)
        self.status_label.setText(f"Processing jobs: {done}/{total}")

    @QtCore.Slot(int, int)
    def _on_thumbnail_finished(self, ok: int, done: int) -> None:
        self.status_label.setText(f"Done: copied {ok}/{done} thumbnails.")

    @QtCore.Slot(str, int, int)
    def _on_pipeline_finished(self, action: str, ok: int, done: int) -> None:
        receipt_path: Optional[Path] = None
        if action in {"process_verify_upload_delete_trim", "delete_uploaded_session_local"}:
            total = max(done, self._last_progress_total)
            archive_complete = ok == done == total
            receipt_write_ok = not self._persistent_pipeline_log_write_failed
            receipt_path = self._finish_end_of_day_log(ok, done)
            if action == "delete_uploaded_session_local":
                if archive_complete and receipt_write_ok:
                    status_text = (
                        "Manual verified cleanup complete: local session copies deleted, camera-drive TRIM completed, "
                        "and the persistent tmill receipt was saved."
                    )
                elif archive_complete:
                    status_text = (
                        "Manual verified cleanup completed, but persistent receipt logging encountered an error. "
                        "The archive/delete result is unaffected; see the GUI log."
                    )
                else:
                    status_text = (
                        f"Manual verified cleanup finished with safeguards active: {ok}/{done} completed step(s) OK "
                        f"out of {total} planned. Any failed deletion gate kept its affected local copy; "
                        "a TRIM-only failure is non-destructive. See the log."
                    )
            elif archive_complete and receipt_write_ok:
                status_text = (
                    "END OF DAY complete: archive verified, local session copies deleted, "
                    "camera-drive TRIM completed, and the persistent tmill receipt was saved."
                )
            elif archive_complete:
                status_text = (
                    "END OF DAY archive/cleanup completed, but persistent receipt logging encountered an error. "
                    "The archive/delete result is unaffected; see the GUI log."
                )
            else:
                status_text = (
                    f"END OF DAY finished with safeguards active: {ok}/{done} completed step(s) OK "
                    f"out of {total} planned. Any session whose verification chain failed was kept; "
                    "see the log. A TRIM failure is non-destructive."
                )
        else:
            status_text = f"Done: {action}: {ok}/{done} step(s) OK."
        if action in {
            "multicam_sync", "process", "info_audit", "process_verify", "process_verify_upload",
            "process_verify_upload_delete_trim",
        }:
            sync_statuses: List[str] = []
            sync_headlines: List[str] = []
            for session in self._selected_sessions():
                summary = self._base_dir() / session / "multicam_sync" / "multicam_sync_summary.yaml"
                if not summary.is_file():
                    continue
                try:
                    import yaml  # type: ignore
                    data = yaml.safe_load(summary.read_text(encoding="utf-8")) or {}
                    sync_statuses.append(str(data.get("status", "unknown")))
                    sync_headlines.append(str(data.get("headline", "")))
                except Exception:
                    continue
            if len(sync_headlines) == 1 and sync_headlines[0]:
                status_text += f" Multi-cam: {sync_headlines[0]}."
            elif sync_statuses:
                counts = {name: sync_statuses.count(name) for name in sorted(set(sync_statuses))}
                status_text += " Multi-cam: " + ", ".join(f"{key}={value}" for key, value in counts.items()) + "."
        if receipt_path is not None:
            status_text += f" Receipt: {receipt_path}."
            receipt_label = "MANUAL_CLEANUP_LOG_SAVED" if action == "delete_uploaded_session_local" else "END_OF_DAY_LOG_SAVED"
            self.log_line.emit(f"Processing: {receipt_label} {receipt_path}")
        self._set_status_text(status_text)
        if action in {"delete_uploaded_session_local", "delete_sessions", "process_verify_upload_delete_trim"}:
            QtCore.QTimer.singleShot(0, self.refresh_sessions)
