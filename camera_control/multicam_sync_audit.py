#!/usr/bin/env python3
"""Session-level multi-camera synchronization audit.

This tool consumes per-camera ``raw_rolling_audit`` CSVs plus their recorded
metadata and reconstructs one shared hardware-trigger frame axis.  It is usable
both as a library by the camera_control Processing tab and directly from the
CLI.

The important design rule is that MP4 playback FPS is *not* acquisition FPS.
Expected acquisition cadence and hardware-trigger state come from the metadata
saved with the recording whenever possible.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import shutil
import socket
import statistics
import subprocess
import sys
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


LogFn = Optional[Callable[[str], None]]


@dataclass
class CameraRun:
    cam: str
    audit_path: Path
    metadata_path: Optional[Path]
    audit_header: Dict[str, str]
    rows: List[Dict[str, Any]]
    hardware_trigger: Optional[bool]
    expected_fps: Optional[float]
    expected_fps_source: str
    playback_fps: Optional[float]
    start_pc_utc_ns: int
    end_pc_utc_ns: int
    start_frame_number: int
    end_frame_number: int
    recorded_frames: int
    inferred_offset: int = 0
    intercept_ns: int = 0


@dataclass
class SyncAuditResult:
    status: str
    ok: bool
    headline: str
    session: str
    output_dir: Path
    report_path: Optional[Path] = None
    summary_path: Optional[Path] = None
    alignment_path: Optional[Path] = None
    total_common_frames: int = 0
    valid_frames: int = 0
    excluded_frames: int = 0
    valid_percent: float = 0.0
    warnings: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)

    @property
    def exit_code(self) -> int:
        if self.status in ("pass", "pass_with_exclusions"):
            return 0
        if self.status == "fail":
            return 1
        return 2


def _log(log: LogFn, message: str) -> None:
    if log:
        log(message)


def _is_local_host(host: str) -> bool:
    text = str(host or "").strip().lower()
    names = {
        "",
        "local",
        "localhost",
        "127.0.0.1",
        "::1",
        socket.gethostname().lower(),
        socket.getfqdn().lower(),
    }
    return text in names


def parse_camera_token(token: str) -> Tuple[str, str]:
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
    host = host.strip() or cam
    if not cam:
        raise ValueError(f"bad camera token: {token!r}")
    return cam, host


def _load_yaml(path: Path) -> Dict[str, Any]:
    try:
        import yaml  # type: ignore

        obj = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        return obj if isinstance(obj, dict) else {}
    except Exception:
        return {}


def _write_yaml(path: Path, obj: Mapping[str, Any]) -> None:
    try:
        import yaml  # type: ignore

        path.write_text(yaml.safe_dump(dict(obj), sort_keys=False), encoding="utf-8")
    except Exception:
        # JSON is valid YAML 1.2 and keeps this tool usable even if PyYAML is
        # absent from a stripped-down analysis environment.
        path.write_text(json.dumps(obj, indent=2) + "\n", encoding="utf-8")


def _coerce_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("true", "yes", "on", "1"):
            return True
        if text in ("false", "no", "off", "0"):
            return False
    return None


def acquisition_context_from_metadata(path: Optional[Path]) -> Tuple[Optional[bool], Optional[float], str]:
    if path is None or not path.is_file():
        return None, None, "metadata_missing"
    doc = _load_yaml(path)
    root = doc.get("cambuffer_recorder_ng", doc)
    if not isinstance(root, dict):
        return None, None, "metadata_invalid"

    sections: List[Tuple[str, Dict[str, Any]]] = []
    for name in ("effective_settings", "requested_settings"):
        value = root.get(name)
        if isinstance(value, dict):
            sections.append((name, value))

    for section_name, settings in sections:
        hw = _coerce_bool(settings.get("camera.hardware_trigger"))
        if hw is True:
            keys = ("camera.expected_hardware_fps", "camera.fps", "fps")
        else:
            keys = ("camera.fps", "fps", "camera.expected_hardware_fps")
        for key in keys:
            value = settings.get(key)
            try:
                fps = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(fps) and fps > 0:
                return hw, fps, f"{section_name}.{key}"
        if hw is not None:
            return hw, None, f"{section_name}.camera.hardware_trigger"

    return None, None, "metadata_keys_missing"


def _parse_env(path: Path) -> Dict[str, str]:
    out: Dict[str, str] = {}
    try:
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            if "=" not in line or line.lstrip().startswith("#"):
                continue
            key, value = line.split("=", 1)
            out[key.strip()] = value.strip()
    except OSError:
        pass
    return out


def _playback_fps_for_base(directory: Path, base: str) -> Optional[float]:
    for suffix in (".process.env", ".audit.env"):
        path = directory / f"{base}{suffix}"
        if not path.is_file():
            continue
        env = _parse_env(path)
        for key in ("PLAYBACK_FPS", "FPS"):
            try:
                value = float(env.get(key, ""))
            except ValueError:
                continue
            if value > 0 and math.isfinite(value):
                return value
    return None


def _read_audit_csv(path: Path, cam: str, metadata_path: Optional[Path]) -> CameraRun:
    header: Dict[str, str] = {}
    rows: List[Dict[str, Any]] = []
    csv_lines: List[str] = []
    with path.open("r", encoding="utf-8", errors="replace", newline="") as fh:
        for line in fh:
            if line.startswith("#"):
                text = line[1:].strip()
                if ":" in text:
                    key, value = text.split(":", 1)
                    header[key.strip()] = value.strip()
            else:
                csv_lines.append(line)

    required = {
        "frame_index",
        "pc_utc_ns",
        "camera_timestamp_ns",
        "camera_frame_number",
    }
    reader = csv.DictReader(csv_lines)
    if not reader.fieldnames or not required.issubset(set(reader.fieldnames)):
        missing = sorted(required - set(reader.fieldnames or []))
        raise ValueError(f"{path}: missing required audit columns: {missing}")

    for raw in reader:
        try:
            row = {
                "frame_index": int(raw["frame_index"]),
                "pc_utc_ns": int(raw["pc_utc_ns"]),
                "pc_utc_iso": raw.get("pc_utc_iso", ""),
                "camera_timestamp_ns": int(raw["camera_timestamp_ns"]),
                "camera_frame_number": int(raw["camera_frame_number"]),
                "file_rollover_boundary": int(raw.get("file_rollover_boundary", "0") or 0),
                "flag": raw.get("flag", ""),
            }
        except Exception as exc:
            raise ValueError(f"{path}: bad audit data row: {exc}") from exc
        rows.append(row)

    if not rows:
        raise ValueError(f"{path}: audit contains no frame rows")

    hw, fps, source = acquisition_context_from_metadata(metadata_path)
    base = path.name[: -len(".audit.csv")]
    playback_fps = _playback_fps_for_base(path.parent, base)
    return CameraRun(
        cam=cam,
        audit_path=path,
        metadata_path=metadata_path,
        audit_header=header,
        rows=rows,
        hardware_trigger=hw,
        expected_fps=fps,
        expected_fps_source=source,
        playback_fps=playback_fps,
        start_pc_utc_ns=rows[0]["pc_utc_ns"],
        end_pc_utc_ns=rows[-1]["pc_utc_ns"],
        start_frame_number=rows[0]["camera_frame_number"],
        end_frame_number=rows[-1]["camera_frame_number"],
        recorded_frames=len(rows),
    )


def _metadata_for_audit(audit: Path) -> Optional[Path]:
    base = audit.name[: -len(".audit.csv")]
    exact = audit.parent / f"{base}.metadata.yaml"
    if exact.is_file():
        return exact
    candidates = sorted(audit.parent.glob("*.metadata.yaml"))
    if len(candidates) == 1:
        return candidates[0]
    # Prefer a camera/session metadata file over a dump-specific metadata file.
    nondump = [p for p in candidates if not re.search(r"dump\d+", p.name)]
    if len(nondump) == 1:
        return nondump[0]
    return None


def _camera_input_dirs(session_dir: Path, cam: str, processed_subdir: str) -> List[Path]:
    candidates = [
        session_dir / "multicam_sync" / "inputs" / cam,
        session_dir / cam / processed_subdir,
    ]
    out: List[Path] = []
    seen = set()
    for p in candidates:
        key = str(p.resolve()) if p.exists() else str(p)
        if p.is_dir() and key not in seen:
            out.append(p)
            seen.add(key)
    return out


def discover_camera_runs(
    session_dir: Path,
    cameras: Sequence[str],
    processed_subdir: str = "processed",
) -> Tuple[Dict[str, CameraRun], List[str]]:
    runs: Dict[str, CameraRun] = {}
    errors: List[str] = []
    for cam in cameras:
        audits: List[Path] = []
        for directory in _camera_input_dirs(session_dir, cam, processed_subdir):
            audits.extend(sorted(directory.glob("*.audit.csv")))
        # Deduplicate by filename+size. A collected input and consolidated local
        # copy may be the same scientific audit.
        unique: Dict[Tuple[str, int], Path] = {}
        for path in audits:
            try:
                key = (path.name, path.stat().st_size)
            except OSError:
                key = (str(path), -1)
            unique.setdefault(key, path)
        audits = list(unique.values())

        if not audits:
            errors.append(f"{cam}: no *.audit.csv found")
            continue
        if len(audits) > 1:
            names = ", ".join(p.name for p in audits[:5])
            errors.append(
                f"{cam}: found {len(audits)} audit runs ({names}); "
                "multi-run session matching is intentionally not guessed"
            )
            continue
        audit = audits[0]
        metadata = _metadata_for_audit(audit)
        try:
            runs[cam] = _read_audit_csv(audit, cam, metadata)
        except Exception as exc:
            errors.append(f"{cam}: {exc}")
    return runs, errors


def collect_camera_inputs(
    session_dir: Path,
    camera_hosts: Mapping[str, str],
    remote_sessions_root: str,
    processed_subdir: str = "processed",
    ssh_user: str = "spencelab",
    log: LogFn = None,
) -> List[str]:
    """Collect compact audit inputs from camera hosts into the master session.

    The full MP4s and raw files are deliberately not copied.  Keeping the audit
    CSV and recording metadata under the master session makes the sync report
    reproducible and allows that session-level report to travel with rosbag and
    session.yaml during upload.
    """
    errors: List[str] = []
    dest_root = session_dir / "multicam_sync" / "inputs"
    dest_root.mkdir(parents=True, exist_ok=True)
    session_name = session_dir.name

    include_args = [
        "--include=*.audit.csv",
        "--include=*.audit.stdout.txt",
        "--include=*.audit.env",
        "--include=*.metadata.yaml",
        "--include=*.process.env",
        "--include=*.raw_info.txt",
        "--exclude=*",
    ]

    for cam, host in camera_hosts.items():
        dest = dest_root / cam
        dest.mkdir(parents=True, exist_ok=True)
        remote_cam = f"{remote_sessions_root.rstrip('/')}/{session_name}/{cam}"
        remote_processed = f"{remote_cam}/{processed_subdir.strip('/') or 'processed'}"
        _log(log, f"multicam sync: collecting {cam} audit inputs from {host}")

        if _is_local_host(host):
            proc_dir = Path(remote_processed).expanduser()
            cam_dir = Path(remote_cam).expanduser()
            if not proc_dir.is_dir():
                errors.append(f"{cam}: no processed directory {proc_dir}")
                continue
            for pattern in (
                "*.audit.csv",
                "*.audit.stdout.txt",
                "*.audit.env",
                "*.metadata.yaml",
                "*.process.env",
                "*.raw_info.txt",
            ):
                for src in proc_dir.glob(pattern):
                    shutil.copy2(src, dest / src.name)
            # info/audit without processing may leave metadata in the camera dir.
            for src in cam_dir.glob("*.metadata.yaml"):
                target = dest / src.name
                if not target.exists():
                    shutil.copy2(src, target)
            continue

        remote = f"{ssh_user}@{host}"
        cmd = [
            "rsync",
            "-a",
            "--prune-empty-dirs",
            *include_args,
            f"{remote}:{remote_processed.rstrip('/')}/",
            f"{dest}/",
        ]
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
        if proc.returncode != 0:
            errors.append(f"{cam}: rsync processed inputs from {host} failed rc={proc.returncode}: {(proc.stdout or '').strip()[-400:]}")
            continue

        # Also fetch top-level recording metadata in case this was a standalone
        # Info + audit operation rather than a full process operation.
        meta_cmd = [
            "rsync",
            "-a",
            "--prune-empty-dirs",
            "--include=*.metadata.yaml",
            "--exclude=*",
            f"{remote}:{remote_cam.rstrip('/')}/",
            f"{dest}/",
        ]
        meta_proc = subprocess.run(meta_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
        if meta_proc.returncode != 0:
            errors.append(f"{cam}: metadata rsync from {host} failed rc={meta_proc.returncode}: {(meta_proc.stdout or '').strip()[-400:]}")
    return errors


def _median_int(values: Sequence[int]) -> int:
    if not values:
        raise ValueError("median of empty values")
    vals = sorted(values)
    n = len(vals)
    if n % 2:
        return vals[n // 2]
    return (vals[n // 2 - 1] + vals[n // 2]) // 2


def _percentile(values: Sequence[float], p: float) -> float:
    if not values:
        return float("nan")
    vals = sorted(values)
    if len(vals) == 1:
        return vals[0]
    x = (len(vals) - 1) * p
    lo = int(math.floor(x))
    hi = int(math.ceil(x))
    if lo == hi:
        return vals[lo]
    return vals[lo] + (vals[hi] - vals[lo]) * (x - lo)


def _ranges(values: Sequence[int]) -> List[Tuple[int, int]]:
    if not values:
        return []
    vals = sorted(set(values))
    out: List[Tuple[int, int]] = []
    start = prev = vals[0]
    for value in vals[1:]:
        if value == prev + 1:
            prev = value
            continue
        out.append((start, prev))
        start = prev = value
    out.append((start, prev))
    return out


def _camera_cadence_ms(run: CameraRun) -> List[float]:
    values: List[float] = []
    for a, b in zip(run.rows, run.rows[1:]):
        df = b["camera_frame_number"] - a["camera_frame_number"]
        dt = b["camera_timestamp_ns"] - a["camera_timestamp_ns"]
        if df > 0 and dt > 0:
            values.append((dt / df) / 1e6)
    return values


def _strictly_increasing(values: Iterable[int]) -> bool:
    iterator = iter(values)
    try:
        prev = next(iterator)
    except StopIteration:
        return True
    for value in iterator:
        if value <= prev:
            return False
        prev = value
    return True


def _block_median_drift_ms(
    ref_map: Mapping[int, Dict[str, Any]],
    cam_map: Mapping[int, Dict[str, Any]],
    valid_triggers: Sequence[int],
    common_start: int,
    common_end: int,
    blocks: int = 20,
) -> Tuple[Optional[float], List[float]]:
    meds: List[float] = []
    span = common_end - common_start + 1
    for block in range(blocks):
        lo = common_start + (span * block) // blocks
        hi = common_start + (span * (block + 1)) // blocks - 1
        diffs = [
            (cam_map[g]["pc_utc_ns"] - ref_map[g]["pc_utc_ns"]) / 1e6
            for g in valid_triggers
            if lo <= g <= hi and g in ref_map and g in cam_map
        ]
        if diffs:
            meds.append(float(statistics.median(diffs)))
    if len(meds) < 2:
        return None, meds
    return meds[-1] - meds[0], meds


def _safe_float_header(header: Mapping[str, str], key: str) -> Optional[float]:
    try:
        value = float(header.get(key, ""))
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def _status_marker(output_dir: Path, status: str) -> None:
    names = [
        "MULTICAM_SYNC_PASS",
        "MULTICAM_SYNC_PASS_WITH_EXCLUSIONS",
        "MULTICAM_SYNC_FAIL",
        "MULTICAM_SYNC_INCOMPLETE",
    ]
    for name in names:
        try:
            (output_dir / name).unlink()
        except FileNotFoundError:
            pass
    mapping = {
        "pass": "MULTICAM_SYNC_PASS",
        "pass_with_exclusions": "MULTICAM_SYNC_PASS_WITH_EXCLUSIONS",
        "fail": "MULTICAM_SYNC_FAIL",
        "incomplete": "MULTICAM_SYNC_INCOMPLETE",
    }
    marker = mapping.get(status, "MULTICAM_SYNC_INCOMPLETE")
    (output_dir / marker).write_text(
        f"status={status}\ngenerated_utc={datetime.now(timezone.utc).isoformat()}\n",
        encoding="utf-8",
    )


def audit_session(
    session_dir: Path | str,
    *,
    expected_cameras: Sequence[str],
    camera_hosts: Optional[Mapping[str, str]] = None,
    remote_sessions_root: Optional[str] = None,
    processed_subdir: str = "processed",
    ssh_user: str = "spencelab",
    collect: bool = False,
    max_offset_residual_frames: float = 0.25,
    cadence_tolerance_fraction: float = 0.02,
    log: LogFn = None,
) -> SyncAuditResult:
    session_dir = Path(session_dir).expanduser().resolve()
    session = session_dir.name
    output_dir = session_dir / "multicam_sync"
    output_dir.mkdir(parents=True, exist_ok=True)

    expected = list(dict.fromkeys(str(c).strip() for c in expected_cameras if str(c).strip()))
    result = SyncAuditResult(
        status="incomplete",
        ok=False,
        headline="MULTI-CAMERA SYNC AUDIT INCOMPLETE",
        session=session,
        output_dir=output_dir,
    )

    collection_errors: List[str] = []
    if collect and camera_hosts and remote_sessions_root:
        collection_errors = collect_camera_inputs(
            session_dir,
            camera_hosts,
            remote_sessions_root,
            processed_subdir=processed_subdir,
            ssh_user=ssh_user,
            log=log,
        )
        result.warnings.extend(collection_errors)

    if len(expected) < 2:
        result.errors.append("need at least two expected cameras for a multi-camera sync audit")
        _write_failure_outputs(result)
        return result

    runs, discovery_errors = discover_camera_runs(session_dir, expected, processed_subdir)
    if discovery_errors:
        result.errors.extend(discovery_errors)
    if len(runs) != len(expected):
        missing = [cam for cam in expected if cam not in runs]
        if missing:
            result.errors.append(f"missing usable audit input for: {', '.join(missing)}")
        _write_failure_outputs(result)
        return result

    # Recording metadata is the authority for acquisition cadence.  The audit
    # header itself may have been generated by older code that accidentally used
    # MP4 playback FPS, so a mismatch is a warning rather than the sync axis.
    fps_values: List[float] = []
    hardware_values: List[bool] = []
    for cam in expected:
        run = runs[cam]
        if run.hardware_trigger is not None:
            hardware_values.append(run.hardware_trigger)
        if run.expected_fps is not None:
            fps_values.append(run.expected_fps)
        audit_fps = _safe_float_header(run.audit_header, "expected_fps")
        if audit_fps and run.expected_fps and abs(audit_fps - run.expected_fps) > max(0.01, 0.001 * run.expected_fps):
            result.warnings.append(
                f"{cam}: per-camera audit header expected_fps={audit_fps:g}, "
                f"recording metadata acquisition_fps={run.expected_fps:g}; using recording metadata"
            )

    if any(run.hardware_trigger is not True for run in runs.values()):
        bad = [f"{cam}={runs[cam].hardware_trigger}" for cam in expected if runs[cam].hardware_trigger is not True]
        result.status = "fail"
        result.errors.append(
            "hardware-trigger lock cannot be asserted because not every camera records camera.hardware_trigger=true: "
            + ", ".join(bad)
        )
        _write_failure_outputs(result)
        return result

    if any(run.expected_fps is None for run in runs.values()):
        missing_fps = [cam for cam in expected if runs[cam].expected_fps is None]
        result.errors.append(
            "recorded acquisition FPS missing from metadata for: " + ", ".join(missing_fps)
        )
        _write_failure_outputs(result)
        return result

    fps = float(statistics.median([runs[cam].expected_fps for cam in expected if runs[cam].expected_fps is not None]))
    fps_spread = max(abs(float(runs[cam].expected_fps) - fps) for cam in expected)
    if fps_spread > max(0.01, 0.001 * fps):
        result.status = "fail"
        result.errors.append(
            "camera acquisition FPS metadata disagree: "
            + ", ".join(f"{cam}={runs[cam].expected_fps:g}" for cam in expected)
        )
        _write_failure_outputs(result)
        return result

    period_ns = int(round(1e9 / fps))

    fatal_integrity: List[str] = []
    for cam in expected:
        run = runs[cam]
        frame_numbers = [r["camera_frame_number"] for r in run.rows]
        frame_indices = [r["frame_index"] for r in run.rows]
        camera_ts = [r["camera_timestamp_ns"] for r in run.rows]
        pc_ts = [r["pc_utc_ns"] for r in run.rows]
        if not _strictly_increasing(frame_numbers):
            fatal_integrity.append(f"{cam}: camera_frame_number is duplicated or reversed")
        if not _strictly_increasing(frame_indices):
            fatal_integrity.append(f"{cam}: frame_index is duplicated or reversed")
        if not _strictly_increasing(camera_ts):
            fatal_integrity.append(f"{cam}: camera_timestamp_ns reset/reversed")
        # Host receipt timestamps can bunch up, but must not run backwards.
        if not _strictly_increasing(pc_ts):
            fatal_integrity.append(f"{cam}: pc_utc_ns reset/reversed")

        cadence = _camera_cadence_ms(run)
        if not cadence:
            fatal_integrity.append(f"{cam}: no valid camera timestamp cadence pairs")
        else:
            med_ms = float(statistics.median(cadence))
            expected_ms = 1000.0 / fps
            if abs(med_ms - expected_ms) / expected_ms > cadence_tolerance_fraction:
                fatal_integrity.append(
                    f"{cam}: median camera cadence {med_ms:.6f} ms disagrees with metadata {expected_ms:.6f} ms"
                )

    if fatal_integrity:
        result.status = "fail"
        result.errors.extend(fatal_integrity)
        _write_failure_outputs(result)
        return result

    # Infer integer trigger offsets from Chrony-synchronized host UTC.  Taking a
    # median intercept across the whole run rejects USB/driver receipt stalls;
    # then snapping differences to whole trigger periods gives the shared axis.
    for cam in expected:
        run = runs[cam]
        intercepts = [r["pc_utc_ns"] - r["camera_frame_number"] * period_ns for r in run.rows]
        run.intercept_ns = _median_int(intercepts)
    earliest_intercept = min(run.intercept_ns for run in runs.values())
    for cam in expected:
        runs[cam].inferred_offset = int(round((runs[cam].intercept_ns - earliest_intercept) / period_ns))

    normalized_origins = [
        runs[cam].intercept_ns - runs[cam].inferred_offset * period_ns for cam in expected
    ]
    offset_residual_ns = max(normalized_origins) - min(normalized_origins)
    offset_residual_frames = offset_residual_ns / period_ns
    if offset_residual_frames > max_offset_residual_frames:
        result.status = "fail"
        result.errors.append(
            f"integer trigger alignment is ambiguous: normalized host-time origin spread "
            f"{offset_residual_ns / 1e6:.3f} ms = {offset_residual_frames:.3f} frames"
        )
        _write_failure_outputs(result)
        return result

    frame_maps: Dict[str, Dict[int, Dict[str, Any]]] = {}
    for cam in expected:
        run = runs[cam]
        mapping: Dict[int, Dict[str, Any]] = {}
        for row in run.rows:
            trigger = row["camera_frame_number"] + run.inferred_offset
            if trigger in mapping:
                result.status = "fail"
                result.errors.append(f"{cam}: duplicate inferred global trigger {trigger}")
                _write_failure_outputs(result)
                return result
            mapping[trigger] = row
        frame_maps[cam] = mapping

    common_start = max(min(m) for m in frame_maps.values())
    common_end = min(max(m) for m in frame_maps.values())
    if common_end < common_start:
        result.status = "fail"
        result.errors.append("camera audit intervals do not overlap on the inferred trigger axis")
        _write_failure_outputs(result)
        return result

    total = common_end - common_start + 1
    missing_by_cam: Dict[str, List[int]] = {}
    for cam in expected:
        mapping = frame_maps[cam]
        missing_by_cam[cam] = [g for g in range(common_start, common_end + 1) if g not in mapping]
    excluded = sorted(set().union(*(set(v) for v in missing_by_cam.values())))
    excluded_set = set(excluded)
    valid_triggers = [g for g in range(common_start, common_end + 1) if g not in excluded_set]
    valid = len(valid_triggers)
    valid_pct = 100.0 * valid / total if total else 0.0

    host_spreads_ms: List[float] = []
    for g in valid_triggers:
        values = [frame_maps[cam][g]["pc_utc_ns"] for cam in expected]
        host_spreads_ms.append((max(values) - min(values)) / 1e6)

    host_stats = {
        "median_ms": _percentile(host_spreads_ms, 0.50),
        "p95_ms": _percentile(host_spreads_ms, 0.95),
        "p99_ms": _percentile(host_spreads_ms, 0.99),
        "max_ms": max(host_spreads_ms) if host_spreads_ms else float("nan"),
        "outlier_threshold_ms": (period_ns * max_offset_residual_frames) / 1e6,
    }
    threshold_ms = float(host_stats["outlier_threshold_ms"])
    host_stats["outlier_frames"] = sum(1 for x in host_spreads_ms if x > threshold_ms)

    reference_cam = expected[0]
    drift: Dict[str, Dict[str, Any]] = {}
    for cam in expected:
        if cam == reference_cam:
            drift[cam] = {"reference": True, "robust_start_to_end_ms": 0.0}
            continue
        delta, block_medians = _block_median_drift_ms(
            frame_maps[reference_cam],
            frame_maps[cam],
            valid_triggers,
            common_start,
            common_end,
        )
        drift[cam] = {
            "reference": False,
            "robust_start_to_end_ms": delta,
            "block_median_min_ms": min(block_medians) if block_medians else None,
            "block_median_max_ms": max(block_medians) if block_medians else None,
        }

    camera_summary: Dict[str, Dict[str, Any]] = {}
    for cam in expected:
        run = runs[cam]
        cadence = _camera_cadence_ms(run)
        missing = missing_by_cam[cam]
        missing_ranges = []
        for a, b in _ranges(missing):
            missing_ranges.append(
                {
                    "global_start": a,
                    "global_end": b,
                    "frames": b - a + 1,
                    "sync_index_start": a - common_start,
                    "sync_index_end": b - common_start,
                }
            )
        camera_summary[cam] = {
            "audit_file": str(run.audit_path),
            "metadata_file": str(run.metadata_path) if run.metadata_path else None,
            "hardware_trigger": run.hardware_trigger,
            "expected_acquisition_fps": run.expected_fps,
            "expected_fps_source": run.expected_fps_source,
            "audit_header_expected_fps": _safe_float_header(run.audit_header, "expected_fps"),
            "mp4_playback_fps": run.playback_fps,
            "recorded_frames": run.recorded_frames,
            "camera_frame_number_first": run.start_frame_number,
            "camera_frame_number_last": run.end_frame_number,
            "global_trigger_offset": run.inferred_offset,
            "global_trigger_first": min(frame_maps[cam]),
            "global_trigger_last": max(frame_maps[cam]),
            "missing_common_frames": len(missing),
            "missing_ranges": missing_ranges,
            "camera_cadence_median_ms": float(statistics.median(cadence)) if cadence else None,
            "camera_cadence_min_ms": min(cadence) if cadence else None,
            "camera_cadence_max_ms": max(cadence) if cadence else None,
            "host_clock_drift_vs_reference": drift[cam],
        }

    status = "pass_with_exclusions" if excluded else "pass"
    if excluded:
        headline = f"PASS WITH {len(excluded):,} EXCLUDED FRAMES OUT OF {total:,}"
    else:
        headline = f"PASS: 0 EXCLUDED FRAMES OUT OF {total:,}"

    result.status = status
    result.ok = True
    result.headline = headline
    result.total_common_frames = total
    result.valid_frames = valid
    result.excluded_frames = len(excluded)
    result.valid_percent = valid_pct

    alignment_path = output_dir / "multicam_sync_alignment.csv"
    with alignment_path.open("w", encoding="utf-8", newline="") as fh:
        fields = ["global_trigger", "sync_frame_index", "all_cameras_valid", "missing_cameras"]
        for cam in expected:
            fields.extend(
                [
                    f"{cam}_mp4_frame",
                    f"{cam}_camera_frame_number",
                    f"{cam}_pc_utc_ns",
                    f"{cam}_pc_utc_iso",
                    f"{cam}_camera_timestamp_ns",
                ]
            )
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for g in range(common_start, common_end + 1):
            missing_cams = [cam for cam in expected if g not in frame_maps[cam]]
            row_out: Dict[str, Any] = {
                "global_trigger": g,
                "sync_frame_index": g - common_start,
                "all_cameras_valid": 0 if missing_cams else 1,
                "missing_cameras": ",".join(missing_cams),
            }
            for cam in expected:
                row = frame_maps[cam].get(g)
                if row is None:
                    row_out[f"{cam}_mp4_frame"] = ""
                    row_out[f"{cam}_camera_frame_number"] = ""
                    row_out[f"{cam}_pc_utc_ns"] = ""
                    row_out[f"{cam}_pc_utc_iso"] = ""
                    row_out[f"{cam}_camera_timestamp_ns"] = ""
                else:
                    row_out[f"{cam}_mp4_frame"] = row["frame_index"]
                    row_out[f"{cam}_camera_frame_number"] = row["camera_frame_number"]
                    row_out[f"{cam}_pc_utc_ns"] = row["pc_utc_ns"]
                    row_out[f"{cam}_pc_utc_iso"] = row["pc_utc_iso"]
                    row_out[f"{cam}_camera_timestamp_ns"] = row["camera_timestamp_ns"]
            writer.writerow(row_out)

    summary: Dict[str, Any] = {
        "multicam_sync_audit_version": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "session": session,
        "status": status,
        "headline": headline,
        "camera_count": len(expected),
        "cameras": expected,
        "acquisition": {
            "hardware_triggered": True,
            "expected_fps": fps,
            "expected_interval_ns": period_ns,
        },
        "alignment": {
            "reference_camera_for_clock_drift": reference_cam,
            "common_global_trigger_first": common_start,
            "common_global_trigger_last": common_end,
            "common_frames": total,
            "fully_synchronized_frames": valid,
            "excluded_frames": len(excluded),
            "valid_percent": valid_pct,
            "offset_origin_residual_ms": offset_residual_ns / 1e6,
            "offset_origin_residual_frames": offset_residual_frames,
            "excluded_ranges": [
                {
                    "global_start": a,
                    "global_end": b,
                    "frames": b - a + 1,
                    "sync_index_start": a - common_start,
                    "sync_index_end": b - common_start,
                    "missing_cameras": sorted(
                        cam for cam in expected if any(g in set(missing_by_cam[cam]) for g in range(a, b + 1))
                    ),
                }
                for a, b in _ranges(excluded)
            ],
        },
        "host_receipt_timing": host_stats,
        "camera_details": camera_summary,
        "warnings": result.warnings,
        "errors": result.errors,
        "outputs": {
            "alignment_csv": str(alignment_path),
            "report_txt": str(output_dir / "multicam_sync_report.txt"),
            "summary_yaml": str(output_dir / "multicam_sync_summary.yaml"),
        },
    }

    summary_path = output_dir / "multicam_sync_summary.yaml"
    _write_yaml(summary_path, summary)

    report_path = output_dir / "multicam_sync_report.txt"
    _write_report(report_path, summary)
    _status_marker(output_dir, status)

    result.report_path = report_path
    result.summary_path = summary_path
    result.alignment_path = alignment_path
    _log(log, f"multicam sync: {headline}; valid={valid:,}/{total:,} ({valid_pct:.3f}%)")
    return result


def _write_failure_outputs(result: SyncAuditResult) -> None:
    result.ok = False
    if result.status not in ("fail", "incomplete"):
        result.status = "incomplete"
    if result.status == "fail":
        result.headline = "MULTI-CAMERA SYNC AUDIT FAIL"
    else:
        result.headline = "MULTI-CAMERA SYNC AUDIT INCOMPLETE"
    summary = {
        "multicam_sync_audit_version": 1,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "session": result.session,
        "status": result.status,
        "headline": result.headline,
        "warnings": result.warnings,
        "errors": result.errors,
    }
    result.summary_path = result.output_dir / "multicam_sync_summary.yaml"
    result.report_path = result.output_dir / "multicam_sync_report.txt"
    _write_yaml(result.summary_path, summary)
    _write_report(result.report_path, summary)
    _status_marker(result.output_dir, result.status)


def _write_report(path: Path, summary: Mapping[str, Any]) -> None:
    lines: List[str] = []
    lines.append("=" * 78)
    lines.append("MULTI-CAMERA SYNC AUDIT")
    lines.append("=" * 78)
    lines.append(f"Session: {summary.get('session', '')}")
    lines.append(f"Generated UTC: {summary.get('generated_utc', '')}")
    lines.append("")
    lines.append(str(summary.get("headline", summary.get("status", "UNKNOWN"))).upper())

    alignment = summary.get("alignment")
    acquisition = summary.get("acquisition")
    cameras = summary.get("camera_details")
    if isinstance(alignment, dict) and isinstance(acquisition, dict) and isinstance(cameras, dict):
        lines.append(
            f"Fully synchronized: {int(alignment.get('fully_synchronized_frames', 0)):,} / "
            f"{int(alignment.get('common_frames', 0)):,} "
            f"({float(alignment.get('valid_percent', 0.0)):.3f}%)"
        )
        lines.append(
            f"Acquisition: hardware triggered @ {float(acquisition.get('expected_fps', 0.0)):g} Hz "
            f"({int(acquisition.get('expected_interval_ns', 0))} ns nominal interval)"
        )
        lines.append(
            f"Common global trigger range: {alignment.get('common_global_trigger_first')} .. "
            f"{alignment.get('common_global_trigger_last')}"
        )
        lines.append(
            f"Integer-offset origin residual: {float(alignment.get('offset_origin_residual_ms', 0.0)):.3f} ms "
            f"({float(alignment.get('offset_origin_residual_frames', 0.0)):.4f} frames)"
        )

        lines.append("")
        lines.append("CAMERA START / LOSS SUMMARY")
        lines.append("-" * 78)
        for cam, info_any in cameras.items():
            info = info_any if isinstance(info_any, dict) else {}
            playback = info.get("mp4_playback_fps")
            playback_text = f"{float(playback):g}" if isinstance(playback, (int, float)) else "unknown"
            lines.append(
                f"{cam:8s} offset={int(info.get('global_trigger_offset', 0)):6d}  "
                f"recorded={int(info.get('recorded_frames', 0)):7,d}  "
                f"missing_common={int(info.get('missing_common_frames', 0)):5,d}  "
                f"acq_fps={float(info.get('expected_acquisition_fps', 0.0)):g}  "
                f"playback_fps={playback_text}"
            )

        excluded_ranges = alignment.get("excluded_ranges", [])
        lines.append("")
        lines.append("EXCLUDED TRIGGER RANGES")
        lines.append("-" * 78)
        if excluded_ranges:
            for item_any in excluded_ranges:
                item = item_any if isinstance(item_any, dict) else {}
                cams = ",".join(item.get("missing_cameras", []))
                lines.append(
                    f"global {item.get('global_start')}..{item.get('global_end')}  "
                    f"frames={item.get('frames')}  "
                    f"sync_index={item.get('sync_index_start')}..{item.get('sync_index_end')}  "
                    f"missing={cams}"
                )
        else:
            lines.append("none")

        lines.append("")
        lines.append("CAMERA CLOCK CADENCE")
        lines.append("-" * 78)
        for cam, info_any in cameras.items():
            info = info_any if isinstance(info_any, dict) else {}
            lines.append(
                f"{cam:8s} median={float(info.get('camera_cadence_median_ms', float('nan'))):.6f} ms  "
                f"min={float(info.get('camera_cadence_min_ms', float('nan'))):.6f} ms  "
                f"max={float(info.get('camera_cadence_max_ms', float('nan'))):.6f} ms"
            )

        timing = summary.get("host_receipt_timing", {})
        if isinstance(timing, dict):
            lines.append("")
            lines.append("HOST UTC / RECEIPT TIMING")
            lines.append("-" * 78)
            lines.append(
                "Cross-camera receipt-time spread on fully present triggers: "
                f"median={float(timing.get('median_ms', float('nan'))):.3f} ms, "
                f"p95={float(timing.get('p95_ms', float('nan'))):.3f} ms, "
                f"p99={float(timing.get('p99_ms', float('nan'))):.3f} ms, "
                f"max={float(timing.get('max_ms', float('nan'))):.3f} ms"
            )
            lines.append(
                f"Receipt-delay outlier frames > {float(timing.get('outlier_threshold_ms', 0.0)):.3f} ms: "
                f"{int(timing.get('outlier_frames', 0)):,}"
            )
            ref = alignment.get("reference_camera_for_clock_drift")
            lines.append(f"Robust block-median host clock drift relative to {ref}:")
            for cam, info_any in cameras.items():
                info = info_any if isinstance(info_any, dict) else {}
                drift = info.get("host_clock_drift_vs_reference", {})
                if isinstance(drift, dict):
                    value = drift.get("robust_start_to_end_ms")
                    if isinstance(value, (int, float)):
                        lines.append(f"  {cam}: {float(value):+.3f} ms over common interval")

        lines.append("")
        lines.append("INTERPRETATION")
        lines.append("-" * 78)
        if summary.get("status") == "pass_with_exclusions":
            lines.append(
                "PASS WITH EXCLUSIONS means the shared trigger axis is unambiguous, but one or "
                "more trigger epochs lack a frame from at least one camera. Downstream multi-camera "
                "analysis should use all_cameras_valid==1 from multicam_sync_alignment.csv."
            )
        else:
            lines.append(
                "PASS means every trigger epoch in the common camera interval has a frame from every camera."
            )
        lines.append(
            "Host UTC receipt time is used to infer the integer start offset between cameras. "
            "Camera frame numbers then carry the exact alignment through dropped frames; USB/driver "
            "receipt stalls therefore do not create cumulative movie de-sync."
        )

    warnings = summary.get("warnings", [])
    errors = summary.get("errors", [])
    if warnings:
        lines.append("")
        lines.append("WARNINGS")
        lines.append("-" * 78)
        for item in warnings:
            lines.append(f"- {item}")
    if errors:
        lines.append("")
        lines.append("ERRORS")
        lines.append("-" * 78)
        for item in errors:
            lines.append(f"- {item}")

    lines.append("")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _infer_local_cameras(session_dir: Path, processed_subdir: str) -> List[str]:
    names = set()
    inputs = session_dir / "multicam_sync" / "inputs"
    if inputs.is_dir():
        names.update(p.name for p in inputs.iterdir() if p.is_dir())
    for p in session_dir.iterdir() if session_dir.is_dir() else []:
        if p.is_dir() and (p / processed_subdir).is_dir() and p.name.startswith("cam"):
            names.add(p.name)
    return sorted(names)


def _build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Reconstruct and audit a shared hardware-trigger frame axis from per-camera raw audit CSVs."
    )
    p.add_argument("session_dir", type=Path, help="Master or consolidated session directory")
    p.add_argument(
        "--camera",
        action="append",
        default=[],
        help="Expected camera token, e.g. cam1, cam1@cam1, cam1@local. Repeat for each camera.",
    )
    p.add_argument("--remote-sessions-root", default="/home/spencelab/camera_sessions")
    p.add_argument("--processed-subdir", default="processed")
    p.add_argument("--ssh-user", default="spencelab")
    p.add_argument(
        "--collect",
        action="store_true",
        help="Collect audit/metadata inputs from --camera hosts before analysis.",
    )
    p.add_argument("--json", action="store_true", help="Print a compact machine-readable result JSON")
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    session_dir = args.session_dir.expanduser().resolve()
    camera_hosts: Dict[str, str] = {}
    cameras: List[str] = []
    for token in args.camera:
        cam, host = parse_camera_token(token)
        cameras.append(cam)
        camera_hosts[cam] = host
    if not cameras:
        cameras = _infer_local_cameras(session_dir, args.processed_subdir)
    if not cameras:
        print("No cameras found. Pass --camera cam1@cam1 --camera cam2@cam2 ...", file=sys.stderr)
        return 2

    result = audit_session(
        session_dir,
        expected_cameras=cameras,
        camera_hosts=camera_hosts or None,
        remote_sessions_root=args.remote_sessions_root,
        processed_subdir=args.processed_subdir,
        ssh_user=args.ssh_user,
        collect=bool(args.collect),
        log=(lambda msg: print(msg, file=sys.stderr)) if not args.json else None,
    )
    if args.json:
        print(
            json.dumps(
                {
                    "status": result.status,
                    "ok": result.ok,
                    "headline": result.headline,
                    "session": result.session,
                    "total_common_frames": result.total_common_frames,
                    "valid_frames": result.valid_frames,
                    "excluded_frames": result.excluded_frames,
                    "valid_percent": result.valid_percent,
                    "report": str(result.report_path) if result.report_path else None,
                    "summary": str(result.summary_path) if result.summary_path else None,
                    "alignment": str(result.alignment_path) if result.alignment_path else None,
                    "warnings": result.warnings,
                    "errors": result.errors,
                },
                indent=2,
            )
        )
    else:
        print(result.headline)
        if result.total_common_frames:
            print(
                f"{result.valid_frames:,} synchronized frames valid / {result.total_common_frames:,} "
                f"({result.valid_percent:.3f}%)"
            )
        if result.report_path:
            print(f"Report: {result.report_path}")
        for warning in result.warnings:
            print(f"WARNING: {warning}", file=sys.stderr)
        for error in result.errors:
            print(f"ERROR: {error}", file=sys.stderr)
    return result.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
