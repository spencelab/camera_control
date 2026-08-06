#!/usr/bin/env python3
"""Plot treadmill speed versus synchronized time from a session telemetry bag.

Examples:
  python3 tools/plot_treadmill_bag.py ~/camera_sessions/<session>/rosbag/telemetry_001
  python3 tools/plot_treadmill_bag.py ~/camera_sessions/<session> --show

The script writes a PNG and CSV beside the session's rosbag directory and prints
an intentionally simple pass/fail-style summary for pilot-day sanity checks.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional


STATUS_TOPIC = "/treadmill_host/status"
EVENT_TOPIC = "/treadmill_host/events"


@dataclass
class StatusSample:
    source_ns: int
    bag_ns: int
    commanded: int
    reported: int
    running: bool
    connected: bool
    controlled: bool


@dataclass
class EventSample:
    source_ns: int
    bag_ns: int
    event_type: str
    speed: int
    running: bool
    controlled: bool
    source: str
    message: str


def _stamp_ns(header) -> int:
    try:
        stamp = header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except Exception:
        return 0


def _find_bag(path: Path) -> Path:
    path = path.expanduser().resolve()
    if path.is_file() and path.suffix == ".mcap":
        return path.parent
    if (path / "metadata.yaml").is_file():
        return path

    candidates = sorted(
        (p for p in path.glob("rosbag/telemetry_*") if (p / "metadata.yaml").is_file()),
        key=lambda p: p.stat().st_mtime_ns,
    )
    if not candidates:
        candidates = sorted(
            (p for p in path.glob("telemetry_*") if (p / "metadata.yaml").is_file()),
            key=lambda p: p.stat().st_mtime_ns,
        )
    if not candidates:
        raise FileNotFoundError(f"no telemetry bag with metadata.yaml found under {path}")
    return candidates[-1]


def _storage_id(bag_dir: Path) -> str:
    text = (bag_dir / "metadata.yaml").read_text(encoding="utf-8", errors="replace")
    match = re.search(r"^\s*storage_identifier:\s*['\"]?([^'\"\s]+)", text, re.MULTILINE)
    if match:
        return match.group(1)
    if any(bag_dir.glob("*.mcap")):
        return "mcap"
    if any(bag_dir.glob("*.db3")):
        return "sqlite3"
    return "mcap"


def read_treadmill_messages(bag_dir: Path) -> tuple[list[StatusSample], list[EventSample]]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as exc:
        raise RuntimeError(
            "ROS 2 Python bag libraries are unavailable. Source /opt/ros/$ROS_DISTRO/setup.bash "
            "and ~/ros2_ws/install/setup.bash before running this script."
        ) from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=_storage_id(bag_dir)),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}

    missing = [topic for topic in (STATUS_TOPIC, EVENT_TOPIC) if topic not in topic_types]
    if len(missing) == 2:
        raise RuntimeError(
            f"bag contains neither {STATUS_TOPIC} nor {EVENT_TOPIC}; topics are: "
            + ", ".join(sorted(topic_types))
        )

    type_cache = {topic: get_message(type_name) for topic, type_name in topic_types.items() if topic in {STATUS_TOPIC, EVENT_TOPIC}}
    statuses: list[StatusSample] = []
    events: list[EventSample] = []

    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        if topic not in type_cache:
            continue
        msg = deserialize_message(data, type_cache[topic])
        source_ns = _stamp_ns(getattr(msg, "header", None)) or int(bag_ns)
        if topic == STATUS_TOPIC:
            statuses.append(
                StatusSample(
                    source_ns=source_ns,
                    bag_ns=int(bag_ns),
                    commanded=int(getattr(msg, "commanded_speed_cm_s", -1)),
                    reported=int(getattr(msg, "reported_speed_cm_s", -1)),
                    running=bool(getattr(msg, "running", False)),
                    connected=bool(getattr(msg, "connected", False)),
                    controlled=bool(getattr(msg, "controlled", False)),
                )
            )
        elif topic == EVENT_TOPIC:
            events.append(
                EventSample(
                    source_ns=source_ns,
                    bag_ns=int(bag_ns),
                    event_type=str(getattr(msg, "event_type", "")),
                    speed=int(getattr(msg, "speed_cm_s", -1)),
                    running=bool(getattr(msg, "running", False)),
                    controlled=bool(getattr(msg, "controlled", False)),
                    source=str(getattr(msg, "source", "")),
                    message=str(getattr(msg, "message", "")),
                )
            )
    del reader
    return statuses, events


def _output_base(bag_dir: Path, requested: Optional[Path]) -> Path:
    if requested is not None:
        return requested.expanduser().resolve().with_suffix("")
    rosbag_dir = bag_dir.parent
    return rosbag_dir / "treadmill_speed_sanity"


def write_csv(path: Path, statuses: Iterable[StatusSample], t0_ns: int) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "source_utc_ns",
            "bag_receive_ns",
            "elapsed_s",
            "commanded_speed_cm_s",
            "reported_speed_cm_s",
            "running",
            "connected",
            "controlled",
        ])
        for sample in statuses:
            writer.writerow([
                sample.source_ns,
                sample.bag_ns,
                f"{(sample.source_ns - t0_ns) / 1e9:.9f}",
                sample.commanded,
                sample.reported,
                int(sample.running),
                int(sample.connected),
                int(sample.controlled),
            ])


def make_plot(
    path: Path,
    bag_name: str,
    statuses: list[StatusSample],
    events: list[EventSample],
    t0_ns: int,
    show: bool,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise RuntimeError("matplotlib is required: sudo apt install python3-matplotlib") from exc

    fig, ax = plt.subplots(figsize=(11, 5.5))

    if statuses:
        elapsed = [(sample.source_ns - t0_ns) / 1e9 for sample in statuses]
        commanded = [max(0, sample.commanded) if sample.running else 0 for sample in statuses]
        ax.step(elapsed, commanded, where="post", label="Commanded effective speed")

        reported_points = [
            ((sample.source_ns - t0_ns) / 1e9, sample.reported)
            for sample in statuses
            if sample.reported >= 0
        ]
        if reported_points:
            ax.plot(
                [point[0] for point in reported_points],
                [point[1] for point in reported_points],
                marker=".",
                linewidth=1.2,
                label="Reported speed",
            )

    command_events = [event for event in events if event.event_type == "speed_command" and event.speed >= 0]
    if command_events:
        ax.scatter(
            [(event.source_ns - t0_ns) / 1e9 for event in command_events],
            [event.speed for event in command_events],
            marker="x",
            label="Speed command events",
        )

    for event in events:
        if event.event_type in {"run", "stop"}:
            x = (event.source_ns - t0_ns) / 1e9
            ax.axvline(x, linewidth=0.8, alpha=0.35)
            ax.text(x, ax.get_ylim()[1] if ax.get_ylim()[1] > 0 else 1, event.event_type, rotation=90, va="top", ha="right", fontsize=8)

    ax.set_xlabel("Elapsed synchronized host time (s)")
    ax.set_ylabel("Treadmill speed (cm/s)")
    ax.set_title(f"Treadmill telemetry sanity check\n{bag_name}")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    if show:
        plt.show()
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_or_session", type=Path, help="telemetry bag directory, rosbag directory, or session directory")
    parser.add_argument("--output", type=Path, help="output basename; .png and .csv are added")
    parser.add_argument("--show", action="store_true", help="open the plot window after saving")
    args = parser.parse_args()

    try:
        bag_dir = _find_bag(args.bag_or_session)
        statuses, events = read_treadmill_messages(bag_dir)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    all_times = [sample.source_ns for sample in statuses] + [event.source_ns for event in events]
    if not all_times:
        print("ERROR: treadmill topics were present but contained no messages", file=sys.stderr)
        return 3
    t0_ns = min(all_times)
    t1_ns = max(all_times)

    output_base = _output_base(bag_dir, args.output)
    output_base.parent.mkdir(parents=True, exist_ok=True)
    png_path = output_base.with_suffix(".png")
    csv_path = output_base.with_suffix(".csv")

    if statuses:
        write_csv(csv_path, statuses, t0_ns)
    make_plot(png_path, bag_dir.name, statuses, events, t0_ns, args.show)

    reported_count = sum(1 for sample in statuses if sample.reported >= 0)
    speed_commands = sum(1 for event in events if event.event_type == "speed_command")
    speed_reports = sum(1 for event in events if event.event_type == "speed_report")
    max_commanded = max((sample.commanded for sample in statuses), default=-1)
    max_reported = max((sample.reported for sample in statuses if sample.reported >= 0), default=-1)

    print(f"BAG={bag_dir}")
    print(f"DURATION_S={(t1_ns - t0_ns) / 1e9:.3f}")
    print(f"STATUS_SAMPLES={len(statuses)}")
    print(f"SPEED_COMMAND_EVENTS={speed_commands}")
    print(f"SPEED_REPORT_EVENTS={speed_reports}")
    print(f"REPORTED_STATUS_SAMPLES={reported_count}")
    print(f"MAX_COMMANDED_CM_S={max_commanded}")
    print(f"MAX_REPORTED_CM_S={max_reported}")
    print(f"PLOT={png_path}")
    if statuses:
        print(f"CSV={csv_path}")
    if reported_count == 0 and speed_reports == 0:
        print(
            "WARNING=No physical speed reports were recorded. Commanded HIIT speed is still preserved; "
            "enable treadmill_host query_speed_hz:=2.0 if physical reports are required."
        )
    else:
        print("REPORTED_SPEED_CHECK=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
