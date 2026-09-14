#!/usr/bin/env python3
"""Record per-module Linux process resources and system load as append-only CSV."""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import signal
import time
from datetime import datetime, timezone
from pathlib import Path


DEFAULT_MODULES = {
    # Native DD-GNG and asynchronous YOLO share one process, so Linux cannot
    # split their RSS/PSS without an allocator profiler. Internal timing is
    # recorded separately on the perception_metrics topic.
    "topo_gng": r"(^|/)(topo_gng_node)(\x00|\s|$)",
    "reachability": r"(^|/)(reachability_graph_node)(\x00|\s|$)",
    "yolo_preview": r"dd_gng_yolo\.py",
    "ordinary_perception": r"om6dof_perception.*perception_node",
    "robot_state": r"robot_state_publisher",
    "move_group": r"(^|/)(move_group)(\x00|\s|$)",
    "rviz": r"(^|/)(rviz2)(\x00|\s|$)",
    "controller": r"controller_node",
    # Keep the executable boundary: rosbag's command line also contains the
    # topic /dynamixel_hardware_interface/health.
    # DynamixelHardware is loaded as a plugin inside ros2_control_node; its
    # process memory cannot be separated from controller_manager/ros2_control.
    "ros2_control": r"(^|/)(ros2_control_node)(\x00|\s|$)",
    "pick": r"direct_pick_node",
    "rosbag": r"ros2 bag record|rosbag2",
    "ros_metrics_recorder": r"ros_metrics_recorder\.py",
    "resource_monitor": r"module_resource_monitor\.py",
    "tegrastats": r"(^|/)(tegrastats)(\x00|\s|$)",
    "outside_video": r"outside_camera\.mp4",
    "desktop_video": r"rviz_desktop\.mp4",
}


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return ""


def proc_argv(pid: int) -> list[str]:
    try:
        raw = (Path("/proc") / str(pid) / "cmdline").read_bytes()
        return [
            token.decode("utf-8", errors="replace")
            for token in raw.split(b"\0") if token
        ]
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return []


def process_identity(argv: list[str]) -> str:
    """Return a stable executable identity without arbitrary CLI arguments.

    Matching a full command line lets diagnostic commands such as
    ``pgrep -af reachability_graph_node`` masquerade as the measured node.
    Native programs use argv[0], Python uses the launched script/module, and
    only recorders whose output target defines their role retain arguments.
    """
    if not argv:
        return ""
    executable = Path(argv[0]).name
    if executable.startswith("python"):
        if len(argv) >= 4 and Path(argv[1]).name == "ros2" \
                and argv[2:4] == ["bag", "record"]:
            return "ros2 bag record"
        if len(argv) >= 3 and argv[1] == "-m":
            return " ".join(argv[:3])
        return " ".join(argv[:2])
    if executable == "ros2" and len(argv) >= 3 and argv[1:3] == ["bag", "record"]:
        return "ros2 bag record"
    if executable in {"ffmpeg", "gst-launch-1.0"}:
        return " ".join(argv)
    return argv[0]


def status_values(pid: int) -> dict[str, int]:
    result = {"VmRSS": 0, "Threads": 0}
    for line in read_text(Path("/proc") / str(pid) / "status").splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        if key in result:
            match = re.search(r"\d+", value)
            result[key] = int(match.group()) if match else 0
    return result


def pss_kb(pid: int) -> int | None:
    for line in read_text(Path("/proc") / str(pid) / "smaps_rollup").splitlines():
        if line.startswith("Pss:"):
            return int(line.split()[1])
    return None


def cpu_ticks(pid: int) -> int:
    stat = read_text(Path("/proc") / str(pid) / "stat")
    close = stat.rfind(")")
    if close < 0:
        return 0
    fields = stat[close + 2 :].split()
    try:
        return int(fields[11]) + int(fields[12])
    except (IndexError, ValueError):
        return 0


def io_bytes(pid: int) -> tuple[int, int] | None:
    values = {"read_bytes": 0, "write_bytes": 0}
    found: set[str] = set()
    for line in read_text(Path("/proc") / str(pid) / "io").splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        if key in values:
            try:
                values[key] = int(value.strip())
                found.add(key)
            except ValueError:
                pass
    if found != set(values):
        return None
    return values["read_bytes"], values["write_bytes"]


def meminfo() -> dict[str, int]:
    values: dict[str, int] = {}
    for line in read_text(Path("/proc/meminfo")).splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            match = re.search(r"\d+", value)
            if match:
                values[key] = int(match.group())
    return values


def system_cpu_ticks() -> tuple[int, int]:
    first = read_text(Path("/proc/stat")).splitlines()[0].split()
    ticks = [int(value) for value in first[1:]]
    idle = ticks[3] + (ticks[4] if len(ticks) > 4 else 0)
    return sum(ticks), idle


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low = int(index)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def parse_modules(items: list[str]) -> dict[str, re.Pattern[str]]:
    definitions = dict(DEFAULT_MODULES)
    for item in items:
        if "=" not in item:
            raise ValueError(f"module must be NAME=REGEX, got {item!r}")
        name, pattern = item.split("=", 1)
        definitions[name] = pattern
    return {name: re.compile(pattern) for name, pattern in definitions.items()}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--period", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=0.0, help="0 means until interrupted")
    parser.add_argument("--module", action="append", default=[], help="additional NAME=REGEX")
    args = parser.parse_args()
    if args.period <= 0:
        parser.error("--period must be positive")

    modules = parse_modules(args.module)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    module_path = args.output_dir / "module_resources.csv"
    system_path = args.output_dir / "system_resources.csv"
    conflict_path = args.output_dir / "process_match_conflicts.csv"
    module_fields = [
        "utc_iso", "unix_ns", "monotonic_ns", "module", "pid_count", "pids",
        "process_identities_json",
        "cpu_percent", "cpu_sample_complete", "cpu_valid_pid_count",
        "rss_mb", "pss_mb", "pss_sample_complete", "pss_valid_pid_count", "threads",
        "read_mb_s", "write_mb_s", "io_sample_complete", "io_valid_pid_count",
        "sample_interval_sec",
    ]
    system_fields = [
        "utc_iso", "unix_ns", "monotonic_ns", "cpu_percent", "load1", "load5", "load15",
        "logical_cpu_count", "mem_total_mb", "mem_available_mb", "mem_used_percent", "swap_used_mb",
        "sample_interval_sec", "cpu_sample_complete",
    ]
    conflict_fields = [
        "utc_iso", "unix_ns", "monotonic_ns", "pid", "matching_modules", "process_identity",
    ]

    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    clock_ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    previous_proc: dict[tuple[str, int], tuple[int, int | None, int | None]] = {}
    previous_total, previous_idle = system_cpu_ticks()
    previous_time = time.monotonic()
    started = previous_time
    sample_index = 0

    with module_path.open("w", newline="", encoding="utf-8") as module_handle, system_path.open(
        "w", newline="", encoding="utf-8"
    ) as system_handle, conflict_path.open("w", newline="", encoding="utf-8") as conflict_handle:
        module_writer = csv.DictWriter(module_handle, fieldnames=module_fields)
        system_writer = csv.DictWriter(system_handle, fieldnames=system_fields)
        conflict_writer = csv.DictWriter(conflict_handle, fieldnames=conflict_fields)
        module_writer.writeheader()
        system_writer.writeheader()
        conflict_writer.writeheader()

        while not stopping and (args.duration <= 0 or time.monotonic() - started < args.duration):
            loop_started = time.monotonic()
            utc = datetime.now(timezone.utc).isoformat()
            unix_ns = time.time_ns()
            monotonic_ns = time.monotonic_ns()
            elapsed = max(loop_started - previous_time, 1e-6)

            process_list: list[tuple[int, str]] = []
            for entry in Path("/proc").iterdir():
                if entry.name.isdigit():
                    pid = int(entry.name)
                    identity = process_identity(proc_argv(pid))
                    if identity:
                        process_list.append((pid, identity))

            # A PID is owned by exactly one reported module. Ambiguous regex
            # matches are excluded and recorded instead of silently double
            # counting the same CPU and memory under multiple module names.
            assigned: dict[str, list[int]] = {name: [] for name in modules}
            identity_by_pid = dict(process_list)
            for pid, identity in process_list:
                matches = [name for name, pattern in modules.items() if pattern.search(identity)]
                if len(matches) == 1:
                    assigned[matches[0]].append(pid)
                elif len(matches) > 1:
                    conflict_writer.writerow({
                        "utc_iso": utc,
                        "unix_ns": unix_ns,
                        "monotonic_ns": monotonic_ns,
                        "pid": pid,
                        "matching_modules": ";".join(matches),
                        "process_identity": identity,
                    })

            current_keys: set[tuple[str, int]] = set()
            for module in modules:
                pids = assigned[module]
                total_cpu = 0.0
                total_rss = 0
                total_pss = 0
                total_threads = 0
                total_read_rate = 0.0
                total_write_rate = 0.0
                cpu_valid_pid_count = 0
                io_valid_pid_count = 0
                pss_valid_pid_count = 0
                for pid in pids:
                    ticks = cpu_ticks(pid)
                    io_sample = io_bytes(pid)
                    read_bytes = io_sample[0] if io_sample is not None else None
                    write_bytes = io_sample[1] if io_sample is not None else None
                    key = (module, pid)
                    current_keys.add(key)
                    prior = previous_proc.get(key)
                    if prior is not None:
                        total_cpu += max(0, ticks - prior[0]) / clock_ticks / elapsed * 100.0
                        cpu_valid_pid_count += 1
                        if read_bytes is not None and write_bytes is not None \
                                and prior[1] is not None and prior[2] is not None:
                            total_read_rate += max(0, read_bytes - prior[1]) / elapsed / (1024 * 1024)
                            total_write_rate += max(0, write_bytes - prior[2]) / elapsed / (1024 * 1024)
                            io_valid_pid_count += 1
                    previous_proc[key] = (ticks, read_bytes, write_bytes)
                    status = status_values(pid)
                    total_rss += status["VmRSS"]
                    pss = pss_kb(pid)
                    if pss is not None:
                        total_pss += pss
                        pss_valid_pid_count += 1
                    total_threads += status["Threads"]
                cpu_complete = bool(pids) and cpu_valid_pid_count == len(pids)
                io_complete = bool(pids) and io_valid_pid_count == len(pids)
                pss_complete = bool(pids) and pss_valid_pid_count == len(pids)
                module_writer.writerow({
                    "utc_iso": utc,
                    "unix_ns": unix_ns,
                    "monotonic_ns": monotonic_ns,
                    "module": module,
                    "pid_count": len(pids),
                    "pids": ";".join(str(pid) for pid in pids),
                    "process_identities_json": json.dumps(
                        [identity_by_pid[pid] for pid in pids], separators=(",", ":")
                    ),
                    "cpu_percent": f"{total_cpu:.6f}",
                    "cpu_sample_complete": int(cpu_complete),
                    "cpu_valid_pid_count": cpu_valid_pid_count,
                    "rss_mb": f"{total_rss / 1024:.6f}",
                    "pss_mb": f"{total_pss / 1024:.6f}" if pss_complete else "",
                    "pss_sample_complete": int(pss_complete),
                    "pss_valid_pid_count": pss_valid_pid_count,
                    "threads": total_threads,
                    "read_mb_s": f"{total_read_rate:.6f}",
                    "write_mb_s": f"{total_write_rate:.6f}",
                    "io_sample_complete": int(io_complete),
                    "io_valid_pid_count": io_valid_pid_count,
                    "sample_interval_sec": f"{elapsed:.9f}",
                })
            previous_proc = {key: value for key, value in previous_proc.items() if key in current_keys}

            total, idle = system_cpu_ticks()
            delta_total = max(total - previous_total, 1)
            cpu_percent = 100.0 * (1.0 - max(0, idle - previous_idle) / delta_total)
            previous_total, previous_idle = total, idle
            memory = meminfo()
            mem_total = memory.get("MemTotal", 0)
            mem_available = memory.get("MemAvailable", 0)
            swap_used = memory.get("SwapTotal", 0) - memory.get("SwapFree", 0)
            loads = os.getloadavg()
            system_writer.writerow({
                "utc_iso": utc,
                "unix_ns": unix_ns,
                "monotonic_ns": monotonic_ns,
                "cpu_percent": f"{cpu_percent:.6f}",
                "load1": f"{loads[0]:.6f}",
                "load5": f"{loads[1]:.6f}",
                "load15": f"{loads[2]:.6f}",
                "logical_cpu_count": os.cpu_count() or 0,
                "mem_total_mb": f"{mem_total / 1024:.6f}",
                "mem_available_mb": f"{mem_available / 1024:.6f}",
                "mem_used_percent": f"{(100.0 * (mem_total - mem_available) / mem_total) if mem_total else 0:.6f}",
                "swap_used_mb": f"{swap_used / 1024:.6f}",
                "sample_interval_sec": f"{elapsed:.9f}",
                "cpu_sample_complete": int(sample_index > 0),
            })
            module_handle.flush()
            system_handle.flush()
            conflict_handle.flush()
            previous_time = loop_started
            sample_index += 1
            delay = args.period - (time.monotonic() - loop_started)
            if delay > 0:
                time.sleep(delay)


if __name__ == "__main__":
    main()
