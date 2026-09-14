#!/usr/bin/env python3
"""Convert NVIDIA tegrastats text into a tidy CSV and a JSON summary.

The parser intentionally emits no numeric value for a field that is absent or
for a CPU core reported as ``off``.  It accepts the small format differences
seen across JetPack/tegrastats releases, including scalar or per-GPC GR3D
frequencies and platform-specific power-rail names.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import statistics
import sys
from collections import defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import TextIO


NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)"
TIMESTAMP_RE = re.compile(
    r"^\s*(?P<timestamp>(?:\d{2}-\d{2}-\d{4}|\d{4}-\d{2}-\d{2})"
    r"[ T]\d{2}:\d{2}:\d{2}(?:\.\d+)?)\s+"
)
RAM_RE = re.compile(
    rf"\bRAM\s+(?P<used>{NUMBER})/(?P<total>{NUMBER})MB"
    rf"(?:\s+\(lfb\s+(?P<blocks>\d+)x(?P<block_size>{NUMBER})MB\))?"
)
SWAP_RE = re.compile(
    rf"\bSWAP\s+(?P<used>{NUMBER})/(?P<total>{NUMBER})MB"
    rf"(?:\s+\(cached\s+(?P<cached>{NUMBER})MB\))?"
)
CPU_RE = re.compile(r"\bCPU\s+\[(?P<cores>[^\]]*)\]")
CPU_VALUE_RE = re.compile(
    rf"^\s*(?P<load>{NUMBER})%(?:@(?P<frequency>{NUMBER}))?\s*$"
)
GR3D_RE = re.compile(
    rf"\bGR3D_FREQ\s+(?P<load>{NUMBER})%"
    rf"(?:@(?:\[(?P<frequencies>[^\]]*)\]|(?P<frequency>{NUMBER})))?"
)
EMC_RE = re.compile(
    rf"\bEMC_FREQ\s+(?P<load>{NUMBER})%(?:@(?P<frequency>{NUMBER}))?"
)
TEMPERATURE_RE = re.compile(
    rf"(?<![A-Za-z0-9_])(?P<sensor>[A-Za-z][A-Za-z0-9_]*)@"
    rf"(?P<temperature>{NUMBER})C\b"
)
POWER_RE = re.compile(
    rf"\b(?P<rail>(?:VDD|VIN|POM)_[A-Za-z0-9_]+)\s+"
    rf"(?P<instantaneous>{NUMBER})mW"
    rf"(?:/(?P<average>{NUMBER})mW)?"
)


@dataclass(frozen=True)
class MetricRow:
    sample_index: int
    source_line: int
    source_timestamp: str
    metric: str
    component: str
    value: float | None
    unit: str
    status: str


def add_numeric(
    rows: list[MetricRow],
    sample_index: int,
    source_line: int,
    source_timestamp: str,
    metric: str,
    component: str,
    value: str | None,
    unit: str,
) -> None:
    if value is None:
        return
    rows.append(
        MetricRow(
            sample_index=sample_index,
            source_line=source_line,
            source_timestamp=source_timestamp,
            metric=metric,
            component=component,
            value=float(value),
            unit=unit,
            status="",
        )
    )


def parse_line(line: str, sample_index: int, source_line: int) -> list[MetricRow]:
    """Parse one tegrastats sample; absent metrics produce no fabricated row."""
    rows: list[MetricRow] = []
    timestamp_match = TIMESTAMP_RE.match(line)
    timestamp = timestamp_match.group("timestamp") if timestamp_match else ""

    ram = RAM_RE.search(line)
    if ram:
        add_numeric(rows, sample_index, source_line, timestamp, "ram_used", "system", ram["used"], "MB")
        add_numeric(rows, sample_index, source_line, timestamp, "ram_total", "system", ram["total"], "MB")
        add_numeric(rows, sample_index, source_line, timestamp, "ram_lfb_blocks", "system", ram["blocks"], "count")
        add_numeric(
            rows,
            sample_index,
            source_line,
            timestamp,
            "ram_lfb_block_size",
            "system",
            ram["block_size"],
            "MB",
        )

    swap = SWAP_RE.search(line)
    if swap:
        add_numeric(rows, sample_index, source_line, timestamp, "swap_used", "system", swap["used"], "MB")
        add_numeric(rows, sample_index, source_line, timestamp, "swap_total", "system", swap["total"], "MB")
        add_numeric(rows, sample_index, source_line, timestamp, "swap_cached", "system", swap["cached"], "MB")

    cpu = CPU_RE.search(line)
    if cpu:
        for core_index, entry in enumerate(cpu["cores"].split(",")):
            component = f"cpu{core_index}"
            stripped = entry.strip()
            if stripped.lower() == "off":
                rows.append(
                    MetricRow(
                        sample_index,
                        source_line,
                        timestamp,
                        "cpu_state",
                        component,
                        None,
                        "",
                        "off",
                    )
                )
                continue
            cpu_value = CPU_VALUE_RE.match(stripped)
            if cpu_value:
                add_numeric(
                    rows,
                    sample_index,
                    source_line,
                    timestamp,
                    "cpu_load",
                    component,
                    cpu_value["load"],
                    "percent",
                )
                add_numeric(
                    rows,
                    sample_index,
                    source_line,
                    timestamp,
                    "cpu_frequency",
                    component,
                    cpu_value["frequency"],
                    "MHz",
                )
            elif stripped:
                rows.append(
                    MetricRow(
                        sample_index,
                        source_line,
                        timestamp,
                        "cpu_state",
                        component,
                        None,
                        "",
                        stripped,
                    )
                )

    gr3d = GR3D_RE.search(line)
    if gr3d:
        add_numeric(rows, sample_index, source_line, timestamp, "gr3d_load", "gpu", gr3d["load"], "percent")
        if gr3d["frequencies"] is not None:
            for gpu_index, frequency in enumerate(gr3d["frequencies"].split(",")):
                frequency = frequency.strip()
                if re.fullmatch(NUMBER, frequency):
                    add_numeric(
                        rows,
                        sample_index,
                        source_line,
                        timestamp,
                        "gr3d_frequency",
                        f"gpu{gpu_index}",
                        frequency,
                        "MHz",
                    )
        else:
            add_numeric(
                rows,
                sample_index,
                source_line,
                timestamp,
                "gr3d_frequency",
                "gpu",
                gr3d["frequency"],
                "MHz",
            )

    emc = EMC_RE.search(line)
    if emc:
        add_numeric(rows, sample_index, source_line, timestamp, "emc_load", "memory_controller", emc["load"], "percent")
        add_numeric(
            rows,
            sample_index,
            source_line,
            timestamp,
            "emc_frequency",
            "memory_controller",
            emc["frequency"],
            "MHz",
        )

    for temperature in TEMPERATURE_RE.finditer(line):
        add_numeric(
            rows,
            sample_index,
            source_line,
            timestamp,
            "temperature",
            temperature["sensor"],
            temperature["temperature"],
            "C",
        )

    for power in POWER_RE.finditer(line):
        add_numeric(
            rows,
            sample_index,
            source_line,
            timestamp,
            "power_instantaneous",
            power["rail"],
            power["instantaneous"],
            "mW",
        )
        add_numeric(
            rows,
            sample_index,
            source_line,
            timestamp,
            "power_average",
            power["rail"],
            power["average"],
            "mW",
        )
    return rows


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low = int(index)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def describe(values: list[float]) -> dict[str, float | int | None]:
    return {
        "n": len(values),
        "mean": statistics.fmean(values) if values else None,
        "median": statistics.median(values) if values else None,
        "p90": percentile(values, 0.90),
        "p95": percentile(values, 0.95),
        "min": min(values) if values else None,
        "max": max(values) if values else None,
    }


def parse_stream(handle: TextIO) -> tuple[list[MetricRow], dict[str, object]]:
    rows: list[MetricRow] = []
    input_line_count = 0
    nonempty_line_count = 0
    blank_line_count = 0
    parsed_sample_count = 0
    unparsed_nonempty_line_numbers: list[int] = []

    for source_line, line in enumerate(handle, start=1):
        input_line_count += 1
        if not line.strip():
            blank_line_count += 1
            continue
        nonempty_line_count += 1
        parsed = parse_line(line, parsed_sample_count, source_line)
        if not parsed:
            unparsed_nonempty_line_numbers.append(source_line)
            continue
        rows.extend(parsed)
        parsed_sample_count += 1

    metadata: dict[str, object] = {
        "input_line_count": input_line_count,
        "nonempty_line_count": nonempty_line_count,
        "blank_line_count": blank_line_count,
        "parsed_sample_count": parsed_sample_count,
        "unparsed_nonempty_line_count": len(unparsed_nonempty_line_numbers),
        "unparsed_nonempty_line_numbers": unparsed_nonempty_line_numbers,
    }
    return rows, metadata


def build_summary(rows: list[MetricRow], metadata: dict[str, object], source: str) -> dict[str, object]:
    values: dict[tuple[str, str, str], list[float]] = defaultdict(list)
    samples: dict[tuple[str, str, str], set[int]] = defaultdict(set)
    states: dict[tuple[str, str, str], int] = defaultdict(int)
    observed_power_rails: set[str] = set()
    parsed_sample_count = int(metadata["parsed_sample_count"])

    for row in rows:
        if row.metric.startswith("power_"):
            observed_power_rails.add(row.component)
        if row.value is None:
            states[(row.metric, row.component, row.status)] += 1
            continue
        key = (row.metric, row.component, row.unit)
        values[key].append(row.value)
        samples[key].add(row.sample_index)

    metric_statistics: list[dict[str, object]] = []
    for metric, component, unit in sorted(values):
        key = (metric, component, unit)
        entry: dict[str, object] = {
            "metric": metric,
            "component": component,
            "unit": unit,
            "samples_observed": len(samples[key]),
            "sample_coverage": (
                len(samples[key]) / parsed_sample_count if parsed_sample_count else None
            ),
        }
        entry.update(describe(values[key]))
        metric_statistics.append(entry)

    state_counts = [
        {"metric": metric, "component": component, "status": status, "n": count}
        for (metric, component, status), count in sorted(states.items())
    ]
    observed_metrics = {row.metric for row in rows}
    requested_rails = ("VDD_IN", "VDD_CPU_GPU_CV", "VDD_SOC")
    presence = {
        "ram": "ram_used" in observed_metrics,
        "swap": "swap_used" in observed_metrics,
        "cpu_load": "cpu_load" in observed_metrics,
        "cpu_frequency": "cpu_frequency" in observed_metrics,
        "gr3d_load": "gr3d_load" in observed_metrics,
        "gr3d_frequency": "gr3d_frequency" in observed_metrics,
        "emc_load": "emc_load" in observed_metrics,
        "emc_frequency": "emc_frequency" in observed_metrics,
        "temperatures": "temperature" in observed_metrics,
        "requested_power_rails": {
            rail: rail in observed_power_rails for rail in requested_rails
        },
    }
    return {
        "schema_version": "tegrastats_tidy_v1",
        "source": source,
        **metadata,
        "numeric_metric_row_count": sum(row.value is not None for row in rows),
        "state_row_count": sum(row.value is None for row in rows),
        "observed_power_rails": sorted(observed_power_rails),
        "field_presence": presence,
        "metric_statistics": metric_statistics,
        "state_counts": state_counts,
    }


def format_value(value: float | None) -> str:
    return "" if value is None else format(value, ".12g")


def write_csv(path: Path, rows: list[MetricRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [field for field in MetricRow.__dataclass_fields__]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            output = asdict(row)
            output["value"] = format_value(row.value)
            writer.writerow(output)


def write_summary(path: Path, summary: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def output_paths(input_path: str, csv_path: Path | None, summary_path: Path | None) -> tuple[Path, Path]:
    if input_path == "-":
        if csv_path is None or summary_path is None:
            raise ValueError("--csv and --summary are required when reading from stdin")
        return csv_path, summary_path
    source = Path(input_path)
    return (
        csv_path or source.with_name(f"{source.stem}_tidy.csv"),
        summary_path or source.with_name(f"{source.stem}_summary.json"),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="tegrastats log path, or '-' for stdin")
    parser.add_argument("--csv", type=Path, help="tidy CSV output path")
    parser.add_argument("--summary", type=Path, help="JSON summary output path")
    args = parser.parse_args()
    try:
        csv_path, summary_path = output_paths(args.input, args.csv, args.summary)
    except ValueError as error:
        parser.error(str(error))

    if args.input == "-":
        rows, metadata = parse_stream(sys.stdin)
    else:
        input_path = Path(args.input)
        try:
            with input_path.open(encoding="utf-8", errors="replace") as handle:
                rows, metadata = parse_stream(handle)
        except OSError as error:
            parser.error(str(error))

    summary = build_summary(rows, metadata, args.input)
    write_csv(csv_path, rows)
    write_summary(summary_path, summary)
    print(csv_path)
    print(summary_path)
    if int(metadata["nonempty_line_count"]) and not int(metadata["parsed_sample_count"]):
        raise SystemExit("no tegrastats samples could be parsed")


if __name__ == "__main__":
    main()
