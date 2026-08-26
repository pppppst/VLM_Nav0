"""Derive reviewable, fail-closed Go2 parameter candidates from saved evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

import yaml


def _floor(value: float, quantum: float) -> float:
    return round(math.floor(value / quantum) * quantum, 9)


def _ceil(value: float, quantum: float) -> float:
    return round(math.ceil(value / quantum) * quantum, 9)


def _point_sections(report: Mapping[str, object]):
    if "point_time_alignment" in report:
        return (
            report["lidar_hz"],
            report["point_time_span_s"],
            report["point_time_alignment"],
            report["lidar_imu_offset_jitter_s"],
        )
    alignment = report["lidar_imu_alignment"]
    return (
        report["rates_and_intervals"]["cloud_rate_from_header_hz"],
        report["point_time_raw"]["maximum_s"],
        alignment["all_reconstructed_points"],
        alignment["header_nearest_imu_minus_header_s"]["range"],
    )


def derive_candidate_config(point_report: Mapping, raw_reports: Sequence[Mapping]) -> dict:
    """Return candidates with margins; never mark a formal gate validated."""

    if not raw_reports:
        raise ValueError("at least one raw LiDAR/IMU report is required")
    point_rate, point_span, all_points, point_header_jitter = _point_sections(
        point_report
    )

    rates = sorted(
        [float(point_rate), *(float(report["lidar_hz"]) for report in raw_reports)]
    )
    span_minimum = min(
        float(point_span["minimum"]),
        *(float(report["point_time_span_s"]["minimum"]) for report in raw_reports),
    )
    span_maximum = max(
        float(point_span["maximum"]),
        *(float(report["point_time_span_s"]["maximum"]) for report in raw_reports),
    )
    span_median = float(point_span["median"])
    header_jitter = max(
        float(point_header_jitter),
        *(float(report["lidar_imu_offset_jitter_s"]) for report in raw_reports),
    )

    imu_sections = [report["imu_raw"] for report in raw_reports]
    gravity_min = min(
        float(section["acceleration_norm_mps2"]["minimum"])
        for section in imu_sections
    )
    gravity_max = max(
        float(section["acceleration_norm_mps2"]["maximum"])
        for section in imu_sections
    )
    gyro_norm_max = max(
        float(section["gyro_norm_radps"]["maximum"]) for section in imu_sections
    )
    acceleration_abs_max = max(
        float(section["maximum_acceleration_abs_mps2"])
        for section in imu_sections
    )
    gyro_abs_max = max(
        float(section["maximum_gyro_abs_radps"]) for section in imu_sections
    )
    acceleration_jump_max = max(
        float(section["maximum_acceleration_jump_mps2"])
        for section in imu_sections
    )

    return {
        "candidate_only": True,
        "validated": False,
        "reason": (
            "Derived from static evidence; dynamic LiDAR/IMU phase and a saved "
            "stationary bag still require review."
        ),
        "observed": {
            "cloud_rate_hz": rates,
            "point_time_span_s": {
                "minimum": span_minimum,
                "median": span_median,
                "maximum": span_maximum,
            },
            "all_point_nearest_imu_absolute_s": all_points["absolute_offset_s"],
            "all_point_matched_count": int(all_points["matched_count"]),
            "header_nearest_imu_jitter_s": header_jitter,
            "imu": {
                "gravity_norm_min_mps2": gravity_min,
                "gravity_norm_max_mps2": gravity_max,
                "gyro_norm_max_radps": gyro_norm_max,
                "acceleration_abs_max_mps2": acceleration_abs_max,
                "gyro_abs_max_radps": gyro_abs_max,
                "acceleration_jump_max_mps2": acceleration_jump_max,
            },
        },
        "lidar_timing": {
            "validated": False,
            "scan_line": 18,
            "scan_line_source": "Unitree point_lio_unilidar L1 configuration",
            "scan_rate_hz": 16,
            "scan_rate_tolerance_hz": 1.0,
            "timestamp_unit": 0,
            "point_time_span_min_s": _floor(max(0.0, span_minimum - 0.005), 0.001),
            "point_time_span_max_s": _ceil(span_maximum + 0.005, 0.001),
            "lidar_imu_offset_jitter_max_s": _ceil(header_jitter + 0.003, 0.001),
        },
        "imu_stationary_thresholds": {
            "validated": False,
            "gravity_min_mps2": _floor(max(0.0, gravity_min - 0.5), 0.1),
            "gravity_max_mps2": _ceil(gravity_max + 0.5, 0.1),
            "stationary_gyro_max_radps": _ceil(gyro_norm_max * 2.0, 0.01),
            "acceleration_abs_max_mps2": _ceil(acceleration_abs_max * 1.5, 1.0),
            "gyro_abs_max_radps": _ceil(gyro_abs_max * 3.0, 0.01),
            "acceleration_jump_max_mps2": _ceil(acceleration_jump_max * 3.0, 0.1),
        },
    }


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--point-report", required=True, type=Path)
    parser.add_argument("--raw-report", required=True, type=Path, action="append")
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args(argv)

    result = derive_candidate_config(
        _load_json(arguments.point_report),
        [_load_json(path) for path in arguments.raw_report],
    )
    result["sources"] = [
        {"path": str(path), "sha256": _sha256(path)}
        for path in [arguments.point_report, *arguments.raw_report]
    ]
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        yaml.safe_dump(result, sort_keys=False), encoding="utf-8"
    )
    print(arguments.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
