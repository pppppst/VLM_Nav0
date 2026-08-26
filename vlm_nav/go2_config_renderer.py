"""Render a numeric SPARK config only from reviewed Go2 measurements."""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path

import yaml

from .go2_preflight import CalibrationError, TBD, require_calibrated


def _positive_integer(section, key):
    value = section.get(key)
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"lidar_timing.{key} must be a measured positive integer")
    return value


def render_fastlio_config(template, calibration, measurements):
    lidar_imu = calibration.get("lidar_imu_extrinsic", {})
    try:
        require_calibrated("lidar_imu_extrinsic", lidar_imu)
    except CalibrationError as error:
        raise ValueError(str(error)) from error
    expected_definition = "p_imu = R_lidar_to_imu * p_lidar + T_lidar_in_imu"
    if lidar_imu.get("definition") != expected_definition:
        raise ValueError("lidar_imu_extrinsic definition/direction is not explicit")

    timing = measurements.get("lidar_timing", {})
    if timing.get("validated") is not True:
        raise ValueError("lidar_timing is not validated")
    if timing.get("evidence") in (None, "", TBD):
        raise ValueError("lidar_timing has no evidence reference")
    scan_line = _positive_integer(timing, "scan_line")
    scan_rate = _positive_integer(timing, "scan_rate_hz")
    timestamp_unit = timing.get("timestamp_unit")
    if timestamp_unit not in (0, 1, 2, 3):
        raise ValueError("lidar_timing.timestamp_unit must be one of 0, 1, 2, 3")

    rendered = deepcopy(template)
    parameters = rendered["/**"]["ros__parameters"]
    parameters["mapping"]["extrinsic_T"] = list(lidar_imu["translation"])
    parameters["mapping"]["extrinsic_R"] = list(lidar_imu["rotation_matrix"])
    parameters["preprocess"]["scan_line"] = scan_line
    parameters["preprocess"]["scan_rate"] = scan_rate
    parameters["preprocess"]["timestamp_unit"] = timestamp_unit
    return rendered


def _load(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--template", required=True, type=Path)
    parser.add_argument("--calibration", required=True, type=Path)
    parser.add_argument("--measurements", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args()
    calibration = _load(arguments.calibration)
    measurements = _load(arguments.measurements)
    rendered = render_fastlio_config(
        _load(arguments.template), calibration, measurements
    )
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        yaml.safe_dump(rendered, sort_keys=False), encoding="utf-8"
    )
    provenance = {
        "lidar_imu_extrinsic": calibration["lidar_imu_extrinsic"],
        "lidar_timing": measurements["lidar_timing"],
        "generated_config": str(arguments.output),
    }
    provenance_path = arguments.output.with_suffix(
        arguments.output.suffix + ".provenance.yaml"
    )
    provenance_path.write_text(
        yaml.safe_dump(provenance, sort_keys=False), encoding="utf-8"
    )
    print(arguments.output)
    print(provenance_path)


if __name__ == "__main__":
    main()
