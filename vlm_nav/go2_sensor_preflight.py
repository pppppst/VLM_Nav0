"""Read-only Go2 LiDAR/IMU and multi-frame clock preflight."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from statistics import median
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
import yaml

from .go2_preflight import (
    ImuHealthAccumulator,
    RawImuAccumulator,
    TimestampAccumulator,
    analyze_cloud_points,
    nearest_stream_offsets,
)
from .go2_point_time import analyze_reconstructed_point_alignment


def _stamp_seconds(header) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1.0e-9


def _number(section, key):
    value = section.get(key)
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{key} is {value!r}; measurement is required")
    return float(value)


def load_validated_config(path):
    with open(path, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    timing = config["lidar_timing"]
    imu = config["imu_stationary_thresholds"]
    point_semantics = timing.get("point_time_semantics", {})
    if point_semantics.get("validated") is not True:
        raise ValueError(
            "lidar_timing.point_time_semantics is not validated; confirm header/point time semantics"
        )
    for key in ("header_stamp_reference", "point_time_reference", "unit"):
        value = str(point_semantics.get(key, "")).strip()
        if not value or value == "TBD / requires measurement":
            raise ValueError(
                f"lidar_timing.point_time_semantics.{key} requires measurement"
            )
    measurement_alignment = timing.get("lidar_imu_measurement_alignment", {})
    if measurement_alignment.get("validated") is not True:
        raise ValueError(
            "lidar_timing.lidar_imu_measurement_alignment is not validated"
        )
    for key in ("method", "evidence"):
        value = str(measurement_alignment.get(key, "")).strip()
        if not value or value == "TBD / requires measurement":
            raise ValueError(
                f"lidar_timing.lidar_imu_measurement_alignment.{key} requires measurement"
            )
    if timing.get("validated") is not True:
        raise ValueError("lidar_timing.validated is false; collect and review evidence first")
    if imu.get("validated") is not True:
        raise ValueError(
            "imu_stationary_thresholds.validated is false; derive thresholds from a stationary bag"
        )
    return config


def _cloud_rows(message, point_time_field="time"):
    requested_fields = ("x", "y", "z", point_time_field, "ring")
    data = point_cloud2.read_points(
        message, field_names=requested_fields, skip_nans=False
    )
    array = np.asarray(data)
    if array.dtype.names:
        return zip(*(array[name].tolist() for name in requested_fields))
    return (tuple(row) for row in array.tolist())


class Go2SensorPreflight(Node):
    def __init__(self, config, duration, *, diagnostic_only=False):
        super().__init__("go2_sensor_preflight")
        self.config = config
        self.duration = float(duration)
        self.diagnostic_only = bool(diagnostic_only)
        timing = config["lidar_timing"]
        imu = config["imu_stationary_thresholds"]
        self.lidar_topic = str(timing.get("topic", "/utlidar/cloud"))
        self.imu_topic = str(timing.get("imu_topic", "/utlidar/imu"))
        self.point_time_field = str(timing.get("point_time_field", "time"))
        self.point_time_is_absolute = bool(timing.get("point_time_is_absolute", False))
        self.raw_imu = RawImuAccumulator()
        self.imu_health = None
        if not self.diagnostic_only:
            self.imu_health = ImuHealthAccumulator(
                gravity_min=_number(imu, "gravity_min_mps2"),
                gravity_max=_number(imu, "gravity_max_mps2"),
                stationary_gyro_max=_number(imu, "stationary_gyro_max_radps"),
                acceleration_abs_max=_number(imu, "acceleration_abs_max_mps2"),
                gyro_abs_max=_number(imu, "gyro_abs_max_radps"),
                jump_max=_number(imu, "acceleration_jump_max_mps2"),
            )
            self.expected_scan_rate = _number(timing, "scan_rate_hz")
            self.scan_rate_tolerance = _number(timing, "scan_rate_tolerance_hz")
            self.point_time_span_min = _number(timing, "point_time_span_min_s")
            self.point_time_span_max = _number(timing, "point_time_span_max_s")
            self.lidar_imu_jitter_max = _number(
                timing, "lidar_imu_offset_jitter_max_s"
            )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.lidar_timestamps = TimestampAccumulator()
        self.imu_timestamps = TimestampAccumulator()
        self.cloud_reports = []
        self.cloud_header_stamps = []
        self.cloud_point_time_frames = []
        self.imu_header_stamps = []
        self.cloud_receipts = []
        self.cloud_errors = []
        self.started = time.monotonic()
        self.report = None
        self.done = False
        self.create_subscription(PointCloud2, self.lidar_topic, self.on_cloud, qos)
        self.create_subscription(Imu, self.imu_topic, self.on_imu, qos)
        self.timer = self.create_timer(0.1, self.check_complete)

    def on_cloud(self, message):
        receive_time = time.time()
        field_names = {field.name for field in message.fields}
        required = {"x", "y", "z", self.point_time_field, "ring"}
        missing = sorted(required - field_names)
        if missing:
            self.cloud_errors.append(f"missing PointCloud2 fields: {missing}")
            return
        try:
            rows = list(_cloud_rows(message, self.point_time_field))
            report = analyze_cloud_points(rows)
        except Exception as error:  # malformed driver data is a failed preflight
            self.cloud_errors.append(f"{type(error).__name__}: {error}")
            return
        stamp = _stamp_seconds(message.header)
        self.lidar_timestamps.add(
            header_stamp=stamp, local_receive_time=receive_time
        )
        self.cloud_reports.append(report)
        self.cloud_header_stamps.append(stamp)
        point_times = np.fromiter((float(row[3]) for row in rows), dtype=np.float64)
        if self.point_time_is_absolute:
            point_times = point_times - stamp
        self.cloud_point_time_frames.append((stamp, point_times))
        self.cloud_receipts.append(time.monotonic())

    def on_imu(self, message):
        receive_time = time.time()
        stamp = _stamp_seconds(message.header)
        self.imu_timestamps.add(header_stamp=stamp, local_receive_time=receive_time)
        self.imu_header_stamps.append(stamp)
        self.raw_imu.add(
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )
        if self.imu_health is not None:
            self.imu_health.add(
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            )

    def check_complete(self):
        if time.monotonic() - self.started < self.duration:
            return
        self.report = self.build_report()
        self.done = True
        self.timer.cancel()

    def build_report(self):
        if not self.cloud_reports or not self.imu_header_stamps:
            return {
                "healthy": False,
                "error": "no valid LiDAR or IMU samples",
                "cloud_errors": self.cloud_errors,
            }
        lidar_clock = self.lidar_timestamps.summary()
        imu_clock = self.imu_timestamps.summary()
        spans = [item["point_time_span_s"] for item in self.cloud_reports]
        if len(self.cloud_receipts) > 1:
            lidar_hz = (len(self.cloud_receipts) - 1) / (
                self.cloud_receipts[-1] - self.cloud_receipts[0]
            )
        else:
            lidar_hz = 0.0

        nearest_offsets = nearest_stream_offsets(
            self.cloud_header_stamps, self.imu_header_stamps
        )
        point_time_alignment = analyze_reconstructed_point_alignment(
            self.cloud_point_time_frames, self.imu_header_stamps
        )
        lidar_imu_offset_jitter_s = max(nearest_offsets) - min(nearest_offsets)

        ring_single_value_diagnostic = all(
            item["ring_single_value_diagnostic"] for item in self.cloud_reports
        )
        report = {
            "generated_at_unix_s": time.time(),
            "measurement_method": self.config["clock"]["measurement_method"],
            "lidar_clock": lidar_clock,
            "imu_clock": imu_clock,
            "lidar_hz": lidar_hz,
            "point_time_span_s": {
                "minimum": min(spans),
                "median": median(spans),
                "maximum": max(spans),
            },
            "lidar_imu_offset_s": {
                "minimum": min(nearest_offsets),
                "median": median(nearest_offsets),
                "maximum": max(nearest_offsets),
            },
            "lidar_imu_offset_jitter_s": lidar_imu_offset_jitter_s,
            "point_time_alignment": point_time_alignment,
            "imu_raw": self.raw_imu.summary(),
            "cloud_count": len(self.cloud_reports),
            "cloud_errors": self.cloud_errors,
            "ring_single_value_diagnostic": ring_single_value_diagnostic,
        }
        if self.diagnostic_only:
            report["formal_preflight"] = "NOT_EVALUATED: thresholds are unvalidated"
            report["healthy"] = None
            return report

        report["imu"] = self.imu_health.summary()
        report["formal_preflight"] = "PASS"
        minimum_samples = int(_number(self.config["clock"], "minimum_samples"))
        report["healthy"] = all(
            (
                lidar_clock["sample_count"] >= minimum_samples,
                imu_clock["sample_count"] >= minimum_samples,
                not self.cloud_errors,
                all(item["healthy"] for item in self.cloud_reports),
                abs(lidar_hz - self.expected_scan_rate) <= self.scan_rate_tolerance,
                min(spans) >= self.point_time_span_min,
                max(spans) <= self.point_time_span_max,
                lidar_clock["timestamp_backward_count"] == 0,
                imu_clock["timestamp_backward_count"] == 0,
                lidar_imu_offset_jitter_s <= self.lidar_imu_jitter_max,
                bool(report["imu"]["healthy"]),
            )
        )
        return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--diagnostic-only", action="store_true")
    arguments = parser.parse_args()
    if arguments.diagnostic_only:
        with open(arguments.config, encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
    else:
        config = load_validated_config(arguments.config)
    rclpy.init()
    node = Go2SensorPreflight(
        config, arguments.duration, diagnostic_only=arguments.diagnostic_only
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        report = node.report or {"healthy": False, "error": "preflight interrupted"}
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(encoded)
    if arguments.output:
        arguments.output.write_text(encoded + "\n", encoding="utf-8")
    if arguments.diagnostic_only:
        raise SystemExit(0 if report.get("cloud_count", 0) > 0 else 1)
    raise SystemExit(0 if report.get("healthy") else 1)


if __name__ == "__main__":
    main()
