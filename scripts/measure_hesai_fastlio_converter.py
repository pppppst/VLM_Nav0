#!/usr/bin/env python3
"""Measure the Hesai-to-FAST-LIO converter without starting downstream nodes."""

import argparse
import json
import math
import struct
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def field_map(message):
    return {field.name: field for field in message.fields}


def scalar(data, offset, fmt):
    return struct.unpack_from(fmt, data, offset)[0]


def summarize_output(message):
    fields = field_map(message)
    required = {"x", "y", "z", "intensity", "time", "ring"}
    if not required.issubset(fields):
        return {
            "point_count": int(message.width * message.height),
            "schema_ok": False,
            "finite_time": False,
            "rings": [],
            "time_span": None,
        }

    count = int(message.width * message.height)
    time_field = fields["time"]
    ring_field = fields["ring"]
    dtype = np.dtype({
        "names": ["time", "ring"],
        "formats": ["<f4", "<u2"],
        "offsets": [time_field.offset, ring_field.offset],
        "itemsize": message.point_step,
    })
    points = np.frombuffer(memoryview(message.data), dtype=dtype, count=count)
    finite = bool(np.isfinite(points["time"]).all())
    rings = set(int(value) for value in np.unique(points["ring"]))
    minimum = float(np.min(points["time"])) if finite and count else math.inf
    maximum = float(np.max(points["time"])) if finite and count else -math.inf
    return {
        "point_count": count,
        "schema_ok": [
            (field.name, int(field.datatype), int(field.offset), int(field.count))
            for field in message.fields
        ] == [
            ("x", 7, 0, 1),
            ("y", 7, 4, 1),
            ("z", 7, 8, 1),
            ("intensity", 7, 12, 1),
            ("time", 7, 16, 1),
            ("ring", 4, 20, 1),
        ],
        "finite_time": finite,
        "rings": sorted(rings),
        "time_span": None if not finite else maximum - minimum,
    }


class ConverterMeasurement(Node):
    def __init__(self, duration):
        super().__init__("measure_hesai_fastlio_converter")
        self.started = time.monotonic()
        self.duration = duration
        self.input_stamps = []
        self.input_stamp_set = set()
        self.output_stamps = []
        self.input_by_stamp = {}
        self.output_records = []
        self.output_callback_errors = []
        self.input_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, "/lidar_points", self.on_input, self.input_qos)
        self.create_subscription(
            PointCloud2, "/lidar_points_fastlio", self.on_output, self.output_qos)

    def on_input(self, message):
        stamp = stamp_ns(message)
        self.input_stamps.append(stamp)
        self.input_stamp_set.add(stamp)
        self.input_by_stamp[stamp] = time.monotonic()

    def on_output(self, message):
        stamp = stamp_ns(message)
        try:
            summary = summarize_output(message)
        except Exception as error:  # pragma: no cover - live message diagnostic
            self.output_callback_errors.append(repr(error))
            return
        summary["header_stamp_ns"] = stamp
        summary["frame_id"] = message.header.frame_id
        summary["receipt_monotonic"] = time.monotonic()
        input_receipt = self.input_by_stamp.get(stamp)
        summary["delivery_lag_sec"] = (
            None if input_receipt is None else summary["receipt_monotonic"] - input_receipt
        )
        self.output_records.append(summary)
        self.output_stamps.append(stamp)

    @staticmethod
    def sequence_stats(values):
        if len(values) < 2:
            return {"count": len(values), "rollback": 0, "dt_min": None, "dt_median": None, "dt_max": None}
        deltas = [(b - a) / 1e9 for a, b in zip(values, values[1:])]
        ordered = sorted(deltas)
        return {
            "count": len(values),
            "rollback": sum(delta < 0 for delta in deltas),
            "dt_min": min(deltas),
            "dt_median": ordered[len(ordered) // 2],
            "dt_max": max(deltas),
        }

    def report(self):
        spans = [record["time_span"] for record in self.output_records if record["time_span"] is not None]
        lags = [record["delivery_lag_sec"] for record in self.output_records if record["delivery_lag_sec"] is not None]
        counts = [record["point_count"] for record in self.output_records]
        ring_ok = [record["rings"] == list(range(16)) for record in self.output_records]
        finite_ok = [record["finite_time"] for record in self.output_records]
        first_input_stamp = min(self.input_stamps) if self.input_stamps else None
        comparable_records = [
            record for record in self.output_records
            if first_input_stamp is None or record["header_stamp_ns"] >= first_input_stamp
        ]
        header_matches = [
            record["header_stamp_ns"] in self.input_stamp_set
            for record in comparable_records
        ]
        return {
            "duration_sec": time.monotonic() - self.started,
            "input": self.sequence_stats(self.input_stamps),
            "output": self.sequence_stats(self.output_stamps),
            "output_point_count": {
                "count": len(counts),
                "min": min(counts) if counts else None,
                "median": sorted(counts)[len(counts) // 2] if counts else None,
                "max": max(counts) if counts else None,
            },
            "output_schema_ok": all(record["schema_ok"] for record in self.output_records),
            "output_rings_ok": all(ring_ok),
            "output_finite_time_ok": all(finite_ok),
            "output_time_span_sec": {
                "min": min(spans) if spans else None,
                "median": sorted(spans)[len(spans) // 2] if spans else None,
                "max": max(spans) if spans else None,
            },
            "header_stamp_matches_input": bool(comparable_records) and all(header_matches),
            "output_callback_errors": self.output_callback_errors,
            "delivery_lag_sec": {
                "count": len(lags),
                "min": min(lags) if lags else None,
                "median": sorted(lags)[len(lags) // 2] if lags else None,
                "max": max(lags) if lags else None,
            },
        }


def check_report(report, min_points=60000, min_span=0.05, max_span=0.15):
    output = report["output"]
    point_counts = report["output_point_count"]
    span = report["output_time_span_sec"]
    checks = {
        "output_nonempty": output["count"] > 0,
        "output_rollback_zero": output["rollback"] == 0,
        "point_count_minimum": point_counts["min"] is not None and point_counts["min"] >= min_points,
        "schema_ok": report["output_schema_ok"],
        "rings_ok": report["output_rings_ok"],
        "finite_time_ok": report["output_finite_time_ok"],
        "time_span_ok": (
            span["min"] is not None and span["min"] >= min_span and
            span["max"] is not None and span["max"] <= max_span
        ),
        "header_stamp_matches_input": report["header_stamp_matches_input"],
    }
    return checks, all(checks.values())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check", type=Path)
    parser.add_argument("--min-points", type=int, default=60000)
    args = parser.parse_args()

    if args.check:
        report = json.loads(args.check.read_text())
        checks, passed = check_report(report, args.min_points)
        print(json.dumps({"checks": checks, "passed": passed}, indent=2))
        raise SystemExit(0 if passed else 1)

    rclpy.init()
    node = ConverterMeasurement(args.duration)
    try:
        while rclpy.ok() and time.monotonic() - node.started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        report = node.report()
        if args.output:
            args.output.write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
