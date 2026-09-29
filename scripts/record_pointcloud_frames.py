#!/usr/bin/env python3
"""Record PointCloud2 frame metadata and correlate converter drops by stamp."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


DTYPES = {
    PointField.FLOAT32: np.dtype("<f4"),
    PointField.FLOAT64: np.dtype("<f8"),
    PointField.UINT16: np.dtype("<u2"),
}


def stamp_ns(message: PointCloud2) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def field_array(message: PointCloud2, field: PointField) -> np.ndarray:
    dtype = DTYPES[field.datatype]
    if message.is_bigendian:
        dtype = dtype.newbyteorder(">")
    return np.ndarray(
        shape=(int(message.height), int(message.width)),
        dtype=dtype,
        buffer=memoryview(message.data),
        offset=int(field.offset),
        strides=(int(message.row_step), int(message.point_step)),
    )


def summarize_cloud(message: PointCloud2, time_field_name: str) -> dict[str, object]:
    fields = {field.name: field for field in message.fields}
    count = int(message.width) * int(message.height)
    result: dict[str, object] = {
        "header_stamp_ns": stamp_ns(message),
        "header_stamp_sec": int(message.header.stamp.sec),
        "header_stamp_nanosec": int(message.header.stamp.nanosec),
        "frame_id": message.header.frame_id,
        "width": int(message.width),
        "height": int(message.height),
        "points_per_frame": count,
        "ring": {"min": None, "max": None, "unique_count": None, "counts": None},
        "time": {"min": None, "max": None, "span": None},
        "nan_count": None,
        "inf_count": None,
        "schema_ok": True,
        "error": None,
    }

    time_datatype = PointField.FLOAT64 if time_field_name == "timestamp" else PointField.FLOAT32
    required = {
        "x": PointField.FLOAT32,
        "y": PointField.FLOAT32,
        "z": PointField.FLOAT32,
        "intensity": PointField.FLOAT32,
        "ring": PointField.UINT16,
        time_field_name: time_datatype,
    }
    arrays: dict[str, np.ndarray] = {}
    for name, datatype in required.items():
        field = fields.get(name)
        if field is None or field.datatype != datatype or field.count != 1:
            result["schema_ok"] = False
            continue
        if int(field.offset) + DTYPES[datatype].itemsize > int(message.point_step):
            result["schema_ok"] = False
            continue
        try:
            arrays[name] = field_array(message, field).reshape(-1)
        except (ValueError, TypeError) as error:
            result["schema_ok"] = False
            result["error"] = f"{type(error).__name__}: {error}"

    if not result["schema_ok"]:
        return result

    numeric = [arrays[name] for name in ("x", "y", "z", "intensity", time_field_name)]
    result["nan_count"] = int(sum(np.isnan(values).sum() for values in numeric))
    result["inf_count"] = int(sum(np.isinf(values).sum() for values in numeric))

    rings = arrays["ring"]
    ring_values, ring_counts = np.unique(rings, return_counts=True)
    result["ring"] = {
        "min": int(rings.min()) if count else None,
        "max": int(rings.max()) if count else None,
        "unique_count": int(ring_values.size),
        "counts": {str(int(value)): int(size) for value, size in zip(ring_values, ring_counts)},
    }
    times = arrays[time_field_name]
    result["time"] = {
        "min": float(times.min()) if count else None,
        "max": float(times.max()) if count else None,
        "span": float(times.max() - times.min()) if count else None,
    }
    return result


def infer_converter_reason(
    record: dict[str, object], *, min_points: int, scan_lines: int,
    min_points_per_ring: int, min_span: float, max_span: float,
) -> str | None:
    if int(record["points_per_frame"]) < min_points:
        return "incomplete_too_few_points"
    if not record["schema_ok"]:
        return "malformed_or_missing_field"
    if int(record["nan_count"]) or int(record["inf_count"]):
        return "nonfinite"
    ring = record["ring"]
    counts = ring["counts"] or {}
    if (
        int(ring["min"]) < 0 or int(ring["max"]) >= scan_lines or
        int(ring["unique_count"]) != scan_lines or
        any(int(counts.get(str(index), 0)) < min_points_per_ring for index in range(scan_lines))
    ):
        return "ring_distribution"
    span = record["time"]["span"]
    if span is None or not min_span <= float(span) <= max_span:
        return "timespan"
    return None


class PointCloudFrameRecorder(Node):
    def __init__(self, input_topic: str, output_topic: str) -> None:
        super().__init__("record_pointcloud_frames")
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.input_frames: list[dict[str, object]] = []
        self.output_frames: list[dict[str, object]] = []
        self.output_stamps: set[int] = set()
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, input_topic, self.on_input, qos)
        self.create_subscription(PointCloud2, output_topic, self.on_output, qos)

    def on_input(self, message: PointCloud2) -> None:
        record = summarize_cloud(message, "timestamp")
        self.input_frames.append(record)
        print(json.dumps({"input": record}, ensure_ascii=False), flush=True)

    def on_output(self, message: PointCloud2) -> None:
        record = summarize_cloud(message, "time")
        self.output_frames.append(record)
        self.output_stamps.add(int(record["header_stamp_ns"]))
        print(json.dumps({"output": record}, ensure_ascii=False), flush=True)

    def report(self, min_points: int, scan_lines: int, min_points_per_ring: int,
               min_span: float, max_span: float) -> dict[str, object]:
        frames = []
        for record in self.input_frames:
            stamp = int(record["header_stamp_ns"])
            output = stamp in self.output_stamps
            item = dict(record)
            item["converter_output_match"] = output
            if output:
                item["converter_reason"] = None
            else:
                item["converter_reason"] = infer_converter_reason(
                    record,
                    min_points=min_points,
                    scan_lines=scan_lines,
                    min_points_per_ring=min_points_per_ring,
                    min_span=min_span,
                    max_span=max_span,
                ) or "not_explained_by_input_gate"
            frames.append(item)
        return {
            "input_topic": self.input_topic,
            "output_topic": self.output_topic,
            "input_frame_count": len(self.input_frames),
            "output_frame_count": len(self.output_frames),
            "matched_frame_count": sum(frame["converter_output_match"] for frame in frames),
            "frames": frames,
            "output_frames": self.output_frames,
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-topic", default="/lidar_points")
    parser.add_argument("--output-topic", default="/lidar_points_fastlio")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--min-points", type=int, default=60000)
    parser.add_argument("--scan-lines", type=int, default=16)
    parser.add_argument("--min-points-per-ring", type=int, default=3000)
    parser.add_argument("--min-span", type=float, default=0.05)
    parser.add_argument("--max-span", type=float, default=0.15)
    args = parser.parse_args()

    rclpy.init()
    node = PointCloudFrameRecorder(args.input_topic, args.output_topic)
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        report = node.report(
            args.min_points, args.scan_lines, args.min_points_per_ring,
            args.min_span, args.max_span,
        )
        if args.output:
            args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
        print(json.dumps({"summary": {key: report[key] for key in (
            "input_frame_count", "output_frame_count", "matched_frame_count")}}, ensure_ascii=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
