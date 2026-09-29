#!/usr/bin/env python3
"""Read-only 30 s timing diagnosis for the XT16 + Unitree IMU LIO chain."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import statistics
import time
from collections import Counter
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor, await_or_execute
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage


def stamp_ns(message) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def stats(values):
    if not values:
        return {"count": 0, "min": None, "median": None, "p95": None, "max": None}
    ordered = sorted(values)
    return {
        "count": len(values),
        "min": min(values),
        "median": statistics.median(values),
        "p95": ordered[math.ceil(0.95 * len(ordered)) - 1],
        "max": max(values),
    }


def pose_drift(poses):
    if len(poses) < 2:
        return {"translation_m": None, "yaw_deg": None}
    start, end = poses[0], poses[-1]
    yaw_delta = (end[3] - start[3] + math.pi) % (2 * math.pi) - math.pi
    return {
        "translation_m": math.dist(start[:3], end[:3]),
        "yaw_deg": abs(math.degrees(yaw_delta)),
    }


class Stream:
    def __init__(self):
        self.stamps = []
        self.receipts = []
        self.offsets = []
        self.received_offsets = []
        self.execute_offsets = []
        self.queue_delays = []

    def add(self, stamp: int, *, received_ns=None, execute_ns=None):
        execute_ns = time.time_ns() if execute_ns is None else execute_ns
        self.stamps.append(stamp)
        self.receipts.append(time.monotonic_ns())
        self.offsets.append(stamp / 1e9 - execute_ns / 1e9)
        self.received_offsets.append(
            None if not received_ns else (received_ns - stamp) / 1e9
        )
        self.execute_offsets.append((execute_ns - stamp) / 1e9)
        self.queue_delays.append(
            None if not received_ns else (execute_ns - received_ns) / 1e9
        )

    def report(self):
        deltas = [(b - a) / 1e9 for a, b in zip(self.stamps, self.stamps[1:])]
        elapsed = (self.receipts[-1] - self.receipts[0]) / 1e9 if len(self.receipts) > 1 else 0.0
        valid_received_offsets = [value for value in self.received_offsets if value is not None]
        valid_queue_delays = [value for value in self.queue_delays if value is not None]
        return {
            "samples": len(self.stamps),
            "hz": (len(self.stamps) - 1) / elapsed if elapsed > 0 else None,
            "stamp_rollback_count": sum(delta < 0 for delta in deltas),
            "duplicate_count": sum(delta == 0 for delta in deltas),
            "dt_sec": stats(deltas),
            "stamp_minus_system_now_sec": stats(self.offsets),
            "middleware_receive_minus_header_stamp_sec": stats(valid_received_offsets),
            "callback_execute_minus_header_stamp_sec": stats(self.execute_offsets),
            "callback_queue_delay_sec": stats(valid_queue_delays),
            "stamps_ns": self.stamps,
        }


class MessageInfoExecutor(SingleThreadedExecutor):
    """Pass Humble's otherwise-discarded RMW receive timestamp to callbacks."""

    def _take_subscription(self, subscription):
        with subscription.handle:
            return subscription.handle.take_message(subscription.msg_type, subscription.raw)

    async def _execute_subscription(self, subscription, taken):
        if taken:
            await await_or_execute(subscription.callback, *taken)


def nearest_relation(source, target):
    if not source or not target:
        return {"count": 0, "signed_sec": stats([]), "absolute_sec": stats([])}
    ordered = sorted(target)
    deltas = []
    for value in source:
        index = bisect.bisect_left(ordered, value)
        candidates = ordered[max(0, index - 1):min(len(ordered), index + 1)]
        nearest = min(candidates, key=lambda candidate: abs(value - candidate))
        deltas.append((value - nearest) / 1e9)
    return {
        "count": len(deltas),
        "signed_sec": stats(deltas),
        "absolute_sec": stats([abs(delta) for delta in deltas]),
    }


def stamp_match(raw, converted):
    raw_counts = Counter(raw)
    converted_counts = Counter(converted)
    matched = sum(min(count, converted_counts[stamp]) for stamp, count in raw_counts.items())
    return {
        "raw_samples": len(raw),
        "converted_samples": len(converted),
        "exactly_matched_converted_samples": matched,
        "converted_without_exact_raw_stamp": len(converted) - matched,
        "raw_without_matching_converted_stamp": len(raw) - matched,
        "all_converted_stamps_exactly_match_raw": bool(converted) and matched == len(converted),
    }


class Gate2Timing(Node):
    def __init__(self, *, odom_only=False, sensor_topic=None, sensor_reliability="reliable"):
        super().__init__("measure_gate2_timing")
        self.streams = {
            "/lidar_points": Stream(),
            "/lidar_points_fastlio": Stream(),
            "/utlidar/imu": Stream(),
            "/odometry": Stream(),
            "tf odom->base_link": Stream(),
        }
        self.odom_poses = []
        odom_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            depth=1,
            reliability=(ReliabilityPolicy.RELIABLE if sensor_reliability == "reliable" else ReliabilityPolicy.BEST_EFFORT),
            durability=DurabilityPolicy.VOLATILE,
        )
        tf_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        if sensor_topic:
            self.create_subscription(PointCloud2, sensor_topic, lambda msg, info: self.add(msg, info, sensor_topic), sensor_qos)
        elif not odom_only:
            self.create_subscription(PointCloud2, "/lidar_points", lambda msg, info: self.add(msg, info, "/lidar_points"), sensor_qos)
            self.create_subscription(PointCloud2, "/lidar_points_fastlio", lambda msg, info: self.add(msg, info, "/lidar_points_fastlio"), sensor_qos)
            self.create_subscription(Imu, "/utlidar/imu", lambda msg, info: self.add(msg, info, "/utlidar/imu"), sensor_qos)
        self.create_subscription(Odometry, "/odometry", self.on_odom, odom_qos)
        self.create_subscription(TFMessage, "/tf", self.on_tf, tf_qos)

    def add(self, message, info, name):
        self.streams[name].add(
            stamp_ns(message), received_ns=info.get("received_timestamp")
        )

    def on_tf(self, message, info):
        for transform in message.transforms:
            if transform.header.frame_id == "odom" and transform.child_frame_id == "base_link":
                self.streams["tf odom->base_link"].add(
                    stamp_ns(transform), received_ns=info.get("received_timestamp")
                )

    def on_odom(self, message, info):
        self.add(message, info, "/odometry")
        position = message.pose.pose.position
        q = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.odom_poses.append((position.x, position.y, position.z, yaw))

    def report(self):
        reports = {name: stream.report() for name, stream in self.streams.items()}
        raw = reports["/lidar_points"]["stamps_ns"]
        converted = reports["/lidar_points_fastlio"]["stamps_ns"]
        imu = reports["/utlidar/imu"]["stamps_ns"]
        odom = reports["/odometry"]["stamps_ns"]
        tf = reports["tf odom->base_link"]["stamps_ns"]
        for report in reports.values():
            report.pop("stamps_ns")
        odom_rollback = reports["/odometry"]["stamp_rollback_count"]
        tf_rollback = reports["tf odom->base_link"]["stamp_rollback_count"]
        raw_rollback = reports["/lidar_points"]["stamp_rollback_count"]
        converted_rollback = reports["/lidar_points_fastlio"]["stamp_rollback_count"]
        match = stamp_match(raw, converted)
        odom_tf = nearest_relation(odom, tf)
        if not raw and odom_rollback:
            diagnosis = "first_failure_spark_odometry_timestamp"
        elif not raw and tf_rollback:
            diagnosis = "first_failure_spark_odom_to_base_link_tf_timestamp"
        elif not raw and odom_tf["absolute_sec"]["max"] not in (None, 0.0):
            diagnosis = "odom_tf_timestamp_mismatch"
        elif not raw:
            diagnosis = "no_odometry_timestamp_failure_observed"
        elif raw_rollback:
            diagnosis = "first_failure_raw_lidar_driver"
        elif not match["all_converted_stamps_exactly_match_raw"]:
            diagnosis = "first_failure_converter_stamp_or_delivery"
        elif converted_rollback:
            diagnosis = "first_failure_converter_order"
        elif odom_rollback:
            diagnosis = "first_failure_spark_odometry_timestamp"
        elif tf_rollback:
            diagnosis = "first_failure_spark_odom_to_base_link_tf_timestamp"
        elif odom_tf["absolute_sec"]["max"] is not None and odom_tf["absolute_sec"]["max"] > 0.005:
            diagnosis = "odom_tf_timestamp_mismatch"
        else:
            diagnosis = "no_timestamp_failure_observed"
        return {
            "streams": reports,
            "raw_to_converted_stamp": match,
            "odometry_minus_nearest_lidar_stamp": nearest_relation(odom, raw),
            "odometry_minus_nearest_imu_stamp": nearest_relation(odom, imu),
            "odometry_minus_nearest_odom_tf_stamp": odom_tf,
            "odometry_drift": pose_drift(self.odom_poses),
            "diagnosis": diagnosis,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--odom-only", action="store_true")
    parser.add_argument("--sensor-topic", choices=("/lidar_points", "/lidar_points_fastlio"))
    parser.add_argument("--sensor-reliability", choices=("reliable", "best_effort"), default="reliable")
    args = parser.parse_args()
    rclpy.init()
    node = Gate2Timing(
        odom_only=args.odom_only,
        sensor_topic=args.sensor_topic,
        sensor_reliability=args.sensor_reliability,
    )
    executor = MessageInfoExecutor()
    executor.add_node(node)
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            executor.spin_once(timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        report = node.report()
        report["duration_sec"] = time.monotonic() - started
        if args.output:
            args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps({key: report[key] for key in (
            "duration_sec", "streams", "raw_to_converted_stamp",
            "odometry_minus_nearest_lidar_stamp", "odometry_minus_nearest_imu_stamp",
            "odometry_minus_nearest_odom_tf_stamp", "diagnosis",
            "odometry_drift",
        )}, indent=2))
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
