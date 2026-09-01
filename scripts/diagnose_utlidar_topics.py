#!/usr/bin/env python3
"""Read-only semantic diagnostic for the three Unitree Go2 point-cloud topics."""

import argparse
import json
import math
import statistics
import struct
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


TOPICS = ["/utlidar/cloud", "/utlidar/cloud_base", "/utlidar/cloud_deskewed"]
DATATYPE_NAMES = {
    1: "INT8",
    2: "UINT8",
    3: "INT16",
    4: "UINT16",
    5: "INT32",
    6: "UINT32",
    7: "FLOAT32",
    8: "FLOAT64",
}
DATATYPE_FORMATS = {
    1: ("b", 1),
    2: ("B", 1),
    3: ("h", 2),
    4: ("H", 2),
    5: ("i", 4),
    6: ("I", 4),
    7: ("f", 4),
    8: ("d", 8),
}


def finite(value: float) -> bool:
    return math.isfinite(value)


def scalar_stats(values: Sequence[float]) -> Dict[str, Any]:
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "min": min(values),
        "median": statistics.median(values),
        "max": max(values),
    }


def stamp_ns(msg: PointCloud2) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def dt_stats(stamps_ns: Sequence[int]) -> Dict[str, Any]:
    dts = [b - a for a, b in zip(stamps_ns, stamps_ns[1:]) if b >= a]
    out = scalar_stats([dt / 1e9 for dt in dts])
    out["rollback_count"] = sum(b < a for a, b in zip(stamps_ns, stamps_ns[1:]))
    out["duplicate_count"] = sum(b == a for a, b in zip(stamps_ns, stamps_ns[1:]))
    return out


def field_map(msg: PointCloud2) -> Dict[str, Any]:
    return {
        f.name: {
            "offset": int(f.offset),
            "datatype": int(f.datatype),
            "datatype_name": DATATYPE_NAMES.get(int(f.datatype), "UNKNOWN"),
            "count": int(f.count),
        }
        for f in msg.fields
    }


def read_field(msg: PointCloud2, field: Dict[str, Any], index: int) -> float:
    fmt, size = DATATYPE_FORMATS[field["datatype"]]
    offset = index * int(msg.point_step) + int(field["offset"])
    endian = ">" if msg.is_bigendian else "<"
    return float(struct.unpack_from(endian + fmt, msg.data, offset)[0])


def time_unit_candidate(values: Sequence[float]) -> str:
    finite_values = [abs(v) for v in values if finite(v)]
    if not finite_values:
        return "unknown"
    vmax = max(finite_values)
    if vmax < 1.0:
        return "seconds-like (candidate only)"
    if vmax < 1_000.0:
        return "milliseconds-like (candidate only)"
    if vmax < 1_000_000.0:
        return "microseconds-like (candidate only)"
    return "nanoseconds-like or larger (candidate only)"


def analyze_frame(msg: PointCloud2) -> Dict[str, Any]:
    fields = field_map(msg)
    n = int(msg.width) * int(msg.height)
    xyz_fields = [fields.get(name) for name in ("x", "y", "z")]
    time_field = fields.get("time")
    ring_field = fields.get("ring")

    xyz = []
    time_values = []
    ring_values = []
    nan_xyz = 0
    inf_xyz = 0
    xyz_finite_values = [[], [], []]

    for i in range(n):
        point = []
        for field in xyz_fields:
            point.append(read_field(msg, field, i) if field else float("nan"))
        xyz.append(point)
        if any(math.isnan(v) for v in point):
            nan_xyz += 1
        if any(math.isinf(v) for v in point):
            inf_xyz += 1
        for axis, value in enumerate(point):
            if finite(value):
                xyz_finite_values[axis].append(value)
        if time_field:
            time_values.append(read_field(msg, time_field, i))
        if ring_field:
            ring_values.append(int(read_field(msg, ring_field, i)))

    time_finite = [v for v in time_values if finite(v)]
    time_deltas = [b - a for a, b in zip(time_values, time_values[1:])]
    monotonic_fraction = (
        sum(delta >= 0 for delta in time_deltas) / len(time_deltas) if time_deltas else None
    )
    time_summary = None
    if time_finite:
        tmin = min(time_finite)
        tmax = max(time_finite)
        trange = tmax - tmin
        time_summary = {
            "datatype": time_field["datatype_name"],
            "min": tmin,
            "max": tmax,
            "range": trange,
            "starts_near_zero": abs(tmin) <= max(1e-6, abs(trange) * 0.05),
            "monotonic_fraction": monotonic_fraction,
            "basically_monotonic": monotonic_fraction is not None and monotonic_fraction >= 0.99,
            "finite_count": len(time_finite),
            "nonfinite_count": len(time_values) - len(time_finite),
        }

    return {
        "stamp_ns": stamp_ns(msg),
        "frame_id": msg.header.frame_id,
        "point_count": n,
        "fields": fields,
        "time_values": time_values,
        "time_summary": time_summary,
        "ring_values": ring_values,
        "xyz": xyz,
        "nan_xyz_point_count": nan_xyz,
        "inf_xyz_point_count": inf_xyz,
        "xyz_range": {
            axis: scalar_stats(values)
            for axis, values in zip(("x", "y", "z"), xyz_finite_values)
        },
    }


def nearest_frame_pairs(left: List[Dict[str, Any]], right: List[Dict[str, Any]]) -> List[Tuple[Dict[str, Any], Dict[str, Any]]]:
    pairs = []
    used = set()
    for lframe in left:
        candidates = sorted(
            ((abs(lframe["stamp_ns"] - rframe["stamp_ns"]), j, rframe) for j, rframe in enumerate(right) if j not in used),
            key=lambda item: item[0],
        )
        if candidates:
            delta, j, rframe = candidates[0]
            if delta <= 20_000_000:
                used.add(j)
                pairs.append((lframe, rframe))
    return pairs


def rigid_fit(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> Optional[Dict[str, float]]:
    try:
        import numpy as np
    except ImportError:
        return None
    if len(left) != len(right) or len(left) < 3:
        return None
    a = np.asarray(left, dtype=float)
    b = np.asarray(right, dtype=float)
    good = np.isfinite(a).all(axis=1) & np.isfinite(b).all(axis=1)
    a = a[good]
    b = b[good]
    if len(a) < 3:
        return None
    ac = a.mean(axis=0)
    bc = b.mean(axis=0)
    h = (a - ac).T @ (b - bc)
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = vt.T @ u.T
    t = bc - r @ ac
    residual = (a @ r.T + t) - b
    norms = np.linalg.norm(residual, axis=1)
    return {
        "matched_finite_points": int(len(a)),
        "rms_m": float(np.sqrt(np.mean(norms * norms))),
        "max_m": float(np.max(norms)),
        "translation_m": [float(v) for v in t],
        "rotation_det": float(np.linalg.det(r)),
    }


def compare_topics(left: List[Dict[str, Any]], right: List[Dict[str, Any]]) -> Dict[str, Any]:
    pairs = nearest_frame_pairs(left, right)
    reports = []
    for lframe, rframe in pairs:
        point_delta = abs(lframe["point_count"] - rframe["point_count"])
        same_count = lframe["point_count"] == rframe["point_count"]
        time_delta = None
        if lframe["time_values"] and rframe["time_values"] and same_count:
            time_delta = max(
                abs(a - b) for a, b in zip(lframe["time_values"], rframe["time_values"])
            )
        fit = rigid_fit(lframe["xyz"], rframe["xyz"]) if same_count else None
        reports.append(
            {
                "header_dt_ms": abs(lframe["stamp_ns"] - rframe["stamp_ns"]) / 1e6,
                "left_points": lframe["point_count"],
                "right_points": rframe["point_count"],
                "point_count_delta": point_delta,
                "same_point_count": same_count,
                "max_abs_time_delta": time_delta,
                "rigid_fit": fit,
            }
        )
    same_count_values = [r["same_point_count"] for r in reports]
    fits = [r["rigid_fit"] for r in reports if r["rigid_fit"]]
    return {
        "pair_count": len(reports),
        "same_point_count_count": sum(same_count_values),
        "same_point_count_fraction": (
            sum(same_count_values) / len(same_count_values) if same_count_values else None
        ),
        "time_max_abs_delta_stats": scalar_stats(
            [r["max_abs_time_delta"] for r in reports if r["max_abs_time_delta"] is not None]
        ),
        "rigid_rms_stats_m": scalar_stats([r["rms_m"] for r in fits]),
        "rigid_max_stats_m": scalar_stats([r["max_m"] for r in fits]),
        "pairs": reports,
    }


class TopicDiagnostic(Node):
    def __init__(self, topics: Sequence[str], target_frames: int):
        super().__init__("utlidar_topic_semantics_diagnostic")
        self.topics = list(topics)
        self.target_frames = target_frames
        self.frames = {topic: [] for topic in topics}
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self._subscriptions = [
            self.create_subscription(PointCloud2, topic, lambda msg, t=topic: self.callback(msg, t), qos)
            for topic in topics
        ]

    def callback(self, msg: PointCloud2, topic: str) -> None:
        if int(msg.width) * int(msg.height) <= 0:
            return
        if len(self.frames[topic]) < self.target_frames:
            self.frames[topic].append(analyze_frame(msg))

    def complete(self) -> bool:
        return all(len(self.frames[topic]) >= self.target_frames for topic in self.topics)

    def publisher_info(self) -> Dict[str, Any]:
        output = {}
        for topic in self.topics:
            infos = self.get_publishers_info_by_topic(topic)
            output[topic] = [
                {
                    "node_name": info.node_name,
                    "node_namespace": info.node_namespace,
                    "topic_type": info.topic_type,
                    "endpoint_type": str(info.endpoint_type),
                    "qos": {
                        "reliability": str(info.qos_profile.reliability),
                        "durability": str(info.qos_profile.durability),
                        "history": str(info.qos_profile.history),
                        "depth": int(info.qos_profile.depth),
                    },
                }
                for info in infos
            ]
        return output


def summarize_topic(frames: List[Dict[str, Any]], publishers: List[Dict[str, Any]]) -> Dict[str, Any]:
    stamps = [f["stamp_ns"] for f in frames]
    point_counts = [f["point_count"] for f in frames]
    frame_ids = sorted(set(f["frame_id"] for f in frames))
    all_time_values = [v for f in frames for v in f["time_values"] if finite(v)]
    all_ring_values = sorted(set(v for f in frames for v in f["ring_values"]))
    ranges = {}
    for axis in ("x", "y", "z"):
        values = [f["xyz_range"][axis] for f in frames if f["xyz_range"][axis].get("count", 0)]
        ranges[axis] = {
            "min": min(v["min"] for v in values) if values else None,
            "max": max(v["max"] for v in values) if values else None,
        }
    return {
        "publisher": publishers,
        "message_type": "sensor_msgs/msg/PointCloud2",
        "frame_ids": frame_ids,
        "header_timestamps_ns": stamps,
        "header_dt": dt_stats(stamps),
        "points_per_frame": scalar_stats(point_counts),
        "fields": frames[0]["fields"],
        "time": {
            "datatype": frames[0]["time_summary"]["datatype"] if frames[0]["time_summary"] else None,
            "candidate_unit": time_unit_candidate(all_time_values),
            "frame_stats": [f["time_summary"] for f in frames],
            "global": scalar_stats(all_time_values),
        },
        "ring": {
            "datatype": frames[0]["fields"].get("ring", {}).get("datatype_name"),
            "min": min(all_ring_values) if all_ring_values else None,
            "max": max(all_ring_values) if all_ring_values else None,
            "unique_values": all_ring_values,
            "unique_count": len(all_ring_values),
        },
        "xyz_range": ranges,
        "nan_xyz_point_count": sum(f["nan_xyz_point_count"] for f in frames),
        "inf_xyz_point_count": sum(f["inf_xyz_point_count"] for f in frames),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--output", default="/tmp/utlidar_topic_semantics.json")
    args = parser.parse_args()

    rclpy.init()
    node = TopicDiagnostic(TOPICS, args.frames)
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and not node.complete() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    publishers = node.publisher_info()
    result: Dict[str, Any] = {
        "requested_nonempty_frames": args.frames,
        "collected_frames": {topic: len(node.frames[topic]) for topic in TOPICS},
        "topics": {},
        "comparisons": {},
    }
    for topic in TOPICS:
        result["topics"][topic] = summarize_topic(node.frames[topic], publishers[topic]) if node.frames[topic] else {
            "publisher": publishers[topic],
            "message_type": "sensor_msgs/msg/PointCloud2",
        }
    if node.frames[TOPICS[0]] and node.frames[TOPICS[1]]:
        result["comparisons"]["cloud_vs_cloud_base"] = compare_topics(
            node.frames[TOPICS[0]], node.frames[TOPICS[1]]
        )
    if node.frames[TOPICS[0]] and node.frames[TOPICS[2]]:
        result["comparisons"]["cloud_vs_cloud_deskewed"] = compare_topics(
            node.frames[TOPICS[0]], node.frames[TOPICS[2]]
        )
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2, allow_nan=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if node.complete() else 2


if __name__ == "__main__":
    raise SystemExit(main())
