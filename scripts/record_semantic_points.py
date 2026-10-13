#!/usr/bin/env python3
"""Record raw RViz PointStamped clicks; never transform or discard a point."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import uuid


class PointRecorder:
    def __init__(self, output_dir, frame_id):
        if frame_id != "map":
            raise ValueError("本次标注只接受 map frame，不进行坐标转换")
        self.frame_id = frame_id
        self.session = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ") + "_" + uuid.uuid4().hex[:8]
        directory = Path(output_dir).resolve()
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / f"clicked_points_{self.session}.jsonl"
        self.file = self.path.open("x", encoding="utf-8")
        self.count = 0
        self.valid_count = 0

    def record(self, message):
        coordinates = {axis: getattr(message.point, axis) for axis in ("x", "y", "z")}
        reasons = []
        if message.header.frame_id != self.frame_id:
            reasons.append("frame_mismatch")
        if not all(math.isfinite(value) for value in coordinates.values()):
            reasons.append("non_finite_coordinates")
        row = dict(
            session=self.session, index=self.count + 1,
            stamp=dict(sec=message.header.stamp.sec, nanosec=message.header.stamp.nanosec),
            frame_id=message.header.frame_id,
            # Non-finite values are strings so the raw record remains standard JSON.
            point={key: value if math.isfinite(value) else repr(value) for key, value in coordinates.items()},
            valid=not reasons, invalid_reasons=reasons,
        )
        self.file.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
        self.file.flush()
        self.count += 1
        self.valid_count += int(row["valid"])
        print(f"#{self.count} {coordinates} frame_id={row['frame_id']!r} "
              f"{'有效' if row['valid'] else '无效: ' + ','.join(reasons)}", flush=True)
        return row

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.file.close()
        print(f"已保存 {self.count} 点（有效 {self.valid_count}），文件: {self.path}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/clicked_point")
    parser.add_argument("--frame-id", default="map", choices=("map",))
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    import rclpy
    from geometry_msgs.msg import PointStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=[])
    node = None
    try:
        with PointRecorder(args.output_dir, args.frame_id) as recorder:
            node = rclpy.create_node("semantic_point_recorder_" + uuid.uuid4().hex[:8])
            node.create_subscription(PointStamped, args.topic, recorder.record, QoSProfile(
                depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ))
            print(f"录点已就绪 topic={args.topic} frame_id={args.frame_id}\n"
                  f"记录文件: {recorder.path}\n可以开始依次点击四个角；Ctrl+C 停止。", flush=True)
            try:
                rclpy.spin(node)
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
