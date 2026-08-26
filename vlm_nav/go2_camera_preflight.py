"""Read-only RGB/aligned-depth/CameraInfo preflight for the Go2 D435."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from .go2_preflight import CameraHealthAccumulator


def _stamp_seconds(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


class Go2CameraPreflight(Node):
    def __init__(self, *, duration: float, minimum_samples: int, sync_slop_s: float):
        super().__init__("go2_camera_preflight")
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/color/camera_info"
        )
        self.declare_parameter("clock_threshold_s", 0.05)
        self.duration = float(duration)
        self.started = time.monotonic()
        self.done = False
        self.report = None
        self.health = CameraHealthAccumulator(
            minimum_samples=minimum_samples,
            sync_slop_s=sync_slop_s,
            clock_threshold_s=float(self.get_parameter("clock_threshold_s").value),
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        info_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Image, str(self.get_parameter("rgb_topic").value), self._on_rgb, image_qos
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._on_depth,
            image_qos,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info,
            info_qos,
        )
        self.timer = self.create_timer(0.1, self._check_complete)

    def _image_values(self, message: Image):
        return {
            "stamp": _stamp_seconds(message),
            "local_receive_time": time.time(),
            "width": int(message.width),
            "height": int(message.height),
            "encoding": str(message.encoding),
            "step": int(message.step),
            "data_size": len(message.data),
        }

    def _on_rgb(self, message: Image) -> None:
        self.health.add_rgb(**self._image_values(message))

    def _on_depth(self, message: Image) -> None:
        self.health.add_depth(**self._image_values(message))

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.health.add_camera_info(
            stamp=_stamp_seconds(message),
            local_receive_time=time.time(),
            width=int(message.width),
            height=int(message.height),
            frame_id=str(message.header.frame_id),
            k=list(message.k),
        )

    def _check_complete(self) -> None:
        if time.monotonic() - self.started < self.duration:
            return
        self.report = self.health.summary()
        self.done = True
        self.timer.cancel()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--minimum-samples", type=int, default=3)
    parser.add_argument("--sync-slop", type=float, default=0.05)
    parser.add_argument("--output", type=Path)
    arguments, ros_arguments = parser.parse_known_args()
    rclpy.init(args=ros_arguments)
    node = Go2CameraPreflight(
        duration=arguments.duration,
        minimum_samples=arguments.minimum_samples,
        sync_slop_s=arguments.sync_slop,
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
    raise SystemExit(0 if report.get("healthy") else 1)


if __name__ == "__main__":
    main()
