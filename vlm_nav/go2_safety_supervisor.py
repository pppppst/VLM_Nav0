"""Continuous fail-closed owner of the Go2 SYSTEM_READY signal.

The one-shot formal sensor report gates process startup.  This node then owns
the live readiness Bool; no other component should publish that topic.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from .go2_preflight import FormalPreflightError, validate_formal_preflight_report


def _stamp_seconds(message) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


class StreamHealth:
    def __init__(self) -> None:
        self.last_receipt = None
        self.last_stamp = None
        self.header_to_wall_offset_s = None
        self.data_ok = False
        self.backward = False

    def observe(self, stamp: float, *, data_ok: bool) -> None:
        """Record receipt freshness, monotonicity and data validity."""

        now_wall = time.time()
        now_steady = time.monotonic()
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.backward = True
        self.last_stamp = stamp
        self.last_receipt = now_steady
        self.header_to_wall_offset_s = stamp - now_wall
        self.data_ok = bool(data_ok)

    def healthy(self, now_steady: float, maximum_age: float) -> bool:
        return bool(
            self.last_receipt is not None
            and now_steady - self.last_receipt <= maximum_age
            and self.last_stamp is not None
            and math.isfinite(self.last_stamp)
            and self.data_ok
            and not self.backward
        )


class Go2SafetySupervisor(Node):
    def __init__(self) -> None:
        super().__init__("go2_safety_supervisor")
        report_path = Path(self.declare_parameter("formal_report_path", "").value)
        report_max_age = float(self.declare_parameter("formal_report_max_age_s", 300.0).value)
        self.raw_max_age = float(self.declare_parameter("raw_max_age_s", 0.5).value)
        self.lio_max_age = float(self.declare_parameter("lio_max_age_s", 1.0).value)
        self.extreme_acceleration = float(
            self.declare_parameter("acceleration_abs_max_mps2", 0.0).value
        )
        self.extreme_gyro = float(self.declare_parameter("gyro_abs_max_radps", 0.0).value)

        if min(
            self.raw_max_age,
            self.lio_max_age,
            self.extreme_acceleration,
            self.extreme_gyro,
        ) <= 0.0:
            raise ValueError("live safety thresholds must be verified positive numbers")

        if not report_path.is_file():
            raise FormalPreflightError(f"formal preflight report does not exist: {report_path}")
        report = json.loads(report_path.read_text(encoding="utf-8"))
        validate_formal_preflight_report(
            report, now_unix_s=time.time(), max_age_s=report_max_age
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.ready_pub = self.create_publisher(Bool, "/vlm_nav/system_ready", latched_qos)
        self.lidar = StreamHealth()
        self.imu = StreamHealth()
        self.odometry_receipt = None
        self.registered_cloud_receipt = None
        self.bridge_normal = False
        self.sport_interface_ready = False
        self.bridge_receipt = None
        self.sport_interface_receipt = None
        self.last_ready = None

        self.create_subscription(PointCloud2, "/utlidar/cloud", self._on_lidar, sensor_qos)
        self.create_subscription(Imu, "/utlidar/imu", self._on_imu, sensor_qos)
        self.create_subscription(Odometry, "/odometry", self._on_odometry, sensor_qos)
        self.create_subscription(
            PointCloud2, "/cloud_registered_base", self._on_registered_cloud, sensor_qos
        )
        self.create_subscription(
            String, "/vlm_nav/go2_bridge_state", self._on_bridge_state, latched_qos
        )
        self.create_subscription(
            Bool,
            "/vlm_nav/go2_sport_interface_ready",
            self._on_sport_interface_ready,
            latched_qos,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self._tick)
        self._publish(False)

    def _on_lidar(self, message: PointCloud2) -> None:
        fields = {field.name for field in message.fields}
        self.lidar.observe(
            _stamp_seconds(message),
            data_ok={"x", "y", "z", "time"}.issubset(fields) and message.width > 0,
        )

    def _on_imu(self, message: Imu) -> None:
        values = (
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )
        finite = all(math.isfinite(value) for value in values)
        bounded = finite and all(
            abs(value) <= self.extreme_acceleration for value in values[:3]
        ) and all(abs(value) <= self.extreme_gyro for value in values[3:])
        self.imu.observe(_stamp_seconds(message), data_ok=bounded)

    def _on_odometry(self, _message: Odometry) -> None:
        self.odometry_receipt = time.monotonic()

    def _on_registered_cloud(self, _message: PointCloud2) -> None:
        self.registered_cloud_receipt = time.monotonic()

    def _on_bridge_state(self, message: String) -> None:
        state = message.data.split(":", 1)[0].strip()
        self.bridge_normal = state in {"DISARMED", "ARMED"}
        self.bridge_receipt = time.monotonic()

    def _on_sport_interface_ready(self, message: Bool) -> None:
        self.sport_interface_ready = bool(message.data)
        self.sport_interface_receipt = time.monotonic()

    def _has_tf(self, parent: str, child: str) -> bool:
        return self.tf_buffer.can_transform(
            parent, child, rclpy.time.Time(), Duration(seconds=0.0)
        )

    def _tick(self) -> None:
        now = time.monotonic()
        lio_fresh = bool(
            self.odometry_receipt is not None
            and self.registered_cloud_receipt is not None
            and now - self.odometry_receipt <= self.lio_max_age
            and now - self.registered_cloud_receipt <= self.lio_max_age
        )
        ready = all(
            (
                self.lidar.healthy(now, self.raw_max_age),
                self.imu.healthy(now, self.raw_max_age),
                lio_fresh,
                self._has_tf("base_link", "utlidar_lidar"),
                self._has_tf("odom", "base_link"),
                self.bridge_normal,
                self.sport_interface_ready,
                self.bridge_receipt is not None
                and now - self.bridge_receipt <= self.lio_max_age,
                self.sport_interface_receipt is not None
                and now - self.sport_interface_receipt <= self.lio_max_age,
            )
        )
        self._publish(ready)

    def _publish(self, ready: bool) -> None:
        message = Bool()
        message.data = ready
        self.ready_pub.publish(message)
        if ready != self.last_ready:
            log = self.get_logger().info if ready else self.get_logger().warn
            log(f"SYSTEM_READY={str(ready).lower()}")
            self.last_ready = ready


def main() -> None:
    rclpy.init()
    node = Go2SafetySupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
