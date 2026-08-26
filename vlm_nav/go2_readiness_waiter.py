"""Fail-closed ROS graph/TF waiter used between Go2 bringup stages."""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformListener


class ReadinessWaiter(Node):
    def __init__(self) -> None:
        super().__init__("go2_readiness_waiter")
        self.declare_parameter("required_topics", [])
        self.declare_parameter("required_transforms", [])
        self.declare_parameter("timeout", 30.0)
        self.declare_parameter("stable_samples", 3)
        self.declare_parameter("required_message_count", 3)
        self.required_topics = list(
            self.get_parameter("required_topics").get_parameter_value().string_array_value
        )
        self.required_transforms = list(
            self.get_parameter("required_transforms")
            .get_parameter_value()
            .string_array_value
        )
        self.timeout = float(self.get_parameter("timeout").value)
        self.stable_samples = int(self.get_parameter("stable_samples").value)
        self.required_message_count = int(
            self.get_parameter("required_message_count").value
        )
        self.received_counts = {topic: 0 for topic in self.required_topics}
        self.dynamic_subscriptions = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.started = time.monotonic()
        self.consecutive_ready = 0
        self.exit_code = 1
        self.done = False
        self.timer = self.create_timer(0.2, self.check)

    def _on_message(self, topic):
        self.received_counts[topic] += 1

    def _ensure_subscription(self, topic, type_names):
        if topic in self.dynamic_subscriptions or not type_names:
            return
        try:
            message_type = get_message(type_names[0])
        except (AttributeError, ImportError, ModuleNotFoundError, ValueError) as error:
            self.get_logger().error(
                f"cannot resolve message type for {topic}: {type_names[0]}: {error}"
            )
            return
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(1, self.required_message_count),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.dynamic_subscriptions[topic] = self.create_subscription(
            message_type,
            topic,
            lambda _message, topic=topic: self._on_message(topic),
            qos,
        )

    def check(self) -> None:
        topics = dict(self.get_topic_names_and_types())
        missing_topics = []
        for topic in self.required_topics:
            if topic not in topics:
                missing_topics.append(f"{topic} (no graph endpoint)")
                continue
            self._ensure_subscription(topic, topics[topic])
            count = self.received_counts[topic]
            if count < self.required_message_count:
                missing_topics.append(
                    f"{topic} ({count}/{self.required_message_count} messages)"
                )
        missing_transforms = []
        for specification in self.required_transforms:
            try:
                parent, child = specification.split("->", 1)
            except ValueError:
                self.get_logger().error(
                    f"invalid transform specification {specification!r}; use parent->child"
                )
                self.done = True
                self.timer.cancel()
                return
            if not self.tf_buffer.can_transform(
                parent.strip(), child.strip(), rclpy.time.Time(), Duration(seconds=0.0)
            ):
                missing_transforms.append(specification)

        if not missing_topics and not missing_transforms:
            self.consecutive_ready += 1
            if self.consecutive_ready >= self.stable_samples:
                self.get_logger().info("readiness contract satisfied")
                self.exit_code = 0
                self.done = True
                self.timer.cancel()
            return

        self.consecutive_ready = 0
        if time.monotonic() - self.started > self.timeout:
            self.get_logger().error(
                "readiness timeout; missing topics=%s transforms=%s"
                % (missing_topics, missing_transforms)
            )
            self.done = True
            self.timer.cancel()


def main() -> None:
    rclpy.init()
    node = ReadinessWaiter()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
