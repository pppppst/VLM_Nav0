"""Continuous owner of NAV_READY for the Go2 profile."""

from __future__ import annotations

import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


class Go2NavSupervisor(Node):
    NAV2_NODES = (
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    )

    def __init__(self) -> None:
        super().__init__("go2_nav_supervisor")
        self.maximum_age = float(self.declare_parameter("maximum_data_age_s", 2.0).value)
        self.upstream_timeout = float(
            self.declare_parameter("system_ready_timeout_s", 1.0).value
        )
        self.lifecycle_timeout = float(
            self.declare_parameter("lifecycle_timeout_s", 2.0).value
        )
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.ready_pub = self.create_publisher(Bool, "/vlm_nav/nav_ready", latched)
        self.system_ready = False
        self.system_ready_receipt = 0.0
        self.receipts = {}
        self.lifecycle_active = {name: False for name in self.NAV2_NODES}
        self.lifecycle_receipts = {name: 0.0 for name in self.NAV2_NODES}
        self.pending = {name: False for name in self.NAV2_NODES}
        self.lifecycle_requested_at = {name: 0.0 for name in self.NAV2_NODES}
        self.last_ready = None
        self.system_ready_sub = self.create_subscription(
            Bool, "/vlm_nav/system_ready", self._on_system_ready, latched
        )
        self.create_subscription(
            PointCloud2,
            "/vlm_nav/obstacle_cloud",
            lambda _message: self._received("obstacle"),
            sensor,
        )
        self.create_subscription(
            LaserScan, "/scan", lambda _message: self._received("scan"), sensor
        )
        self.create_subscription(
            OccupancyGrid, "/map", lambda _message: self._received("map"), latched
        )
        self.create_subscription(
            Costmap,
            "/local_costmap/costmap_raw",
            lambda _message: self._received("costmap"),
            latched,
        )
        self.clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in self.NAV2_NODES
        }
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self._tick)
        self._publish(False)

    def _on_system_ready(self, message: Bool) -> None:
        self.system_ready = bool(message.data)
        self.system_ready_receipt = time.monotonic()

    def _received(self, key: str) -> None:
        self.receipts[key] = time.monotonic()

    def _request_lifecycle_states(self) -> None:
        now = time.monotonic()
        for name, client in self.clients.items():
            if self.pending[name]:
                if now - self.lifecycle_requested_at[name] > self.lifecycle_timeout:
                    self.lifecycle_active[name] = False
                continue
            if not client.service_is_ready():
                self.lifecycle_active[name] = False
                continue
            self.pending[name] = True
            self.lifecycle_requested_at[name] = now
            future = client.call_async(GetState.Request())
            future.add_done_callback(
                lambda completed, node_name=name: self._on_lifecycle_state(
                    node_name, completed
                )
            )

    def _on_lifecycle_state(self, name, future) -> None:
        self.pending[name] = False
        try:
            self.lifecycle_active[name] = (
                future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
            )
            self.lifecycle_receipts[name] = time.monotonic()
        except Exception as error:  # service loss is a readiness loss, not a crash
            self.lifecycle_active[name] = False
            self.get_logger().warn(f"cannot query lifecycle state for {name}: {error}")

    def _tick(self) -> None:
        self._request_lifecycle_states()
        now = time.monotonic()
        data_fresh = all(
            key in self.receipts and now - self.receipts[key] <= self.maximum_age
            for key in ("obstacle", "scan", "map", "costmap")
        )
        map_tf = self.tf_buffer.can_transform(
            "map", "odom", rclpy.time.Time(), Duration(seconds=0.0)
        )
        system_contract = bool(
            self.system_ready
            and now - self.system_ready_receipt <= self.upstream_timeout
            and self.system_ready_sub.get_publisher_count() == 1
        )
        lifecycle_contract = all(
            self.lifecycle_active[name]
            and now - self.lifecycle_receipts[name] <= self.lifecycle_timeout
            for name in self.NAV2_NODES
        )
        ready = bool(
            system_contract
            and data_fresh
            and map_tf
            and lifecycle_contract
        )
        self._publish(ready)

    def _publish(self, ready: bool) -> None:
        message = Bool()
        message.data = ready
        self.ready_pub.publish(message)
        if ready != self.last_ready:
            log = self.get_logger().info if ready else self.get_logger().warn
            log(f"NAV_READY={str(ready).lower()}")
            self.last_ready = ready


def main() -> None:
    rclpy.init()
    node = Go2NavSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
