#!/usr/bin/env python3
"""Publish a frozen, read-only Go2 footprint yaw sweep for RViz."""

import math
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


HALF_LENGTH = 0.38
HALF_WIDTH = 0.18


def yaw_offsets_degrees():
    return list(range(-90, 91, 5))


def rectangle_points():
    corners = (
        (HALF_LENGTH, HALF_WIDTH),
        (HALF_LENGTH, -HALF_WIDTH),
        (-HALF_LENGTH, -HALF_WIDTH),
        (-HALF_LENGTH, HALF_WIDTH),
        (HALF_LENGTH, HALF_WIDTH),
    )
    return [Point(x=float(x), y=float(y), z=0.0) for x, y in corners]


def yaw_from_quaternion(rotation):
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


class Go2FootprintSweepDebug(Node):
    def __init__(self):
        super().__init__("go2_footprint_sweep_debug")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            MarkerArray, "/vlm_nav/go2_footprint_sweep", qos
        )
        self.frozen_pose = None
        self.last_tf_warning = 0.0
        self.timer = self.create_timer(1.0, self.publish_sweep)

    def freeze_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", Time(), timeout=Duration(seconds=0.2)
            ).transform
        except TransformException as error:
            now = time.monotonic()
            if now - self.last_tf_warning >= 5.0:
                self.get_logger().warning(f"waiting for map -> base_link TF: {error}")
                self.last_tf_warning = now
            return False

        self.frozen_pose = (
            transform.translation.x,
            transform.translation.y,
            transform.translation.z + 0.05,
            yaw_from_quaternion(transform.rotation),
        )
        x, y, _, yaw = self.frozen_pose
        self.get_logger().info(
            f"frozen base_link pose: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f} deg"
        )
        return True

    def publish_sweep(self):
        if self.frozen_pose is None and not self.freeze_pose():
            return

        x, y, z, base_yaw = self.frozen_pose
        stamp = self.get_clock().now().to_msg()
        points = rectangle_points()
        markers = []
        for marker_id, offset_degrees in enumerate(yaw_offsets_degrees()):
            yaw = base_yaw + math.radians(offset_degrees)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "go2_footprint_sweep"
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            marker.points = points
            marker.scale.x = 0.03 if offset_degrees == 0 else 0.012
            marker.color.r = 1.0 if offset_degrees == 0 else 0.70
            marker.color.g = 1.0 if offset_degrees == 0 else 0.70
            marker.color.b = 1.0 if offset_degrees == 0 else 0.70
            marker.color.a = 1.0 if offset_degrees == 0 else 0.28
            markers.append(marker)
        self.publisher.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args)
    node = Go2FootprintSweepDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
