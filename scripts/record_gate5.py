#!/usr/bin/env python3
"""Read-only Gate 5 evidence recorder (topics plus optional rosbag)."""
import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from unitree_api.msg import Request
from diagnostic_msgs.msg import DiagnosticArray


def stamp(msg):
    h = getattr(msg, "header", None)
    return None if h is None else h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", type=Path, default=Path("/tmp/go2_gate5_record"))
    parser.add_argument("--no-bag", action="store_true")
    parser.add_argument("--discovery-timeout", type=float, default=10.0)
    args = parser.parse_args()
    topics = ["/api/sport/request", "/body_imu", "/odometry", "/tf",
              "/cloud_registered_base", "/vlm_nav/system_ready",
              "/vlm_nav/go2_bridge_state", "/diagnostics"]
    rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    if rmw and rmw != "rmw_cyclonedds_cpp":
        raise RuntimeError(f"recorder requires rmw_cyclonedds_cpp, got {rmw}")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    # Keep the recorder and its rosbag child on the same explicit ROS domain/RMW.
    domain = os.environ.setdefault("ROS_DOMAIN_ID", "0")
    bag = None
    if not args.no_bag:
        bag = subprocess.Popen(["ros2", "bag", "record", "-o", str(args.output) + ".bag", *topics],
                               env=os.environ.copy())
    rclpy.init()
    node = rclpy.create_node("gate5_readonly_recorder")
    records = {"sport": [], "imu": [], "odom": [], "tf": [], "cloud": 0,
               "system_ready": [], "bridge_state": [], "diagnostics": []}
    node.create_subscription(Request, "/api/sport/request",
                             lambda m: records["sport"].append({"time": time.time(),
                                 "api_id": m.header.identity.api_id,
                                 "parameter": m.parameter}), 10)
    def imu_cb(m):
        values = [m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                  m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
        records["imu"].append({"stamp": stamp(m), "finite": all(math.isfinite(v) for v in values)})
    node.create_subscription(Imu, "/body_imu", imu_cb, 50)
    def odom_cb(m):
        q = m.pose.pose.orientation
        records["odom"].append({
            "stamp": stamp(m), "x": m.pose.pose.position.x, "y": m.pose.pose.position.y,
            "q": [q.x, q.y, q.z, q.w],
            "yaw": math.atan2(2 * (q.w * q.z + q.x * q.y),
                               1 - 2 * (q.y * q.y + q.z * q.z)),
        })
    node.create_subscription(Odometry, "/odometry", odom_cb, 10)
    def tf_cb(m):
        for t in m.transforms:
            if t.header.frame_id == "odom" and t.child_frame_id == "base_link":
                records["tf"].append(stamp(t))
    node.create_subscription(TFMessage, "/tf", tf_cb, 50)
    node.create_subscription(PointCloud2, "/cloud_registered_base",
                             lambda _: records.__setitem__("cloud", records["cloud"] + 1), 1)
    node.create_subscription(Bool, "/vlm_nav/system_ready",
                             lambda m: records["system_ready"].append(m.data), 1)
    node.create_subscription(String, "/vlm_nav/go2_bridge_state",
                             lambda m: records["bridge_state"].append(m.data), 1)
    node.create_subscription(DiagnosticArray, "/diagnostics",
                             lambda m: records["diagnostics"].append(time.time()), 10)
    discovery_deadline = time.monotonic() + args.discovery_timeout
    while node.count_publishers("/api/sport/request") == 0 and time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.count_publishers("/api/sport/request") == 0:
        raise RuntimeError("/api/sport/request discovery timed out")
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    # Keep spinning after the motion window so an explicit StopMove at the
    # boundary is retained in both the topic sample and the rosbag.
    settle_deadline = time.monotonic() + 2.0
    while time.monotonic() < settle_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if bag:
        bag.send_signal(signal.SIGINT)
        bag.wait(timeout=10)
    yaws = []
    for item in records["odom"]:
        value = item["yaw"]
        if yaws:
            while value - yaws[-1] > math.pi: value -= 2 * math.pi
            while value - yaws[-1] < -math.pi: value += 2 * math.pi
        yaws.append(value)
    positions = [(item["x"], item["y"]) for item in records["odom"]]
    distances = [math.hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(positions, positions[1:])]
    yaw_jumps = [abs(b - a) for a, b in zip(yaws, yaws[1:])]
    odom_stamps = [item["stamp"] for item in records["odom"]]
    summary = {
        "rmw_implementation": os.environ["RMW_IMPLEMENTATION"],
        "ros_domain_id": domain,
        "duration_sec": args.duration,
        "sport_count": len(records["sport"]),
        "sport": records["sport"],
        "imu_count": len(records["imu"]),
        "imu_nonfinite": sum(not x["finite"] for x in records["imu"]),
        "odom_rollback": sum(b < a for a, b in zip(odom_stamps, odom_stamps[1:])),
        "odom_duplicate": sum(b == a for a, b in zip(odom_stamps, odom_stamps[1:])),
        "tf_rollback": sum(b < a for a, b in zip(records["tf"], records["tf"][1:])),
        "tf_duplicate": sum(b == a for a, b in zip(records["tf"], records["tf"][1:])),
        "cloud_count": records["cloud"],
        "system_ready": records["system_ready"][-1] if records["system_ready"] else None,
        "bridge_state": records["bridge_state"][-1] if records["bridge_state"] else None,
        "delta_x": (positions[-1][0] - positions[0][0]) if positions else None,
        "delta_y": (positions[-1][1] - positions[0][1]) if positions else None,
        "delta_yaw_deg": math.degrees(yaws[-1] - yaws[0]) if yaws else None,
        "trajectory_length_m": sum(distances),
        "max_position_jump_m": max(distances, default=0.0),
        "max_yaw_jump_deg": math.degrees(max(yaw_jumps, default=0.0)),
    }
    args.output.with_suffix(".json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
