"""Single-shot, fail-closed Go2 Gate 5 motion pulse."""

from __future__ import annotations

import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


TOPICS = (
    "/cmd_vel_bridge",
    "/vlm_nav/go2_bridge_state",
    "/vlm_nav/control_armed",
    "/api/sport/request",
    "/api/sport/response",
    "/sportmodestate",
    "/odometry",
)


def _wait(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return predicate()


def _call(node, client, timeout=3.0):
    future = client.call_async(Trigger.Request())
    if not _wait(node, future.done, timeout):
        raise RuntimeError(f"service timeout: {client.srv_name}")
    return future.result()


def _publish_for(node, publisher, values, vx, vyaw, duration):
    command = Twist()
    command.linear.x = vx
    command.angular.z = vyaw
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        publisher.publish(command)
        tick_end = min(deadline, time.monotonic() + 0.05)
        while time.monotonic() < tick_end:
            rclpy.spin_once(node, timeout_sec=tick_end - time.monotonic())
            if values["state"] != "ARMED" or values["armed"] is not True:
                raise RuntimeError(f"bridge left ARMED during motion: {values}")


def _stop_and_settle(node, publisher, values, duration):
    publisher.publish(Twist())
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        rclpy.spin_once(
            node, timeout_sec=max(0.0, min(0.05, deadline - time.monotonic()))
        )
        if values["state"] != "ARMED" or values["armed"] is not True:
            raise RuntimeError(f"bridge left ARMED while stopping: {values}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=0.20)
    parser.add_argument("--yaw-rate", type=float, default=0.50)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--settle-duration", type=float, default=1.0)
    parser.add_argument(
        "--sequence", choices=("single", "linear", "yaw"), default="single"
    )
    parser.add_argument("--execute-extended", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 0.0 < args.speed <= 0.40:
        parser.error("--speed must be in (0, 0.40]")
    if not 0.0 < args.yaw_rate <= 0.50:
        parser.error("--yaw-rate must be in (0, 0.50]")
    if not 0.0 < args.settle_duration <= 5.0:
        parser.error("--settle-duration must be in (0, 5.0]")
    if args.sequence == "single" and not 0.0 < args.duration <= 2.0:
        parser.error("single-pulse --duration must be in (0, 2.0]")
    if args.sequence != "single":
        if not args.execute_extended:
            parser.error("linear/yaw sequences require --execute-extended")
        if not 0.0 < args.duration <= 10.0:
            parser.error("extended --duration must be in (0, 10.0]")

    output = args.output or Path(
        f"/tmp/go2_gate5_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{os.getpid()}"
    )
    if output.exists():
        parser.error(f"output already exists: {output}")

    rclpy.init()
    node = rclpy.create_node("go2_gate5_pulse")
    latched = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    values = {"system_ready": None, "state": None, "armed": None}
    node.create_subscription(
        Bool, "/vlm_nav/system_ready",
        lambda message: values.__setitem__("system_ready", message.data), latched,
    )
    node.create_subscription(
        String, "/vlm_nav/go2_bridge_state",
        lambda message: values.__setitem__("state", message.data), latched,
    )
    node.create_subscription(
        Bool, "/vlm_nav/control_armed",
        lambda message: values.__setitem__("armed", message.data), latched,
    )
    publisher = node.create_publisher(Twist, "/cmd_vel_bridge", 10)
    arm = node.create_client(Trigger, "/twist_to_go2_sport_bridge/arm")
    disarm = node.create_client(Trigger, "/twist_to_go2_sport_bridge/disarm")
    armed_by_tool = False
    bag = None

    try:
        if not arm.wait_for_service(timeout_sec=5.0) or not disarm.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("bridge arm/disarm service unavailable")
        if not _wait(node, lambda: node.count_subscribers("/cmd_vel_bridge") > 0, 5.0):
            raise RuntimeError("bridge cmd_vel subscription unavailable")
        if not _wait(node, lambda: values["system_ready"] is True, 5.0):
            raise RuntimeError("SYSTEM_READY is not true")
        if not _wait(
            node,
            lambda: values["state"] == "DISARMED" and values["armed"] is False,
            5.0,
        ):
            raise RuntimeError(f"bridge is not safely DISARMED: {values}")

        bag = subprocess.Popen(
            ["ros2", "bag", "record", "--output", str(output), *TOPICS],
            start_new_session=True,
        )
        time.sleep(1.0)
        if bag.poll() is not None:
            raise RuntimeError(f"rosbag recorder exited with code {bag.returncode}")

        response = _call(node, arm)
        if not response.success:
            raise RuntimeError(f"ARM failed: {response.message}")
        armed_by_tool = True
        if not _wait(
            node,
            lambda: values["state"] == "ARMED" and values["armed"] is True,
            2.0,
        ):
            raise RuntimeError(f"ARM response/state mismatch: {values}")

        phases = {
            "single": (("forward", args.speed, 0.0),),
            "linear": (
                ("forward", args.speed, 0.0),
                ("backward", -args.speed, 0.0),
            ),
            "yaw": (
                ("counterclockwise", 0.0, args.yaw_rate),
                ("clockwise", 0.0, -args.yaw_rate),
            ),
        }[args.sequence]
        for name, vx, vyaw in phases:
            print(f"START: {name} vx={vx:.2f} vyaw={vyaw:.2f} duration={args.duration:.1f}s")
            _publish_for(node, publisher, values, vx, vyaw, args.duration)
            _stop_and_settle(node, publisher, values, args.settle_duration)
            print(f"STOPPED: {name}")

        response = _call(node, disarm)
        armed_by_tool = False
        if not response.success:
            raise RuntimeError(f"DISARM failed: {response.message}")
        if not _wait(
            node,
            lambda: values["state"] == "DISARMED" and values["armed"] is False,
            2.0,
        ):
            raise RuntimeError(f"DISARM response/state mismatch: {values}")
        print(f"COMPLETE: sequence={args.sequence} bridge DISARMED; bag={output}")
    finally:
        if armed_by_tool and disarm.service_is_ready():
            try:
                _call(node, disarm, timeout=2.0)
            except Exception as error:
                print(f"ERROR: emergency DISARM failed: {error}")
        node.destroy_node()
        rclpy.shutdown()
        if bag is not None and bag.poll() is None:
            os.killpg(bag.pid, signal.SIGINT)
        if bag is not None:
            try:
                bag.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(bag.pid, signal.SIGKILL)
                bag.wait(timeout=5.0)


if __name__ == "__main__":
    main()
