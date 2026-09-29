import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from unitree_api.msg import Request, Response


def _spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    return predicate()


def test_dry_run_arms_before_command_and_faults_on_foreign_sport_request(
    tmp_path, monkeypatch
):
    executable = (
        Path(get_package_prefix("vlm_nav"))
        / "lib"
        / "vlm_nav"
        / "twist_to_go2_sport_bridge"
    )
    suffix = f"p{os.getpid()}"
    cmd_topic = f"/vlm_nav/test/{suffix}/cmd_vel"
    sport_topic = f"/vlm_nav/test/{suffix}/sport_request"
    debug_topic = f"/vlm_nav/test/{suffix}/debug_request"
    state_topic = f"/vlm_nav/test/{suffix}/bridge_state"
    node_name = f"go2_bridge_runtime_test_{suffix}"
    ros_log_dir = tmp_path / "ros_logs"
    ros_log_dir.mkdir()
    monkeypatch.setenv("ROS_LOG_DIR", str(ros_log_dir))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-r",
            f"__node:={node_name}",
            "-p",
            "dry_run:=true",
            "-p",
            "require_system_ready:=false",
            "-p",
            "endpoint_loss_debounce:=0.2",
            "-p",
            "foreign_sport_quiet_period_s:=0.0",
            "-p",
            f"cmd_vel_topic:={cmd_topic}",
            "-p",
            f"sport_request_topic:={sport_topic}",
            "-p",
            f"sport_response_topic:={sport_topic}/response",
            "-p",
            f"debug_request_topic:={debug_topic}",
            "-p",
            f"state_topic:={state_topic}",
        ],
        env=os.environ.copy(),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    rclpy.init()
    probe = rclpy.create_node(f"go2_bridge_runtime_probe_{suffix}")
    commands = []
    states = []
    real_sport_requests = []
    transient_reliable = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    cmd_pub = probe.create_publisher(Twist, cmd_topic, 10)
    debug_sub = probe.create_subscription(
        String, debug_topic, lambda message: commands.append(json.loads(message.data)), 10
    )
    state_sub = probe.create_subscription(
        String, state_topic, lambda message: states.append(message.data), transient_reliable
    )
    sport_sub = probe.create_subscription(
        Request, sport_topic, lambda message: real_sport_requests.append(message), 10
    )
    foreign_sport_pub = probe.create_publisher(Request, sport_topic, 10)
    sport_response_pub = probe.create_publisher(Response, f"{sport_topic}/response", 10)
    arm = probe.create_client(Trigger, f"/{node_name}/arm")

    try:
        assert arm.wait_for_service(timeout_sec=5.0)
        assert _spin_until(probe, lambda: probe.count_publishers(sport_topic) >= 2, 5.0)

        assert _spin_until(probe, lambda: probe.count_subscribers(sport_topic) >= 1, 2.0)
        assert _spin_until(
            probe,
            lambda: probe.count_subscribers(f"{sport_topic}/response") >= 1,
            2.0,
        )
        time.sleep(0.1)
        future = arm.call_async(Trigger.Request())
        assert _spin_until(probe, future.done, 2.0)
        assert future.result().success, future.result().message

        request = Twist()
        request.linear.x = 0.8
        request.angular.z = -0.9
        cmd_pub.publish(request)
        assert _spin_until(
            probe,
            lambda: any(command["api_id"] == 1008 for command in commands),
            1.0,
        )
        move = next(command for command in commands if command["api_id"] == 1008)
        move_parameter = json.loads(move["parameter"])
        assert move_parameter == {"x": 0.4, "y": 0.0, "z": -0.5}

        foreign = Request()
        foreign.header.identity.id = 9001
        foreign.header.identity.api_id = 1027
        foreign_sport_pub.publish(foreign)
        assert _spin_until(probe, lambda: any(
            state.startswith("FAULT: foreign Sport request") for state in states
        ), 2.0)
        assert _spin_until(
            probe,
            lambda: [command["api_id"] for command in commands].count(1003) == 1,
            1.0,
        )
        assert [command["api_id"] for command in commands].count(1003) == 1
        assert any(
            command["api_id"] == 1008
            and json.loads(command["parameter"])
            == {"x": 0.0, "y": 0.0, "z": 0.0}
            for command in commands
        )
        assert any(message.header.identity.id == 9001 for message in real_sport_requests)
    finally:
        probe.destroy_subscription(debug_sub)
        probe.destroy_subscription(state_sub)
        probe.destroy_subscription(sport_sub)
        probe.destroy_publisher(foreign_sport_pub)
        probe.destroy_publisher(sport_response_pub)
        probe.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate(timeout=5.0)
