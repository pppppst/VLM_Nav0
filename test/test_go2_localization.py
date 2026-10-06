"""Offline contracts only: no AMCL process, saved map, or robot required."""

import importlib.util
import math
from pathlib import Path
import subprocess
import time
from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from tf2_ros import TransformException
import yaml

from vlm_nav.go2_nav_supervisor import Go2NavSupervisor
from vlm_nav.go2_readiness_waiter import ReadinessWaiter


ROOT = Path(__file__).resolve().parents[1]


def test_go2_scan_excludes_rear_120_degrees_in_body_frame():
    params = yaml.safe_load((ROOT / "config/go2_laserscan.yaml").read_text())["/**"]["ros__parameters"]
    assert params["target_frame"] == "base_link"
    assert math.degrees(params["angle_min"]) == pytest.approx(-120.0)
    assert math.degrees(params["angle_max"]) == pytest.approx(120.0)


@pytest.fixture
def ros(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    rclpy.init(args=[])
    yield
    rclpy.shutdown()


def launch_module():
    spec = importlib.util.spec_from_file_location("go2_launch", ROOT / "launch/go2_system.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_localization_nodes_are_mutually_exclusive(monkeypatch):
    module = launch_module()
    slam = module._localization_nodes(str(ROOT), "slam", "", "/scan_amcl")
    assert [node.node_package for node in slam] == ["slam_toolbox"]
    # Mock file existence only; do not create or load a pretend saved map.
    monkeypatch.setattr(module.os.path, "isfile", lambda path: path == "/accepted/map.yaml")
    amcl = module._localization_nodes(str(ROOT), "amcl", "/accepted/map.yaml", "/scan_amcl")
    assert [node.node_package for node in amcl] == [
        "nav2_map_server", "nav2_amcl", "nav2_lifecycle_manager"
    ]


@pytest.mark.parametrize("map_path", ["", "/no/such/saved/map.yaml"])
def test_missing_map_fails_before_preflight_or_process_creation(map_path):
    from launch import LaunchContext

    module = launch_module()
    context = LaunchContext()
    context.launch_configurations.update(
        localization_mode="amcl", map=map_path, amcl_scan_topic="/scan_amcl"
    )
    with pytest.raises(RuntimeError, match="AMCL localization requires a valid saved map YAML"):
        module._build(context)
    result = subprocess.run(
        ["bash", str(ROOT / "scripts/manualnav2.sh"), "localization_mode:=amcl", f"map:={map_path}"],
        capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert "AMCL localization requires a valid saved map YAML" in result.stderr


def grid_message():
    # In-memory unit-test input only; never published or written as a map file.
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.width = message.info.height = 1
    message.info.resolution = 0.05
    message.data = [0]
    return message


@pytest.mark.parametrize("mode,required", [("slam", 3), ("amcl", 1)])
def test_map_waiter_qos_and_message_count(ros, mode, required):
    rclpy.shutdown()
    rclpy.init(args=["--ros-args", "-p", f"localization_mode:={mode}",
                     "-p", 'required_topics:=["/map"]'])
    node = ReadinessWaiter()
    try:
        node._ensure_subscription("/map", ["nav_msgs/msg/OccupancyGrid"])
        qos = node.dynamic_subscriptions["/map"].qos_profile
        assert qos.durability == (DurabilityPolicy.TRANSIENT_LOCAL if mode == "amcl" else DurabilityPolicy.VOLATILE)
        assert qos.reliability == (ReliabilityPolicy.RELIABLE if mode == "amcl" else ReliabilityPolicy.BEST_EFFORT)
        node.get_topic_names_and_types = lambda: [("/map", ["nav_msgs/msg/OccupancyGrid"])]
        node._on_message("/map", grid_message())
        for _ in range(3):
            node.check()
        assert node.done == (required == 1)
        if required == 3:
            node._on_message("/map", grid_message())
            node._on_message("/map", grid_message())
            for _ in range(3):
                node.check()
        assert node.done and node.exit_code == 0
    finally:
        node.destroy_node()


@pytest.mark.parametrize("invalid", ["width", "height", "resolution", "frame", "payload", "nan"])
def test_invalid_static_map_does_not_satisfy_waiter(ros, invalid):
    node = ReadinessWaiter()
    try:
        node.localization_mode = "amcl"
        node.received_counts["/map"] = 0
        message = grid_message()
        if invalid in ("width", "height"):
            setattr(message.info, invalid, 0)
        elif invalid == "resolution":
            message.info.resolution = 0.0
        elif invalid == "nan":
            message.info.resolution = float("nan")
        elif invalid == "frame":
            message.header.frame_id = "odom"
        else:
            message.data = []
        node._on_message("/map", message)
        assert node.received_counts["/map"] == 0
    finally:
        node.destroy_node()


@pytest.mark.parametrize("mode", ["slam", "amcl"])
def test_readiness_static_map_and_required_gates(ros, mode):
    rclpy.shutdown()
    rclpy.init(args=["--ros-args", "-p", f"localization_mode:={mode}"])
    node = Go2NavSupervisor()
    try:
        node._request_lifecycle_states = lambda: None
        node.count_publishers = lambda _topic: 1
        published = []
        node._publish = published.append
        now = time.monotonic()
        node.system_ready = True
        node.system_ready_receipt = now
        node.receipts.update(dict.fromkeys(("obstacle", "scan", "costmap"), now))
        node._on_map(grid_message())
        node.receipts["map"] = now - 100.0
        node.lifecycle_active = dict.fromkeys(node.NAV2_NODES, True)
        node.lifecycle_receipts = dict.fromkeys(node.NAV2_NODES, now)
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        node.tf_buffer = Mock()
        node.tf_buffer.can_transform.return_value = True
        node.tf_buffer.lookup_transform.return_value = transform
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = transform.header.stamp
        pose.pose.pose.orientation.w = 1.0
        node._on_amcl_pose(pose)
        node._tick()
        assert published[-1] == (mode == "amcl")
        if mode == "slam":
            node.receipts["map"] = now
            node._tick()
            assert published[-1]
            assert "map_server" not in node.lifecycle_clients
            assert node.scan_topic == "/scan"
            return
        assert node.scan_topic == "/scan_amcl"
        for name in ("map_server", "amcl", "controller_server"):
            node.lifecycle_active[name] = False
            node._tick()
            assert not published[-1], name
            node.lifecycle_active[name] = True
        for key in ("map", "scan", "amcl_pose", "costmap"):
            receipt = node.receipts.pop(key)
            node._tick()
            assert not published[-1], key
            node.receipts[key] = receipt
        node.map_valid = False
        node._tick()
        assert not published[-1]
        node.map_valid = True
        node.tf_buffer.can_transform.return_value = False
        node._tick()
        assert not published[-1]
        node.tf_buffer.can_transform.return_value = True
        transform.header.stamp.sec -= 10
        node._tick()
        assert not published[-1]
        node.tf_buffer.lookup_transform.side_effect = TransformException("missing odom")
        node._tick()
        assert not published[-1]
        node.tf_buffer.lookup_transform.side_effect = None
        transform.header.stamp = node.get_clock().now().to_msg()
        node.receipts["amcl_pose"] -= 10.0
        node._tick()
        assert published[-1]
        pose.pose.pose.orientation.w = 0.0
        node._on_amcl_pose(pose)
        node._tick()
        assert not published[-1]
    finally:
        node.destroy_node()


def test_amcl_config_and_static_costmap_overlay():
    params = yaml.safe_load((ROOT / "config/go2_amcl.yaml").read_text())["amcl"]["ros__parameters"]
    # Temporary field comparison: only the motion-model selection changes.
    assert params["robot_model_type"] == "nav2_amcl::OmniMotionModel"
    assert params["laser_model_type"] == "likelihood_field_prob"
    assert params["scan_topic"] == "/scan_amcl"
    assert params["set_initial_pose"] is False
    assert params["do_beamskip"] is False
    assert all(params[f"alpha{i}"] == 0.1 for i in range(1, 5))
    assert params["alpha5"] == 0.2
    overlay = yaml.safe_load((ROOT / "config/nav2_go2_static_map.yaml").read_text())
    for name in ("global_costmap", "local_costmap"):
        assert overlay[name][name]["ros__parameters"]["plugins"] == ["static_layer", "inflation_layer"]
    launch = (ROOT / "launch/go2_navigation.launch.py").read_text()
    assert 'if localization_mode == "amcl":' in launch
    assert "nav2_go2_static_map.yaml" in launch
    slam = yaml.safe_load((ROOT / "config/nav2_go2.yaml").read_text())
    assert "obstacle_layer" in slam["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"]
    assert "voxel_layer" in slam["local_costmap"]["local_costmap"]["ros__parameters"]["plugins"]
    system_launch = (ROOT / "launch/go2_system.launch.py").read_text()
    assert 'timeout=600.0 if localization_mode == "amcl" else 120.0' in system_launch
    manual = (ROOT / "scripts/manualnav2.sh").read_text()
    assert "amcl_bootstrap" in manual
    assert "use RViz 2D Pose Estimate within 10 minutes" in manual
    assert 'localization_mode:="${localization_mode}"' in manual
    vlm_launch = (ROOT / "launch/go2_vlm.launch.py").read_text()
    assert '"localization_mode": LaunchConfiguration("localization_mode")' in vlm_launch
