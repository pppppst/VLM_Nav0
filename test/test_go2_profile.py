from pathlib import Path
import importlib.util
import os

import yaml
import pytest

from vlm_nav.go2_config_renderer import render_fastlio_config


ROOT = Path(__file__).resolve().parents[1]


def load_launch_module():
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/go2_roslog")
    spec = importlib.util.spec_from_file_location(
        "go2_system_launch", ROOT / "launch/go2_system.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_yaml(relative):
    return yaml.safe_load((ROOT / relative).read_text(encoding="utf-8"))


def test_go2_calibration_uses_unitree_l1_lidar_imu_and_urdf_base_lidar_candidate():
    calibration = load_yaml("config/go2_calibration.yaml")

    lidar_imu = calibration["lidar_imu_extrinsic"]
    assert lidar_imu["calibrated"] is True
    assert "Unitree official L1" in lidar_imu["source"]
    assert lidar_imu["definition"] == (
        "p_imu = R_lidar_to_imu * p_lidar + T_lidar_in_imu"
    )
    assert lidar_imu["lidar_frame"] == "utlidar_lidar"
    assert lidar_imu["imu_frame"] == "utlidar_imu"
    assert lidar_imu["translation"] == [0.007698, 0.014655, -0.00667]
    assert lidar_imu["rotation_matrix"] == [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]

    base_lidar = calibration["base_lidar_extrinsic"]
    assert base_lidar["calibrated"] is False
    assert "official Go2 URDF" in base_lidar["source"]
    assert base_lidar["parent_frame"] == "base_link"
    assert base_lidar["child_frame"] == "utlidar_lidar"
    assert base_lidar["xyz"] == [0.28945, 0.0, -0.046825]
    assert base_lidar["rpy"] == [0.0, 2.8782, 0.0]

    camera = calibration["camera_extrinsic"]
    assert camera["calibrated"] is False
    assert camera["parent_frame"] == "base_link"
    assert camera["child_frame"] == "camera_link"
    assert camera["xyz"] == "TBD / requires measurement"
    assert camera["rpy"] == "TBD / requires measurement"


def test_go2_waiter_omits_empty_ros_parameter_arrays():
    launch = load_launch_module()
    waiter = launch._waiter("test_waiter", transforms=("base_link->utlidar_lidar",))
    parameters = waiter.__dict__["_Node__parameters"][0]
    values = list(parameters.values())
    assert all(value != () for value in values)


def test_go2_fastlio_contract_has_single_plan_a_tf_owner_and_base_cloud():
    config = load_yaml("config/spark_fast_lio_go2.yaml")
    params = config["/**"]["ros__parameters"]

    assert params["common"]["lid_topic"] == "/utlidar/cloud"
    assert params["common"]["imu_topic"] == "/utlidar/imu"
    assert params["common"]["map_frame"] == "odom"
    assert params["common"]["base_frame"] == "base_link"
    assert params["publish"]["scan_baseframe_pub_en"] is True
    assert params["mapping"]["extrinsic_T"] == "TBD / requires measurement"
    assert params["mapping"]["extrinsic_R"] == "TBD / requires measurement"


def test_go2_robot_profile_uses_registered_base_cloud_and_blocks_uncalibrated_vlm():
    config = load_yaml("config/robot_go2.yaml")
    obstacle = config["obstacle_cloud_filter"]["ros__parameters"]
    vlm = config["vlm_nav"]["ros__parameters"]

    assert obstacle["input_topic"] == "/cloud_registered_base"
    assert obstacle["target_frame"] == "base_link"
    assert obstacle["min_height"] == -0.35
    assert obstacle["max_height"] == 0.20
    assert obstacle["self_crop_min_x"] == -0.40
    assert obstacle["self_crop_max_x"] == 0.40
    assert obstacle["self_crop_min_y"] == -0.18
    assert obstacle["self_crop_max_y"] == 0.18
    assert obstacle["self_crop_min_z"] == -0.40
    assert obstacle["self_crop_max_z"] == 0.16
    assert vlm["rgb_topic"] == "/camera/camera/color/image_raw"
    assert vlm["depth_topic"] == "/camera/camera/aligned_depth_to_color/image_raw"
    assert vlm["camera_info_topic"] == "/camera/camera/color/camera_info"
    assert vlm["require_camera_calibration"] is True
    assert vlm["camera_extrinsic_calibrated"] is False
    assert vlm["stop_cmd_topic"] == "/cmd_vel_bridge"
    assert vlm["arrival_odom_topic"] == "/odometry"


def test_go2_nav2_is_explicit_nonholonomic_dwb_profile():
    config = load_yaml("config/nav2_go2.yaml")
    controller = config["controller_server"]["ros__parameters"]
    follow = controller["FollowPath"]
    smoother = config["velocity_smoother"]["ros__parameters"]

    assert follow["plugin"] == "dwb_core::DWBLocalPlanner"
    assert follow["min_vel_y"] == 0.0
    assert follow["max_vel_y"] == 0.0
    assert follow["acc_lim_y"] == 0.0
    assert follow["decel_lim_y"] == 0.0
    assert follow["vy_samples"] == 1
    assert smoother["max_velocity"][1] == 0.0
    assert smoother["min_velocity"][1] == 0.0
    assert smoother["max_accel"][1] == 0.0
    assert smoother["max_decel"][1] == 0.0
    assert follow["max_vel_x"] == 0.20
    assert follow["max_vel_theta"] == 0.40
    global_cloud = config["global_costmap"]["global_costmap"]["ros__parameters"][
        "obstacle_layer"
    ]["go2_cloud"]
    local_cloud = config["local_costmap"]["local_costmap"]["ros__parameters"][
        "voxel_layer"
    ]["go2_cloud"]
    for cloud in (global_cloud, local_cloud):
        assert cloud["topic"] == "/vlm_nav/obstacle_cloud"
        assert cloud["sensor_frame"] == "utlidar_lidar"


def test_go2_nav2_routes_smoothed_output_to_bridge_without_relay():
    source = (ROOT / "launch/go2_navigation.launch.py").read_text()
    system = (ROOT / "launch/go2_system.launch.py").read_text()

    assert "go2_navigation.launch.py" in system
    assert "nav2_velocity_smoother" in source
    assert "('cmd_vel', 'cmd_vel_nav')" in source
    assert "('cmd_vel_smoothed', '/cmd_vel_bridge')" in source
    assert "relay" not in source.lower()


def test_go2_profile_does_not_enable_ranger_adapter_or_publish_fake_camera_tf():
    launch = (ROOT / "launch/go2_system.launch.py").read_text(encoding="utf-8")

    assert "fastlio_odom_adapter" not in launch
    assert "static_transform_publisher" in launch
    assert "base_lidar_extrinsic" in launch
    assert "camera_extrinsic" in launch
    assert "calibrated" in launch
    assert 'if camera.get("calibrated") is True' in launch
    assert "-0.16" not in launch
    assert "1.185" not in launch


def test_go2_fastlio_is_gated_by_fresh_formal_preflight_and_runtime_supervisor():
    launch = (ROOT / "launch/go2_system.launch.py").read_text(encoding="utf-8")
    setup = (ROOT / "setup.py").read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("sensor_preflight_report"' in launch
    assert "validate_formal_preflight_report" in launch
    assert "go2_safety_supervisor" in launch
    assert '"/vlm_nav/system_ready"' in (
        ROOT / "vlm_nav/go2_safety_supervisor.py"
    ).read_text(encoding="utf-8")
    assert "go2_safety_supervisor =" in setup


def test_ranger_profile_files_remain_separate_from_go2_profile():
    ranger = load_yaml("config/robot.yaml")
    go2 = load_yaml("config/robot_go2.yaml")

    assert ranger["obstacle_cloud_filter"]["ros__parameters"]["input_topic"] == (
        "/cloud_registered_body"
    )
    assert go2["obstacle_cloud_filter"]["ros__parameters"]["input_topic"] == (
        "/cloud_registered_base"
    )
    assert (ROOT / "config/nav2_overrides.yaml").exists()
    assert (ROOT / "config/nav2_go2.yaml").exists()


def test_system_launch_defaults_to_ranger_and_selects_go2_explicitly():
    source = (ROOT / "launch/system.launch.py").read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("robot_profile", default_value="ranger")' in source
    assert '"robot_profile", "go2"' in source
    assert '"robot_profile", "ranger"' in source
    assert "go2_system.launch.py" in source


def test_vlm_enable_gate_rejects_uncalibrated_go2_camera():
    source = (ROOT / "vlm_nav/vlm_navigator.py").read_text(encoding="utf-8")

    assert '"require_camera_calibration": False' in source
    assert '"camera_extrinsic_calibrated": False' in source
    assert "camera extrinsic is not calibrated" in source
    assert '"require_external_safety_gates": False' in source
    assert "NAV_READY / CONTROL_ARMED / VLM API gate is false" in source

    go2 = load_yaml("config/robot_go2.yaml")["vlm_nav"]["ros__parameters"]
    assert go2["require_external_safety_gates"] is True


def test_fastlio_renderer_uses_reviewed_unitree_l1_extrinsic_direction():
    template = load_yaml("config/spark_fast_lio_go2.yaml")
    calibration = load_yaml("config/go2_calibration.yaml")
    measurements = load_yaml("config/go2_preflight.yaml")

    unreviewed = load_yaml("config/go2_preflight.yaml")
    unreviewed["lidar_timing"]["validated"] = False
    with pytest.raises(ValueError, match="lidar_timing is not validated"):
        render_fastlio_config(template, calibration, unreviewed)

    rendered = render_fastlio_config(template, calibration, measurements)
    params = rendered["/**"]["ros__parameters"]

    assert params["mapping"]["extrinsic_T"] == [0.007698, 0.014655, -0.00667]
    assert params["mapping"]["extrinsic_R"] == [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    assert params["preprocess"]["scan_line"] == 18
    assert params["preprocess"]["scan_rate"] == 16
    assert params["preprocess"]["timestamp_unit"] == 0


def test_go2_vlm_launch_is_separate_calibrated_and_disabled_by_default():
    source = (ROOT / "launch/go2_vlm.launch.py").read_text()

    assert 'DeclareLaunchArgument("enabled", default_value="false")' in source
    assert 'require_calibrated("camera_extrinsic"' in source
    assert '"camera_extrinsic_calibrated": True' in source
    assert "fastlio_odom_adapter" not in source
    assert "obstacle_cloud_filter" not in source
    assert "/camera/camera/aligned_depth_to_color/image_raw" in source
    assert "map->odom" in source
