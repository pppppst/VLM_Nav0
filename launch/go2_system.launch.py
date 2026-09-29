"""Deterministic, fail-closed Go2 Plan A bringup.

This launch never enables the Sport bridge and never starts VLM automatically.
Each downstream stage is created only after the previous topic/TF contract exits
successfully. Hardware values left as TBD make their node fail closed.
"""

from __future__ import annotations

import os
import json
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vlm_nav.go2_preflight import require_calibrated, validate_formal_preflight_report


STAGES = ("fastlio", "obstacle", "scan", "slam", "nav2")


def _static_transform(name, transform):
    xyz = transform["xyz"]
    rpy = transform["rpy"]
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        output="screen",
        arguments=[
            "--x",
            str(xyz[0]),
            "--y",
            str(xyz[1]),
            "--z",
            str(xyz[2]),
            "--roll",
            str(rpy[0]),
            "--pitch",
            str(rpy[1]),
            "--yaw",
            str(rpy[2]),
            "--frame-id",
            transform["parent_frame"],
            "--child-frame-id",
            transform["child_frame"],
        ],
    )


def _waiter(name, *, topics=(), transforms=(), timeout=30.0):
    waiter_parameters = {
        "timeout": timeout,
        "stable_samples": 3,
    }
    if topics:
        waiter_parameters["required_topics"] = list(topics)
    if transforms:
        waiter_parameters["required_transforms"] = list(transforms)
    return Node(
        package="vlm_nav",
        executable="go2_readiness_waiter",
        name=name,
        output="screen",
        parameters=[waiter_parameters],
    )


def _advance_after(waiter, actions, label):
    def on_exit(event, _context):
        if event.returncode != 0:
            return [
                EmitEvent(
                    event=Shutdown(
                        reason=f"Go2 stage {label} readiness failed ({event.returncode})"
                    )
                )
            ]
        return actions

    return RegisterEventHandler(
        OnProcessExit(target_action=waiter, on_exit=on_exit)
    )


def _build(context):
    share = get_package_share_directory("vlm_nav")
    calibration_path = LaunchConfiguration("calibration_file").perform(context)
    target_stage = LaunchConfiguration("target_stage").perform(context)
    preflight_report_path = LaunchConfiguration("sensor_preflight_report").perform(context)
    preflight_config_path = LaunchConfiguration("sensor_preflight_config").perform(context)
    preflight_max_age = float(LaunchConfiguration("sensor_preflight_max_age_s").perform(context))
    bridge_dry_run = LaunchConfiguration("bridge_dry_run").perform(context).lower() in ("1", "true", "yes")
    start_bridge = LaunchConfiguration("start_bridge").perform(context).lower() in ("1", "true", "yes")
    if target_stage not in STAGES:
        raise RuntimeError(f"target_stage must be one of {STAGES}, got {target_stage!r}")

    with open(preflight_report_path, encoding="utf-8") as stream:
        preflight_report = json.load(stream)
    validate_formal_preflight_report(
        preflight_report, now_unix_s=time.time(), max_age_s=preflight_max_age
    )
    with open(preflight_config_path, encoding="utf-8") as stream:
        preflight_config = yaml.safe_load(stream)
    imu_thresholds = preflight_config["imu_stationary_thresholds"]
    if imu_thresholds.get("validated") is not True:
        raise RuntimeError("IMU thresholds are not validated")

    with open(calibration_path, encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    require_calibrated("lidar_imu_extrinsic", calibration["lidar_imu_extrinsic"])
    require_calibrated("base_lidar_extrinsic", calibration["base_lidar_extrinsic"])

    static_nodes = [
        _static_transform("go2_base_to_lidar", calibration["base_lidar_extrinsic"])
    ]
    body_imu = calibration.get("base_imu_extrinsic")
    lowstate_adapter = None
    if body_imu is not None:
        require_calibrated("base_imu_extrinsic", body_imu)
        static_nodes.append(_static_transform("go2_base_to_body_imu", body_imu))
        lowstate_adapter = Node(
            package="vlm_nav",
            executable="lowstate_imu_adapter",
            name="lowstate_imu_adapter",
            output="screen",
        )
    hesai_driver = Node(
        package="hesai_ros_driver",
        executable="hesai_ros_driver_node",
        name="hesai_ros_driver_node",
        output="screen",
        parameters=[
            {"config_path": LaunchConfiguration("hesai_config_file")}
        ],
    )
    converter = Node(
        package="hesai_fastlio_converter",
        executable="hesai_fastlio_converter_node",
        name="hesai_fastlio_converter",
        output="screen",
        parameters=[
            {
                "input_topic": "/lidar_points",
                "output_topic": "/lidar_points_fastlio",
                "input_reliability": "best_effort",
                "output_reliability": "reliable",
                "expected_points": 64000,
                "min_points": 60000,
                "scan_lines": 16,
                "expected_scan_period_sec": 0.1,
                "min_scan_period_sec": 0.05,
                "max_scan_period_sec": 0.15,
                "min_points_per_ring": 3000,
                "input_time_field": "timestamp",
                "input_time_scale": 1.0,
            }
        ],
    )
    camera = calibration["camera_extrinsic"]
    if camera.get("calibrated") is True:
        require_calibrated("camera_extrinsic", camera)
        static_nodes.append(_static_transform("go2_base_to_camera", camera))

    bridge = Node(
        package="vlm_nav",
        executable="twist_to_go2_sport_bridge",
        name="twist_to_go2_sport_bridge",
        output="screen",
        parameters=[os.path.join(share, "config", "go2_bridge.yaml"), {"dry_run": bridge_dry_run}],
    )
    safety_parameters = {
        "formal_report_path": preflight_report_path,
        "formal_report_max_age_s": preflight_max_age,
        "lidar_frame": "hesai_lidar",
        "imu_topic": "/body_imu" if body_imu is not None else "/utlidar/imu",
    }
    safety_supervisor = Node(
        package="vlm_nav",
        executable="go2_safety_supervisor",
        name="go2_safety_supervisor",
        output="screen",
        parameters=[safety_parameters],
    )
    base_waiter = _waiter(
        "wait_go2_base_lidar_tf",
        transforms=("base_link->hesai_lidar",),
        timeout=10.0,
    )
    spark = Node(
        package="spark_fast_lio",
        executable="spark_lio_mapping",
        name="lio_mapping",
        output="screen",
        remappings=[("lidar", "/lidar_points_fastlio"), ("imu", "/body_imu")],
        parameters=[LaunchConfiguration("spark_config_file")],
    )
    fastlio_waiter = _waiter(
        "wait_go2_fastlio",
        topics=("/odometry", "/cloud_registered_base"),
        transforms=("odom->base_link", "base_link->hesai_lidar"),
        timeout=60.0,
    )
    obstacle = Node(
        package="vlm_nav",
        executable="obstacle_cloud_filter",
        name="obstacle_cloud_filter",
        output="screen",
        parameters=[os.path.join(share, "config", "robot_go2.yaml")],
    )
    obstacle_waiter = _waiter(
        "wait_go2_obstacle_cloud",
        topics=("/vlm_nav/obstacle_cloud",),
        timeout=30.0,
    )
    scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="go2_cloud_to_scan",
        output="screen",
        remappings=[
            ("cloud_in", "/vlm_nav/obstacle_cloud"),
            ("scan", "/scan"),
        ],
        parameters=[os.path.join(share, "config", "go2_laserscan.yaml")],
    )
    scan_waiter = _waiter("wait_go2_scan", topics=("/scan",), timeout=30.0)
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[os.path.join(share, "config", "slam_toolbox_go2.yaml")],
    )
    slam_waiter = _waiter(
        "wait_go2_slam",
        topics=("/map",),
        transforms=("map->odom",),
        timeout=120.0,
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "go2_navigation.launch.py")
        ),
        launch_arguments={
            "overrides_file": os.path.join(share, "config", "nav2_go2.yaml")
        }.items(),
    )

    stage_index = STAGES.index(target_stage)
    after_base = [spark, fastlio_waiter]
    after_fastlio = [obstacle, obstacle_waiter] if stage_index >= 1 else []
    after_obstacle = [scan, scan_waiter] if stage_index >= 2 else []
    after_scan = [slam, slam_waiter] if stage_index >= 3 else []
    after_slam = [nav2] if stage_index >= 4 else []

    handlers = [_advance_after(base_waiter, after_base, "base TF")]
    if after_fastlio:
        handlers.append(_advance_after(fastlio_waiter, after_fastlio, "FAST-LIO"))
    if after_obstacle:
        handlers.append(_advance_after(obstacle_waiter, after_obstacle, "obstacle"))
    if after_scan:
        handlers.append(_advance_after(scan_waiter, after_scan, "LaserScan"))
    if after_slam:
        handlers.append(_advance_after(slam_waiter, after_slam, "SLAM"))

    runtime_nodes = [
        *handlers,
        *static_nodes,
        hesai_driver,
        converter,
        safety_supervisor,
        base_waiter,
    ]
    if start_bridge:
        runtime_nodes.append(bridge)
    if lowstate_adapter is not None:
        runtime_nodes.append(lowstate_adapter)
    return runtime_nodes


def generate_launch_description():
    share = get_package_share_directory("vlm_nav")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "calibration_file",
                default_value=os.path.join(share, "config", "go2_calibration_xt16.yaml"),
            ),
            DeclareLaunchArgument(
                "spark_config_file",
                default_value=os.path.join(share, "config", "spark_fast_lio_go2_xt16.yaml"),
            ),
            DeclareLaunchArgument("sensor_preflight_report", default_value=""),
            DeclareLaunchArgument(
                "sensor_preflight_config",
                default_value=os.path.join(share, "config", "go2_preflight_xt16.yaml"),
            ),
            DeclareLaunchArgument(
                "hesai_config_file",
                default_value="/home/isee-pst/Documents/liang/hesai_xt16_ws/src/HesaiLidar_ROS_2.0/config/config.yaml",
            ),
            DeclareLaunchArgument("sensor_preflight_max_age_s", default_value="600.0"),
            DeclareLaunchArgument("target_stage", default_value="fastlio"),
            DeclareLaunchArgument("bridge_dry_run", default_value="true"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            OpaqueFunction(function=_build),
        ]
    )
