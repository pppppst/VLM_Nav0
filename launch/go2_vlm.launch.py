"""Start only the Go2 VLM observer after NAV and camera contracts are ready."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vlm_nav.go2_preflight import require_calibrated


def _build(context):
    share = get_package_share_directory("vlm_nav")
    calibration_path = LaunchConfiguration("calibration_file").perform(context)
    with open(calibration_path, encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream)
    require_calibrated("camera_extrinsic", calibration["camera_extrinsic"])

    waiter = Node(
        package="vlm_nav",
        executable="go2_readiness_waiter",
        name="wait_go2_vlm_inputs",
        output="screen",
        parameters=[
            {
                "required_topics": [
                    "/map",
                    "/local_costmap/costmap_raw",
                    "/camera/camera/color/image_raw",
                    "/camera/camera/depth/image_rect_raw",
                    "/camera/camera/color/camera_info",
                    "/camera/camera/depth/camera_info",
                ],
                "required_transforms": [
                    "map->odom",
                    "odom->base_link",
                    "base_link->camera_link",
                    "camera_color_optical_frame->camera_depth_optical_frame",
                    "map->camera_depth_optical_frame",
                ],
                "localization_mode": LaunchConfiguration("localization_mode"),
                "timeout": 30.0,
                "stable_samples": 3,
            }
        ],
    )
    navigator = Node(
        package="vlm_nav",
        executable="vlm_navigator",
        name="vlm_nav",
        output="screen",
        parameters=[
            os.path.join(share, "config", "robot_go2.yaml"),
            {
                "camera_extrinsic_calibrated": True,
                "enabled": LaunchConfiguration("enabled"),
                "target_description": LaunchConfiguration("target_description"),
            },
        ],
    )

    def on_waiter_exit(event, _context):
        if event.returncode != 0:
            return [
                EmitEvent(
                    event=Shutdown(reason="Go2 NAV/RGB-D readiness failed")
                )
            ]
        return [navigator]

    return [
        RegisterEventHandler(
            OnProcessExit(target_action=waiter, on_exit=on_waiter_exit)
        ),
        waiter,
    ]


def generate_launch_description():
    share = get_package_share_directory("vlm_nav")
    return LaunchDescription(
        [
            DeclareLaunchArgument("enabled", default_value="false"),
            DeclareLaunchArgument("target_description", default_value="chair"),
            DeclareLaunchArgument("localization_mode", default_value="slam"),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=os.path.join(share, "config", "go2_calibration.yaml"),
            ),
            OpaqueFunction(function=_build),
        ]
    )
