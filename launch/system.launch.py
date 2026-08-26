"""Select the preserved Ranger stack or the fail-closed Go2 Plan A stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("vlm_nav")
    launch_dir = os.path.join(share, "launch")
    ranger_params = os.path.join(share, "config", "robot.yaml")

    ranger = GroupAction(
        condition=LaunchConfigurationEquals("robot_profile", "ranger"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "mapping.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "navigation.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "vlm_navigation.launch.py")
                ),
                launch_arguments={
                    "params_file": ranger_params,
                    "enabled": LaunchConfiguration("enabled"),
                    "target_description": LaunchConfiguration("target_description"),
                    "enable_odom_adapter": LaunchConfiguration("enable_odom_adapter"),
                    "publish_camera_tf": LaunchConfiguration("publish_camera_tf"),
                }.items(),
            ),
        ],
    )
    go2 = GroupAction(
        condition=LaunchConfigurationEquals("robot_profile", "go2"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "go2_system.launch.py")
                ),
                launch_arguments={
                    "target_stage": LaunchConfiguration("go2_target_stage"),
                    "calibration_file": LaunchConfiguration("go2_calibration_file"),
                    "spark_config_file": LaunchConfiguration("go2_spark_config_file"),
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_profile", default_value="ranger"),
            DeclareLaunchArgument("enabled", default_value="false"),
            DeclareLaunchArgument("target_description", default_value="chair"),
            DeclareLaunchArgument("enable_odom_adapter", default_value="true"),
            DeclareLaunchArgument("publish_camera_tf", default_value="true"),
            DeclareLaunchArgument("go2_target_stage", default_value="fastlio"),
            DeclareLaunchArgument(
                "go2_calibration_file",
                default_value=os.path.join(share, "config", "go2_calibration.yaml"),
            ),
            DeclareLaunchArgument(
                "go2_spark_config_file",
                default_value=os.path.join(
                    share, "config", "spark_fast_lio_go2.yaml"
                ),
            ),
            ranger,
            go2,
        ]
    )
