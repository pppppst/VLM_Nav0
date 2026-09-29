"""Non-composed Nav2 launch with every motion output routed to the Go2 bridge."""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def deep_merge(destination, source):
    for key, value in source.items():
        if isinstance(value, dict) and isinstance(destination.get(key), dict):
            deep_merge(destination[key], value)
        else:
            destination[key] = value


def configure_go2_navigation(context):
    bringup = get_package_share_directory("nav2_bringup")
    default_params = os.path.join(bringup, "params", "nav2_params.yaml")
    overrides = LaunchConfiguration("overrides_file").perform(context)
    with open(default_params, encoding="utf-8") as stream:
        params = yaml.safe_load(stream)
    with open(overrides, encoding="utf-8") as stream:
        deep_merge(params, yaml.safe_load(stream))
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]
    # The real robot has no /clock; costmap nodes must use wall time too.
    for server in lifecycle_nodes:
        params.setdefault(server, {}).setdefault("ros__parameters", {})[
            "use_sim_time"
        ] = False
    for costmap in ("local_costmap", "global_costmap"):
        params.setdefault(costmap, {}).setdefault(costmap, {}).setdefault(
            "ros__parameters", {}
        )["use_sim_time"] = False
    handle = tempfile.NamedTemporaryFile(
        mode="w", prefix="vlm_nav2_go2_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(params, handle, sort_keys=False)
    handle.close()
    configured_params = handle.name
    common_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    return [
        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[configured_params],
            remappings=common_remaps
            + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', '/cmd_vel_bridge')],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,
                    "autostart": True,
                    "node_names": lifecycle_nodes,
                }
            ],
        ),
        Node(
            package="vlm_nav",
            executable="go2_nav_supervisor",
            name="go2_nav_supervisor",
            output="screen",
        ),
    ]


def generate_launch_description():
    share = get_package_share_directory("vlm_nav")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "overrides_file",
                default_value=os.path.join(share, "config", "nav2_go2.yaml"),
            ),
            OpaqueFunction(function=configure_go2_navigation),
        ]
    )
