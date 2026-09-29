from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [Node(package="vlm_nav", executable="minimal_cloud_probe", output="screen")]
    )
