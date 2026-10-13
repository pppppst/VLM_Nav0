"""Camera mount TF only: no hardware bringup or movement."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from vlm_nav.go2_camera_tf import camera_tf_action


def generate_launch_description():
    default = os.path.join(get_package_share_directory('vlm_nav'), 'config', 'go2_calibration_xt16.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=default),
        OpaqueFunction(function=lambda context: [camera_tf_action(
            LaunchConfiguration('calibration_file').perform(context))]),
    ])
