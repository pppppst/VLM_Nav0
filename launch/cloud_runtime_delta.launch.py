from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_probe", default_value="true"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="go2_base_to_lidar",
                arguments=[
                    "--x", "0.171", "--y", "0.0", "--z", "0.0908",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "1.5707963267948966",
                    "--frame-id", "base_link", "--child-frame-id", "hesai_lidar",
                ],
            ),
            Node(
                package="hesai_ros_driver",
                executable="hesai_ros_driver_node",
                name="hesai_ros_driver_node",
                parameters=[{
                    "config_path": "/home/isee-pst/Documents/liang/hesai_xt16_ws/src/HesaiLidar_ROS_2.0/config/config.yaml"
                }],
            ),
            Node(
                package="hesai_fastlio_converter",
                executable="hesai_fastlio_converter_node",
                name="hesai_fastlio_converter",
                parameters=[{
                    "input_topic": "/lidar_points",
                    "output_topic": "/lidar_points_fastlio",
                    "input_reliability": "reliable",
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
                }],
            ),
            Node(
                package="spark_fast_lio",
                executable="spark_lio_mapping",
                name="lio_mapping",
                remappings=[("lidar", "/lidar_points_fastlio"), ("imu", "/utlidar/imu")],
                parameters=["/home/isee-pst/unitree_ros2/VLM_Nav/config/spark_fast_lio_go2_xt16.yaml"],
            ),
            TimerAction(
                period=3.0,
                actions=[Node(package="vlm_nav", executable="minimal_cloud_probe")],
                condition=IfCondition(LaunchConfiguration("start_probe")),
            ),
        ]
    )
