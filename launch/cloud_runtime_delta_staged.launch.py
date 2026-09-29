from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="go2_base_to_lidar",
        arguments=[
            "--x", "0.171", "--y", "0.0", "--z", "0.0908",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "1.5707963267948966",
            "--frame-id", "base_link", "--child-frame-id", "hesai_lidar",
        ],
    )
    driver = Node(
        package="hesai_ros_driver",
        executable="hesai_ros_driver_node",
        name="hesai_ros_driver_node",
        parameters=[{
            "config_path": "/home/isee-pst/Documents/liang/hesai_xt16_ws/src/HesaiLidar_ROS_2.0/config/config.yaml"
        }],
    )
    converter = Node(
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
    )
    base_waiter = Node(
        package="vlm_nav",
        executable="go2_readiness_waiter",
        name="wait_go2_base_lidar_tf",
        parameters=[{
            "timeout": 10.0,
            "stable_samples": 3,
            "required_transforms": ["base_link->hesai_lidar"],
        }],
    )
    spark = Node(
        package="spark_fast_lio",
        executable="spark_lio_mapping",
        name="lio_mapping",
        remappings=[("lidar", "/lidar_points_fastlio"), ("imu", "/utlidar/imu")],
        parameters=[
            "/home/isee-pst/unitree_ros2/VLM_Nav/config/spark_fast_lio_go2_xt16.yaml"
        ],
    )
    fastlio_waiter = Node(
        package="vlm_nav",
        executable="go2_readiness_waiter",
        name="wait_go2_fastlio",
        condition=IfCondition(LaunchConfiguration("with_fastlio_waiter")),
    )
    start_spark = RegisterEventHandler(
        OnProcessExit(target_action=base_waiter, on_exit=[spark, fastlio_waiter])
    )
    bridge = Node(
        package="vlm_nav",
        executable="twist_to_go2_sport_bridge",
        name="twist_to_go2_sport_bridge",
        parameters=["/home/isee-pst/unitree_ros2/VLM_Nav/config/go2_bridge.yaml"],
        condition=IfCondition(LaunchConfiguration("with_bridge_safety")),
    )
    safety = Node(
        package="vlm_nav",
        executable="go2_safety_supervisor",
        name="go2_safety_supervisor",
        parameters=[{
            "formal_report_path": "/tmp/gate3_preflight_60s.json",
            "formal_report_max_age_s": 86400.0,
            "lidar_frame": "hesai_lidar",
            "acceleration_abs_max_mps2": 16.0,
            "gyro_abs_max_radps": 0.07,
        }],
        condition=IfCondition(LaunchConfiguration("with_bridge_safety")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("with_bridge_safety", default_value="false"),
        DeclareLaunchArgument("with_fastlio_waiter", default_value="false"),
        start_spark,
        static_tf,
        driver,
        converter,
        bridge,
        safety,
        base_waiter,
    ])
