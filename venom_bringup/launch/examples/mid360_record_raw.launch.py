import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    venom_bringup_dir = get_package_share_directory("venom_bringup")

    bag_dir = LaunchConfiguration("bag_dir")

    livox_user_config = os.path.join(
        venom_bringup_dir, "config", "examples", "MID360_config.json"
    )
    default_bag_dir = os.path.join(
        os.path.expanduser("~"),
        "venom_bags",
        "mid360_raw",
        f"mid360_raw_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )

    declare_bag_dir = DeclareLaunchArgument(
        "bag_dir",
        default_value=default_bag_dir,
        description="Output directory for raw Mid360 rosbag recording.",
    )

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {"xfer_format": 1},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": "base_link"},
            {"lvx_file_path": "/home/livox/livox_test.lvx"},
            {"user_config_path": livox_user_config},
            {"cmdline_input_bd_code": "livox0000000001"},
        ],
    )

    rosbag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            bag_dir,
            "/livox/lidar",
            "/livox/imu",
            "/tf",
            "/tf_static",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_bag_dir,
            livox_driver,
            rosbag_record,
        ]
    )
