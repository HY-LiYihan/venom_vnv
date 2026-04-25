import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pose_bridge_launch = os.path.join(
        get_package_share_directory("venom_px4_bridge"),
        "launch",
        "px4_external_pose_bridge.launch.py",
    )

    fmu_prefix = LaunchConfiguration("fmu_prefix")
    input_odom_topic = LaunchConfiguration("input_odom_topic")

    declare_args = [
        DeclareLaunchArgument(
            "fmu_prefix",
            default_value="/fmu",
            description="PX4 DDS topic namespace prefix.",
        ),
        DeclareLaunchArgument(
            "input_odom_topic",
            default_value="/lio/vps/odometry",
            description="Odometry topic published by the Lio/VPS algorithm (nav_msgs/Odometry, ENU/FLU).",
        ),
    ]

    pose_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pose_bridge_launch),
        launch_arguments={
            "fmu_prefix": fmu_prefix,
            "input_odom_topic": input_odom_topic,
        }.items(),
    )

    return LaunchDescription(declare_args + [pose_bridge])
