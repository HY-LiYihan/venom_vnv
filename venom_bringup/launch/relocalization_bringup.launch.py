import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    livox_driver_dir = get_package_share_directory('livox_ros_driver2')
    point_lio_dir = get_package_share_directory('point_lio')
    small_gicp_dir = get_package_share_directory('small_gicp_relocalization')
    venom_bringup_dir = get_package_share_directory('venom_bringup')

    pcd_file = LaunchConfiguration('pcd_file')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_pcd_file = DeclareLaunchArgument(
        'pcd_file',
        default_value=PathJoinSubstitution([
            EnvironmentVariable("HOME"),
            "venom_ws/src/venom_vnv/lio/Point-LIO/PCD/scans.pcd"
        ]),
        description='Path to prior map PCD file'
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            EnvironmentVariable("HOME"),
            "venom_ws/src/venom_vnv/venom_bringup/rviz_cfg/relocalization.rviz"
        ]),
        description='Path to RViz config file'
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_driver_dir, 'launch_ROS2', 'msg_MID360_launch.py')
        )
    )

    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_lio_dir, 'launch', 'point_lio.launch.py')
        ),
        launch_arguments={'rviz': 'False'}.items()
    )

    prior_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(small_gicp_dir, 'launch', 'prior_map_publisher.launch.py')
        ),
        launch_arguments={'pcd_file': pcd_file}.items()
    )

    relocalization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(small_gicp_dir, 'launch', 'relocalization.launch.py')
        ),
        launch_arguments={'prior_pcd_file': pcd_file}.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        declare_pcd_file,
        declare_rviz_config,
        livox_launch,
        point_lio_launch,
        prior_map_launch,
        relocalization_launch,
        rviz_node,
    ])
