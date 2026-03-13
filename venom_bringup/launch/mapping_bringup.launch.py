import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    livox_driver_dir = get_package_share_directory('livox_ros_driver2')
    point_lio_dir = get_package_share_directory('point_lio')

    # Include Livox driver launch
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_driver_dir, 'launch_ROS2', 'msg_MID360_launch.py')
        )
    )

    # Include Point-LIO mapping launch
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_lio_dir, 'launch', 'mapping.launch.py')
        )
    )

    # Static TF: map to odom (identity, Point-LIO provides localization)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Static TF: base_link to laser_link
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.3', '0', '0', '0', 'base_link', 'laser_link']
    )

    # Convert 3D point cloud to 2D laser scan
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'laser_link',
            'transform_tolerance': 0.01,
            'min_height': -0.2,
            'max_height': 0.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.001,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 50.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/cloud_registered'),
            ('scan', '/scan')
        ]
    )

    # 2D SLAM mapping with slam_toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'laser_link',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'transform_publish_period': 0.0,  # Disable map->odom transform publishing
            'use_scan_matching': False,

            # 地图分辨率
            'resolution': 0.05,

            # 扫描匹配参数
            'minimum_travel_distance': 0.2,
            'minimum_travel_heading': 0.2,

            'map_update_interval': 1.0,
        }]
    )

    # Delay slam_toolbox startup by 10 seconds
    delayed_slam_toolbox = TimerAction(
        period=5.0,
        actions=[slam_toolbox_node]
    )

    return LaunchDescription([
        livox_launch,
        point_lio_launch,
        map_to_odom_tf,
        base_to_laser_tf,
        pointcloud_to_laserscan_node,
        delayed_slam_toolbox,
    ])
