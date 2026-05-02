"""
Launch file for health-aware waypoint navigation.

This launch file starts the health-aware multi-waypoint commander with:
- Health monitoring and emergency return
- Mission state persistence
- Round-trip navigation

Usage:
    ros2 launch venom_bringup health_aware_navigation.launch.py
    
    # With custom waypoints file:
    ros2 launch venom_bringup health_aware_navigation.launch.py \\
        waypoints_file:=/path/to/waypoints.yaml
    
    # With custom config file:
    ros2 launch venom_bringup health_aware_navigation.launch.py \\
        mission_config_file:=/path/to/mission_config.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for health-aware navigation"""
    
    # Get package share directory
    pkg_share = get_package_share_directory('venom_bringup')
    
    # Default file paths
    default_waypoints_file = os.path.join(
        pkg_share, 'config', 'scout_mini', 'waypoints.yaml'
    )
    
    default_config_file = os.path.join(
        pkg_share, 'config', 'scout_mini', 'mission_config.yaml'
    )
    default_road_network_file = ''
    
    # Declare launch arguments
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints_file,
        description='Path to waypoints YAML file'
    )
    road_network_file_arg = DeclareLaunchArgument(
        'road_network_file',
        default_value=default_road_network_file,
        description='Path to a road-network YAML file'
    )
    route_name_arg = DeclareLaunchArgument(
        'route_name',
        default_value='',
        description='Named route to extract from the road-network file'
    )
    route_nodes_arg = DeclareLaunchArgument(
        'route_nodes',
        default_value='',
        description='Explicit route node list, for example A,B,C'
    )
    route_frame_id_arg = DeclareLaunchArgument(
        'route_frame_id',
        default_value='map',
        description='Fallback frame_id for road-network route entries'
    )
    coordinate_mode_arg = DeclareLaunchArgument(
        'coordinate_mode',
        default_value='auto',
        description='Road-network coordinate mode'
    )
    start_node_id_arg = DeclareLaunchArgument(
        'start_node_id',
        default_value='',
        description='Graph-search start node id'
    )
    goal_node_id_arg = DeclareLaunchArgument(
        'goal_node_id',
        default_value='',
        description='Graph-search goal node id'
    )
    start_x_m_arg = DeclareLaunchArgument(
        'start_x_m',
        default_value='0.0',
        description='Graph-search start x in meters'
    )
    start_y_m_arg = DeclareLaunchArgument(
        'start_y_m',
        default_value='0.0',
        description='Graph-search start y in meters'
    )
    goal_x_m_arg = DeclareLaunchArgument(
        'goal_x_m',
        default_value='0.0',
        description='Graph-search goal x in meters'
    )
    goal_y_m_arg = DeclareLaunchArgument(
        'goal_y_m',
        default_value='0.0',
        description='Graph-search goal y in meters'
    )
    use_start_goal_xy_arg = DeclareLaunchArgument(
        'use_start_goal_xy',
        default_value='false',
        description='Whether to use start/goal XY instead of node ids'
    )
    blocked_edges_arg = DeclareLaunchArgument(
        'blocked_edges',
        default_value='',
        description='Blocked edges, for example A->B;B->C'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'mission_config_file',
        default_value=default_config_file,
        description='Path to mission configuration YAML file'
    )
    
    # Get launch configurations
    waypoints_file = LaunchConfiguration('waypoints_file')
    road_network_file = LaunchConfiguration('road_network_file')
    route_name = LaunchConfiguration('route_name')
    route_nodes = LaunchConfiguration('route_nodes')
    route_frame_id = LaunchConfiguration('route_frame_id')
    coordinate_mode = LaunchConfiguration('coordinate_mode')
    start_node_id = LaunchConfiguration('start_node_id')
    goal_node_id = LaunchConfiguration('goal_node_id')
    start_x_m = LaunchConfiguration('start_x_m')
    start_y_m = LaunchConfiguration('start_y_m')
    goal_x_m = LaunchConfiguration('goal_x_m')
    goal_y_m = LaunchConfiguration('goal_y_m')
    use_start_goal_xy = LaunchConfiguration('use_start_goal_xy')
    blocked_edges = LaunchConfiguration('blocked_edges')
    config_file = LaunchConfiguration('mission_config_file')
    
    # Create health-aware commander node
    health_aware_commander = Node(
        package='venom_bringup',
        executable='multi_waypoint_commander',
        name='health_aware_commander',
        output='screen',
        parameters=[
            {'waypoints_file': waypoints_file},
            {'road_network_file': road_network_file},
            {'route_name': route_name},
            {'route_nodes': route_nodes},
            {'route_frame_id': route_frame_id},
            {'coordinate_mode': coordinate_mode},
            {'start_node_id': start_node_id},
            {'goal_node_id': goal_node_id},
            {'start_x_m': start_x_m},
            {'start_y_m': start_y_m},
            {'goal_x_m': goal_x_m},
            {'goal_y_m': goal_y_m},
            {'use_start_goal_xy': use_start_goal_xy},
            {'blocked_edges': blocked_edges},
            {'mission_config_file': config_file}
        ],
        remappings=[
            ('/game_status', '/game_status'),
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom')
        ]
    )
    
    # Alternative: Run as a process (uncomment if needed)
    # health_aware_process = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'run', 'venom_bringup', 'multi_waypoint_commander',
    #         '--ros-args',
    #         '-p', ['waypoints_file:=', waypoints_file],
    #         '-p', ['mission_config_file:=', config_file]
    #     ],
    #     output='screen'
    # )
    
    return LaunchDescription([
        waypoints_file_arg,
        road_network_file_arg,
        route_name_arg,
        route_nodes_arg,
        route_frame_id_arg,
        coordinate_mode_arg,
        start_node_id_arg,
        goal_node_id_arg,
        start_x_m_arg,
        start_y_m_arg,
        goal_x_m_arg,
        goal_y_m_arg,
        use_start_goal_xy_arg,
        blocked_edges_arg,
        config_file_arg,
        health_aware_commander,
        
        # Or use the process version:
        # health_aware_process
    ])
