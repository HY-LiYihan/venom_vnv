from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('venom_mission_commander'),
        'config',
        'rmul_sim_mission.yaml',
    ])

    mission_config = LaunchConfiguration('mission_config')
    nav2_wait_mode = LaunchConfiguration('nav2_wait_mode')
    navigator_ready_timeout_sec = LaunchConfiguration('navigator_ready_timeout_sec')
    nav_feedback_log_interval_sec = LaunchConfiguration('nav_feedback_log_interval_sec')
    log_state_transitions = LaunchConfiguration('log_state_transitions')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_config',
            default_value=default_config,
            description='Mission YAML for the currently loaded simulation map.',
        ),
        DeclareLaunchArgument(
            'nav2_wait_mode',
            default_value='bt_navigator',
            description='Nav2 wait mode: bt_navigator or full.',
        ),
        DeclareLaunchArgument(
            'navigator_ready_timeout_sec',
            default_value='30.0',
            description='Startup wait timeout for mock/Nav2 navigator readiness.',
        ),
        DeclareLaunchArgument(
            'nav_feedback_log_interval_sec',
            default_value='5.0',
            description='Seconds between periodic Nav2 feedback logs; 0 disables.',
        ),
        DeclareLaunchArgument(
            'log_state_transitions',
            default_value='false',
            description='Print low-level MissionManager state transitions.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo /clock for Nav2 simulation.',
        ),
        Node(
            package='venom_mission_commander',
            executable='mission_commander',
            name='mission_commander',
            output='screen',
            parameters=[{
                'mission_config': mission_config,
                'use_nav': True,
                'mock_nav_delay_sec': 0.0,
                'nav2_wait_mode': nav2_wait_mode,
                'navigator_ready_timeout_sec': ParameterValue(
                    navigator_ready_timeout_sec,
                    value_type=float,
                ),
                'nav_feedback_log_interval_sec': ParameterValue(
                    nav_feedback_log_interval_sec,
                    value_type=float,
                ),
                'log_state_transitions': ParameterValue(log_state_transitions, value_type=bool),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
