import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('venom_serial_driver')
    config_file = os.path.join(pkg_dir, 'config', 'serial_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Serial port name'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='921600',
            description='Baud rate'
        ),

        Node(
            package='venom_serial_driver',
            executable='serial_node',
            name='serial_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'port_name': LaunchConfiguration('port_name'),
                    'baud_rate': LaunchConfiguration('baud_rate'),
                }
            ]
        )
    ])
