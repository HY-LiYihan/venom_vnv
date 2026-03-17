import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generates the launch description for the Hikrobot Camera node.

    This script loads the camera parameters from a YAML file and launches
    the driver node with appropriate configurations and topic remappings.
    """
    
    # 1. Define Package Name
    package_name = 'hik_camera'

    # 2. Resolve File Paths
    pkg_share = get_package_share_directory(package_name)
    
    # Default paths
    default_params_file = os.path.join(pkg_share, 'config', 'camera_params.yaml')
    default_camera_info_url = f'package://{package_name}/config/camera_info.yaml'

    # 3. Declare Launch Arguments
    # These create the "params_file" configuration variable.
    params_file_arg = DeclareLaunchArgument(
        name='params_file',
        default_value=default_params_file,
        description='Full path to the ROS parameters file to load'
    )

    camera_info_url_arg = DeclareLaunchArgument(
        name='camera_info_url',
        default_value=default_camera_info_url,
        description='URL for the camera calibration info file'
    )

    use_sensor_data_qos_arg = DeclareLaunchArgument(
        name='use_sensor_data_qos',
        default_value='false',
        description='Whether to use Best Effort QoS (SensorData) for image transport'
    )

    # 4. Define the Node
    # Note: We use LaunchConfiguration('params_file') here, so it must be declared before this Node action.
    hik_camera_node = Node(
        package=package_name,
        executable='hik_camera_node',
        name='hik_camera',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
            }
        ]
    )

    # 5. Create Launch Description
    # CRITICAL FIX: The order matters! 
    # Arguments must be declared BEFORE they are used in LogInfo or Node.
    return LaunchDescription([
        # --- Declarations come first ---
        params_file_arg,
        camera_info_url_arg,
        use_sensor_data_qos_arg,
        
        # --- Actions using the configurations come next ---
        LogInfo(msg=['Loading camera params from: ', LaunchConfiguration('params_file')]),
        hik_camera_node
    ])