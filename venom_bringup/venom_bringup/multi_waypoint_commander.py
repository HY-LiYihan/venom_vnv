"""Multi-waypoint navigation commander for Scout Mini using Nav2 Simple Commander API.

Loads a list of waypoints from a YAML file, sends them to the Nav2
waypoint_follower action server via BasicNavigator.followWaypoints(),
and polls for feedback until the task is complete.

Usage (after colcon build + source install/setup.bash):
    ros2 run venom_bringup multi_waypoint_commander

    # Override the waypoints file at runtime:
    ros2 run venom_bringup multi_waypoint_commander \\
        --ros-args -p waypoints_file:=/absolute/path/to/waypoints.yaml

The node exits with code 0 on success and 1 on failure or cancellation.
"""

import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rcl_interfaces.msg import ParameterDescriptor

from venom_bringup.road_network_waypoint_utils import load_route_waypoints


# ---------------------------------------------------------------------------
# YAML loading
# ---------------------------------------------------------------------------

def load_waypoints(file_path: str) -> list:
    """Load waypoint definitions from a YAML file.

    Args:
        file_path: Absolute path to the YAML file containing a top-level
            ``waypoints`` list. Each entry must have ``frame_id``, ``x``,
            ``y``, and ``yaw`` keys.

    Returns:
        A list of waypoint dictionaries as read from the file.

    Raises:
        FileNotFoundError: If ``file_path`` does not exist.
        KeyError: If the YAML file is missing the ``waypoints`` key.
        ValueError: If the waypoints list is empty.
    """
    if not os.path.isfile(file_path):
        raise FileNotFoundError(f'Waypoints file not found: {file_path}')

    with open(file_path, 'r') as fh:
        data = yaml.safe_load(fh)

    if 'waypoints' not in data:
        raise KeyError(f"Missing top-level 'waypoints' key in {file_path}")

    waypoints = data['waypoints']
    if not waypoints:
        raise ValueError(f'Waypoints list is empty in {file_path}')

    return waypoints


# ---------------------------------------------------------------------------
# Conversion helper
# ---------------------------------------------------------------------------

def waypoint_to_pose_stamped(waypoint: dict, stamp) -> PoseStamped:
    """Convert a waypoint dictionary to a geometry_msgs/PoseStamped.

    The yaw angle (radians) is converted to a quaternion using the
    simplified formula valid for 2-D planar rotation:
        qz = sin(yaw / 2),  qw = cos(yaw / 2),  qx = qy = 0.

    Args:
        waypoint: Dictionary with keys ``frame_id`` (str), ``x`` (float),
            ``y`` (float), and ``yaw`` (float, radians).
        stamp: ROS Time message (e.g. ``navigator.get_clock().now().to_msg()``).

    Returns:
        A fully populated PoseStamped message.
    """
    pose = PoseStamped()
    pose.header.frame_id = str(waypoint['frame_id'])
    pose.header.stamp = stamp

    pose.pose.position.x = float(waypoint['x'])
    pose.pose.position.y = float(waypoint['y'])
    pose.pose.position.z = 0.0

    yaw = float(waypoint['yaw'])
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)

    return pose


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main() -> None:
    """Run the multi-waypoint commander node.

    Reads the ``waypoints_file`` ROS parameter (defaults to the package-
    installed ``config/scout_mini/waypoints.yaml``), builds PoseStamped
    goals, and sends them to Nav2 via followWaypoints().  Polls feedback
    at approximately 2 Hz until the task finishes, then logs the result
    and exits with code 0 (success) or 1 (failure/cancellation).
    """
    rclpy.init()

    navigator = BasicNavigator(node_name='multi_waypoint_commander')

    # ------------------------------------------------------------------
    # Resolve the waypoints file path from a ROS parameter.
    # Default: <package_share>/config/scout_mini/waypoints.yaml
    # ------------------------------------------------------------------
    pkg_share = get_package_share_directory('venom_bringup')
    default_waypoints_file = os.path.join(
        pkg_share, 'config', 'scout_mini', 'waypoints.yaml'
    )

    navigator.declare_parameter(
        'waypoints_file',
        default_waypoints_file,
        ParameterDescriptor(
            description='Absolute path to the waypoints YAML file.'
        ),
    )
    navigator.declare_parameter(
        'road_network_file',
        '',
        ParameterDescriptor(
            description='Absolute path to a road-network YAML file.'
        ),
    )
    navigator.declare_parameter(
        'route_name',
        '',
        ParameterDescriptor(
            description='Named route to extract from the road-network file.'
        ),
    )
    navigator.declare_parameter(
        'route_nodes',
        '',
        ParameterDescriptor(
            description='Explicit route node list, for example "A,B,C".'
        ),
    )
    navigator.declare_parameter(
        'route_frame_id',
        'map',
        ParameterDescriptor(
            description='Fallback frame_id for road-network route entries.'
        ),
    )
    navigator.declare_parameter(
        'coordinate_mode',
        'auto',
        ParameterDescriptor(
            description='Road-network coordinate mode: auto, geodetic, cartesian_m, or cartesian_cm.'
        ),
    )
    navigator.declare_parameter('map_origin_longitude_deg', 0.0)
    navigator.declare_parameter('map_origin_latitude_deg', 0.0)
    navigator.declare_parameter('map_origin_x_m', 0.0)
    navigator.declare_parameter('map_origin_y_m', 0.0)
    navigator.declare_parameter('map_origin_yaw_rad', 0.0)
    navigator.declare_parameter('start_node_id', '')
    navigator.declare_parameter('goal_node_id', '')
    navigator.declare_parameter('start_x_m', 0.0)
    navigator.declare_parameter('start_y_m', 0.0)
    navigator.declare_parameter('goal_x_m', 0.0)
    navigator.declare_parameter('goal_y_m', 0.0)
    navigator.declare_parameter('use_start_goal_xy', False)
    navigator.declare_parameter('blocked_edges', '')
    waypoints_file: str = (
        navigator.get_parameter('waypoints_file').get_parameter_value().string_value
    )
    road_network_file: str = (
        navigator.get_parameter('road_network_file')
        .get_parameter_value()
        .string_value
    )
    route_name: str = navigator.get_parameter('route_name').get_parameter_value().string_value
    route_nodes: str = navigator.get_parameter('route_nodes').get_parameter_value().string_value
    route_frame_id: str = (
        navigator.get_parameter('route_frame_id').get_parameter_value().string_value
    )
    coordinate_mode: str = (
        navigator.get_parameter('coordinate_mode').get_parameter_value().string_value
    )
    map_origin_longitude_deg = (
        navigator.get_parameter('map_origin_longitude_deg').get_parameter_value().double_value
    )
    map_origin_latitude_deg = (
        navigator.get_parameter('map_origin_latitude_deg').get_parameter_value().double_value
    )
    map_origin_x_m = (
        navigator.get_parameter('map_origin_x_m').get_parameter_value().double_value
    )
    map_origin_y_m = (
        navigator.get_parameter('map_origin_y_m').get_parameter_value().double_value
    )
    map_origin_yaw_rad = (
        navigator.get_parameter('map_origin_yaw_rad').get_parameter_value().double_value
    )
    start_node_id: str = (
        navigator.get_parameter('start_node_id').get_parameter_value().string_value
    )
    goal_node_id: str = (
        navigator.get_parameter('goal_node_id').get_parameter_value().string_value
    )
    use_start_goal_xy = (
        navigator.get_parameter('use_start_goal_xy').get_parameter_value().bool_value
    )
    start_x_m = (
        navigator.get_parameter('start_x_m').get_parameter_value().double_value
        if use_start_goal_xy else None
    )
    start_y_m = (
        navigator.get_parameter('start_y_m').get_parameter_value().double_value
        if use_start_goal_xy else None
    )
    goal_x_m = (
        navigator.get_parameter('goal_x_m').get_parameter_value().double_value
        if use_start_goal_xy else None
    )
    goal_y_m = (
        navigator.get_parameter('goal_y_m').get_parameter_value().double_value
        if use_start_goal_xy else None
    )
    blocked_edges: str = (
        navigator.get_parameter('blocked_edges').get_parameter_value().string_value
    )
    navigator.get_logger().info(f'Loading waypoints from: {waypoints_file}')

    # ------------------------------------------------------------------
    # Load waypoints from YAML
    # ------------------------------------------------------------------
    try:
        if road_network_file:
            raw_waypoints = load_route_waypoints(
                file_path=road_network_file,
                route_name=route_name or None,
                route_nodes=route_nodes or None,
                default_frame_id=route_frame_id,
                coordinate_mode=coordinate_mode,
                map_origin_longitude_deg=map_origin_longitude_deg,
                map_origin_latitude_deg=map_origin_latitude_deg,
                map_origin_yaw_rad=map_origin_yaw_rad,
                map_origin_x_m=map_origin_x_m,
                map_origin_y_m=map_origin_y_m,
                start_node_id=start_node_id or None,
                goal_node_id=goal_node_id or None,
                start_x_m=start_x_m,
                start_y_m=start_y_m,
                goal_x_m=goal_x_m,
                goal_y_m=goal_y_m,
                blocked_edges=blocked_edges or None,
            )
            navigator.get_logger().info(
                f'Loaded {len(raw_waypoints)} waypoint(s) from road network: '
                f'{road_network_file}'
            )
        else:
            raw_waypoints = load_waypoints(waypoints_file)
    except (FileNotFoundError, KeyError, ValueError) as exc:
        navigator.get_logger().fatal(f'[ERROR] {exc}')
        navigator.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if not road_network_file:
        navigator.get_logger().info(
            f'Loaded {len(raw_waypoints)} waypoint(s) from YAML.'
        )

    # ------------------------------------------------------------------
    # Build PoseStamped list
    # ------------------------------------------------------------------
    now = navigator.get_clock().now().to_msg()
    goal_poses = [waypoint_to_pose_stamped(wp, now) for wp in raw_waypoints]

    # ------------------------------------------------------------------
    # Wait for bt_navigator to become active.
    # slam_toolbox runs as a plain node (no managed lifecycle), so
    # waitUntilNav2Active() cannot be used — it always polls a localizer's
    # get_state service first (defaulting to amcl), which will never appear.
    # _waitForNodeToActivate(bt_navigator) is sufficient: bt_navigator
    # reaches ACTIVE only after costmaps and controller_server are ready.
    # ------------------------------------------------------------------
    navigator.get_logger().info('Waiting for bt_navigator to become active...')
    navigator._waitForNodeToActivate('bt_navigator')

    # ------------------------------------------------------------------
    # Send the waypoint mission
    # ------------------------------------------------------------------
    total = len(goal_poses)
    navigator.get_logger().info(
        f'Sending {total} waypoint(s) to Nav2 followWaypoints...'
    )
    accepted = navigator.followWaypoints(goal_poses)
    if not accepted:
        navigator.get_logger().error(
            '[ERROR] followWaypoints goal was rejected by Nav2.'
        )
        navigator.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # ------------------------------------------------------------------
    # Poll for completion and print per-waypoint progress.
    # isTaskComplete() internally spins with a ~0.10 s timeout, so
    # printing every 10 polls gives approximately one log line per second.
    # ------------------------------------------------------------------
    poll_count = 0

    while not navigator.isTaskComplete():
        poll_count += 1
        feedback = navigator.getFeedback()

        if feedback is not None and poll_count % 10 == 0:
            current_idx: int = feedback.current_waypoint
            wp = raw_waypoints[current_idx]
            navigator.get_logger().info(
                f'[Progress] Navigating to waypoint '
                f'{current_idx + 1}/{total} '
                f'(x={float(wp["x"]):.2f}, '
                f'y={float(wp["y"]):.2f}, '
                f'yaw={float(wp["yaw"]):.3f} rad)'
            )

    # ------------------------------------------------------------------
    # Report final result
    # ------------------------------------------------------------------
    result: TaskResult = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info(
            f'Mission complete: all {total} waypoint(s) reached successfully.'
        )
        exit_code = 0
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn(
            '[WARN] Mission was canceled before completion.'
        )
        exit_code = 1
    elif result == TaskResult.FAILED:
        navigator.get_logger().error(
            '[ERROR] Mission failed. Check Nav2 logs for details.'
        )
        exit_code = 1
    else:
        navigator.get_logger().error(
            f'[ERROR] Mission ended with unknown result: {result}'
        )
        exit_code = 1

    navigator.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
