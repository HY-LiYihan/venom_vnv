"""Minimal sequential waypoint commander for quick Nav2 validation.

This node intentionally keeps the mission flow simple for simulation bringup:
it loads a waypoint list from YAML, waits for Nav2 to become active, and sends
each waypoint with ``BasicNavigator.goToPose()`` one by one.

Usage:
    ros2 run venom_bringup simple_waypoint_commander

    ros2 run venom_bringup simple_waypoint_commander --ros-args \
      -p waypoints_file:=/absolute/path/to/waypoints.yaml
"""

import math
import os
import sys
import time
from typing import Any, Dict, List

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rcl_interfaces.msg import ParameterDescriptor


def load_waypoints(file_path: str) -> List[Dict[str, Any]]:
    """Load a non-empty ``waypoints`` list from YAML."""
    if not os.path.isfile(file_path):
        raise FileNotFoundError(f'Waypoints file not found: {file_path}')

    with open(file_path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}

    waypoints = data.get('waypoints')
    if waypoints is None:
        raise KeyError(f"Missing top-level 'waypoints' key in {file_path}")
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError(f'Waypoints list is empty in {file_path}')

    return waypoints


def waypoint_to_pose(waypoint: Dict[str, Any], stamp) -> PoseStamped:
    """Convert one YAML waypoint entry to ``PoseStamped``."""
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


def format_waypoint(waypoint: Dict[str, Any]) -> str:
    """Build a short human-readable waypoint summary."""
    return (
        f'frame={waypoint["frame_id"]}, '
        f'x={float(waypoint["x"]):.2f}, '
        f'y={float(waypoint["y"]):.2f}, '
        f'yaw={float(waypoint["yaw"]):.3f}'
    )


def main() -> None:
    """Run the commander and exit 0 on success, 1 on failure."""
    rclpy.init()
    navigator = BasicNavigator(node_name='simple_waypoint_commander')

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
        'stop_on_failure',
        True,
        ParameterDescriptor(
            description='Stop the mission immediately when one waypoint fails.'
        ),
    )
    navigator.declare_parameter(
        'start_delay_sec',
        0.0,
        ParameterDescriptor(
            description='Optional delay before sending the first waypoint.'
        ),
    )

    waypoints_file = (
        navigator.get_parameter('waypoints_file')
        .get_parameter_value()
        .string_value
    )
    stop_on_failure = (
        navigator.get_parameter('stop_on_failure')
        .get_parameter_value()
        .bool_value
    )
    start_delay_sec = (
        navigator.get_parameter('start_delay_sec')
        .get_parameter_value()
        .double_value
    )

    try:
        raw_waypoints = load_waypoints(waypoints_file)
    except (FileNotFoundError, KeyError, ValueError) as exc:
        navigator.get_logger().fatal(str(exc))
        navigator.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    navigator.get_logger().info(f'Loaded {len(raw_waypoints)} waypoint(s)')
    navigator.get_logger().info('Waiting for bt_navigator to become active...')
    navigator._waitForNodeToActivate('bt_navigator')

    if start_delay_sec > 0.0:
        navigator.get_logger().info(
            f'Waiting {start_delay_sec:.1f}s before starting mission...'
        )
        time.sleep(start_delay_sec)

    failed_indices: List[int] = []

    for index, waypoint in enumerate(raw_waypoints, start=1):
        goal_pose = waypoint_to_pose(
            waypoint, navigator.get_clock().now().to_msg()
        )
        navigator.get_logger().info(
            f'Sending waypoint {index}/{len(raw_waypoints)}: '
            f'{format_waypoint(waypoint)}'
        )
        navigator.goToPose(goal_pose)

        poll_count = 0
        while not navigator.isTaskComplete():
            poll_count += 1
            feedback = navigator.getFeedback()
            if feedback is not None and poll_count % 10 == 0:
                navigator.get_logger().info(
                    f'[Waypoint {index}] distance remaining: '
                    f'{feedback.distance_remaining:.2f} m'
                )

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info(
                f'Waypoint {index}/{len(raw_waypoints)} reached successfully.'
            )
            continue

        failed_indices.append(index)
        if result == TaskResult.CANCELED:
            navigator.get_logger().warn(f'Waypoint {index} was canceled.')
        elif result == TaskResult.FAILED:
            navigator.get_logger().error(f'Waypoint {index} failed.')
        else:
            navigator.get_logger().error(
                f'Waypoint {index} ended with unknown result: {result}'
            )

        if stop_on_failure:
            navigator.get_logger().error(
                'Mission stopped because stop_on_failure=true.'
            )
            navigator.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    if failed_indices:
        navigator.get_logger().warn(
            f'Mission finished with failed waypoint(s): {failed_indices}'
        )
        exit_code = 1
    else:
        navigator.get_logger().info('Mission complete: all waypoints reached.')
        exit_code = 0

    navigator.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
