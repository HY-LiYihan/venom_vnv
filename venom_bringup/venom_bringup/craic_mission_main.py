"""CRAIC competition mission commander built on Nav2 Simple Commander."""

from __future__ import annotations

import math
import sys
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from tf2_ros import Buffer, TransformListener

from venom_bringup.craic_waypoint_utils import CraicWaypoint, load_craic_waypoints
from venom_bringup.road_network_waypoint_utils import (
    load_planned_road_route,
    route_to_craic_waypoints,
)
from venom_bringup.waypoint_behavior import (
    WaypointBehaviorConfig,
    WaypointExecutionPlan,
    build_execution_plan,
    build_resume_plan,
    normalize_angle,
    quaternion_to_yaw,
)


def distance_xy(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


class CraicMissionCommander(BasicNavigator):
    """Waypoint-driven competition mission node."""

    def __init__(self) -> None:
        super().__init__(node_name='craic_mission_main')

        self._declare_parameters()
        self._load_parameters()

        self._route_node_ids: List[str] = []
        self._planned_route_name: Optional[str] = None
        if self.road_network_file:
            planned_route = load_planned_road_route(
                file_path=self.road_network_file,
                route_name=self.route_name or None,
                route_nodes=self.route_nodes or None,
                default_frame_id=self.waypoint_frame_id,
                coordinate_mode=self.coordinate_mode,
                map_origin_longitude_deg=self.map_origin_longitude_deg,
                map_origin_latitude_deg=self.map_origin_latitude_deg,
                map_origin_yaw_rad=self.map_origin_yaw_rad,
                map_origin_x_m=self.map_origin_x_m,
                map_origin_y_m=self.map_origin_y_m,
                start_node_id=self.start_node_id or None,
                goal_node_id=self.goal_node_id or None,
                start_x_m=self.start_x_m,
                start_y_m=self.start_y_m,
                goal_x_m=self.goal_x_m,
                goal_y_m=self.goal_y_m,
                blocked_edges=self.blocked_edges or None,
            )
            self._waypoints = route_to_craic_waypoints(planned_route)
            self._route_node_ids = planned_route.route_node_ids
            self._planned_route_name = planned_route.route_name
        else:
            self._waypoints = load_craic_waypoints(
                file_path=self.waypoint_file,
                coordinate_mode=self.coordinate_mode,
                origin_longitude_deg=self.map_origin_longitude_deg,
                origin_latitude_deg=self.map_origin_latitude_deg,
                map_origin_yaw_rad=self.map_origin_yaw_rad,
                map_origin_x_m=self.map_origin_x_m,
                map_origin_y_m=self.map_origin_y_m,
                use_first_waypoint_as_origin=self.use_first_waypoint_as_origin,
            )
        self._goal_poses = [self._to_pose_stamped(waypoint) for waypoint in self._waypoints]

        self._cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._odom_sub = self.create_subscription(
            Odometry,
            self.pose_tracking_topic,
            self._on_pose_update,
            20,
        )
        self._controller_param_client = self.create_client(
            SetParameters,
            '/controller_server/set_parameters',
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._current_pose_xy: Optional[tuple[float, float]] = None
        self._current_abs_waypoint_index = 0
        self._active_slice_start = 0
        self._last_logged_waypoint_index = -1
        self._last_progress_pose_xy: Optional[tuple[float, float]] = None
        self._last_progress_time = time.monotonic()
        self._recovery_attempts = 0
        self._following_waypoints = False
        self._current_yaw: Optional[float] = None
        self._active_plan: Optional[WaypointExecutionPlan] = None
        self._special_action_retry_count = 0
        self._behavior_config = WaypointBehaviorConfig(
            default_final_stop_distance_m=self.final_goal_stop_distance_m,
            cruise_max_linear_speed_mps=self.cruise_max_linear_speed_mps,
            cruise_max_speed_xy_mps=self.cruise_max_speed_xy_mps,
            cruise_max_angular_speed_radps=self.cruise_max_angular_speed_radps,
            cruise_xy_goal_tolerance_m=self.cruise_xy_goal_tolerance_m,
            cruise_yaw_goal_tolerance_rad=self.cruise_yaw_goal_tolerance_rad,
            left_turn_max_linear_speed_mps=self.left_turn_max_linear_speed_mps,
            left_turn_max_speed_xy_mps=self.left_turn_max_speed_xy_mps,
            left_turn_max_angular_speed_radps=self.left_turn_max_angular_speed_radps,
            left_turn_position_tolerance_m=self.left_turn_position_tolerance_m,
            left_turn_yaw_tolerance_rad=self.left_turn_yaw_tolerance_rad,
            left_turn_settle_time_sec=self.left_turn_settle_time_sec,
            right_turn_max_linear_speed_mps=self.right_turn_max_linear_speed_mps,
            right_turn_max_speed_xy_mps=self.right_turn_max_speed_xy_mps,
            right_turn_max_angular_speed_radps=self.right_turn_max_angular_speed_radps,
            right_turn_position_tolerance_m=self.right_turn_position_tolerance_m,
            right_turn_yaw_tolerance_rad=self.right_turn_yaw_tolerance_rad,
            right_turn_settle_time_sec=self.right_turn_settle_time_sec,
            park_max_linear_speed_mps=self.park_max_linear_speed_mps,
            park_max_speed_xy_mps=self.park_max_speed_xy_mps,
            park_max_angular_speed_radps=self.park_max_angular_speed_radps,
            park_position_tolerance_m=self.park_position_tolerance_m,
            park_yaw_tolerance_rad=self.park_yaw_tolerance_rad,
            park_settle_time_sec=self.park_settle_time_sec,
            special_action_retry_limit=self.special_action_retry_limit,
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            'waypoint_file',
            '',
            ParameterDescriptor(description='Path to CRAIC waypoint.txt task file.'),
        )
        self.declare_parameter(
            'coordinate_mode',
            'geodetic',
            ParameterDescriptor(description='One of geodetic, cartesian_m, cartesian_cm, auto.'),
        )
        self.declare_parameter('map_origin_longitude_deg', 0.0)
        self.declare_parameter('map_origin_latitude_deg', 0.0)
        self.declare_parameter('map_origin_x_m', 0.0)
        self.declare_parameter('map_origin_y_m', 0.0)
        self.declare_parameter('map_origin_yaw_rad', 0.0)
        self.declare_parameter('use_first_waypoint_as_origin', True)
        self.declare_parameter('road_network_file', '')
        self.declare_parameter('route_name', '')
        self.declare_parameter('route_nodes', '')
        self.declare_parameter('start_node_id', '')
        self.declare_parameter('goal_node_id', '')
        self.declare_parameter('start_x_m', 0.0)
        self.declare_parameter('start_y_m', 0.0)
        self.declare_parameter('goal_x_m', 0.0)
        self.declare_parameter('goal_y_m', 0.0)
        self.declare_parameter('use_start_goal_xy', False)
        self.declare_parameter('blocked_edges', '')
        self.declare_parameter('waypoint_frame_id', 'map')
        self.declare_parameter('pose_tracking_topic', '/odometry/global')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('startup_wait_timeout_sec', 90.0)
        self.declare_parameter('require_map_topic', True)
        self.declare_parameter('require_pose_topic', True)
        self.declare_parameter('require_tf_ready', True)
        self.declare_parameter('nav2_activation_timeout_sec', 60.0)
        self.declare_parameter('final_goal_stop_distance_m', 10.0)
        self.declare_parameter('stuck_timeout_sec', 10.0)
        self.declare_parameter('stuck_progress_radius_m', 0.8)
        self.declare_parameter('max_recovery_attempts', 5)
        self.declare_parameter('backup_distance_m', 0.8)
        self.declare_parameter('backup_speed_mps', 0.2)
        self.declare_parameter('spin_angle_rad', 1.57)
        self.declare_parameter('recovery_time_allowance_sec', 15.0)
        self.declare_parameter('progress_check_period_sec', 0.2)
        self.declare_parameter('left_turn_position_tolerance_m', 0.45)
        self.declare_parameter('left_turn_yaw_tolerance_rad', 0.22)
        self.declare_parameter('left_turn_settle_time_sec', 0.35)
        self.declare_parameter('left_turn_max_linear_speed_mps', 0.8)
        self.declare_parameter('left_turn_max_speed_xy_mps', 0.8)
        self.declare_parameter('left_turn_max_angular_speed_radps', 0.9)
        self.declare_parameter('right_turn_position_tolerance_m', 0.35)
        self.declare_parameter('right_turn_yaw_tolerance_rad', 0.30)
        self.declare_parameter('right_turn_settle_time_sec', 0.20)
        self.declare_parameter('right_turn_max_linear_speed_mps', 0.65)
        self.declare_parameter('right_turn_max_speed_xy_mps', 0.65)
        self.declare_parameter('right_turn_max_angular_speed_radps', 0.8)
        self.declare_parameter('park_position_tolerance_m', 0.18)
        self.declare_parameter('park_yaw_tolerance_rad', 0.12)
        self.declare_parameter('park_settle_time_sec', 1.0)
        self.declare_parameter('park_max_linear_speed_mps', 0.35)
        self.declare_parameter('park_max_speed_xy_mps', 0.35)
        self.declare_parameter('park_max_angular_speed_radps', 0.45)
        self.declare_parameter('cruise_max_linear_speed_mps', 2.0)
        self.declare_parameter('cruise_max_speed_xy_mps', 2.0)
        self.declare_parameter('cruise_max_angular_speed_radps', 1.6)
        self.declare_parameter('cruise_xy_goal_tolerance_m', 0.5)
        self.declare_parameter('cruise_yaw_goal_tolerance_rad', 0.4)
        self.declare_parameter('action_control_rate_hz', 20.0)
        self.declare_parameter('action_heading_kp', 1.6)
        self.declare_parameter('action_linear_kp', 0.8)
        self.declare_parameter('action_min_angular_speed_radps', 0.18)
        self.declare_parameter('action_min_linear_speed_mps', 0.05)
        self.declare_parameter('action_heading_gate_rad', 0.45)
        self.declare_parameter('special_action_entry_distance_m', 1.2)
        self.declare_parameter('turn_action_timeout_sec', 10.0)
        self.declare_parameter('park_action_timeout_sec', 15.0)
        self.declare_parameter('turn_creep_max_linear_mps', 0.25)
        self.declare_parameter('park_final_align_max_angular_speed_radps', 0.35)
        self.declare_parameter('special_action_retry_limit', 2)

    def _load_parameters(self) -> None:
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.coordinate_mode = self.get_parameter('coordinate_mode').value
        self.map_origin_longitude_deg = float(self.get_parameter('map_origin_longitude_deg').value)
        self.map_origin_latitude_deg = float(self.get_parameter('map_origin_latitude_deg').value)
        self.map_origin_x_m = float(self.get_parameter('map_origin_x_m').value)
        self.map_origin_y_m = float(self.get_parameter('map_origin_y_m').value)
        self.map_origin_yaw_rad = float(self.get_parameter('map_origin_yaw_rad').value)
        self.use_first_waypoint_as_origin = bool(
            self.get_parameter('use_first_waypoint_as_origin').value
        )
        self.road_network_file = self.get_parameter('road_network_file').value
        self.route_name = self.get_parameter('route_name').value
        self.route_nodes = self.get_parameter('route_nodes').value
        self.start_node_id = self.get_parameter('start_node_id').value
        self.goal_node_id = self.get_parameter('goal_node_id').value
        self.use_start_goal_xy = bool(self.get_parameter('use_start_goal_xy').value)
        self.start_x_m = float(self.get_parameter('start_x_m').value) if self.use_start_goal_xy else None
        self.start_y_m = float(self.get_parameter('start_y_m').value) if self.use_start_goal_xy else None
        self.goal_x_m = float(self.get_parameter('goal_x_m').value) if self.use_start_goal_xy else None
        self.goal_y_m = float(self.get_parameter('goal_y_m').value) if self.use_start_goal_xy else None
        self.blocked_edges = self.get_parameter('blocked_edges').value
        self.waypoint_frame_id = self.get_parameter('waypoint_frame_id').value
        self.pose_tracking_topic = self.get_parameter('pose_tracking_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.startup_wait_timeout_sec = float(
            self.get_parameter('startup_wait_timeout_sec').value
        )
        self.require_map_topic = bool(self.get_parameter('require_map_topic').value)
        self.require_pose_topic = bool(self.get_parameter('require_pose_topic').value)
        self.require_tf_ready = bool(self.get_parameter('require_tf_ready').value)
        self.nav2_activation_timeout_sec = float(
            self.get_parameter('nav2_activation_timeout_sec').value
        )
        self.final_goal_stop_distance_m = float(
            self.get_parameter('final_goal_stop_distance_m').value
        )
        self.stuck_timeout_sec = float(self.get_parameter('stuck_timeout_sec').value)
        self.stuck_progress_radius_m = float(
            self.get_parameter('stuck_progress_radius_m').value
        )
        self.max_recovery_attempts = int(self.get_parameter('max_recovery_attempts').value)
        self.backup_distance_m = float(self.get_parameter('backup_distance_m').value)
        self.backup_speed_mps = float(self.get_parameter('backup_speed_mps').value)
        self.spin_angle_rad = float(self.get_parameter('spin_angle_rad').value)
        self.recovery_time_allowance_sec = float(
            self.get_parameter('recovery_time_allowance_sec').value
        )
        self.progress_check_period_sec = float(
            self.get_parameter('progress_check_period_sec').value
        )
        self.left_turn_position_tolerance_m = float(
            self.get_parameter('left_turn_position_tolerance_m').value
        )
        self.left_turn_yaw_tolerance_rad = float(
            self.get_parameter('left_turn_yaw_tolerance_rad').value
        )
        self.left_turn_settle_time_sec = float(
            self.get_parameter('left_turn_settle_time_sec').value
        )
        self.left_turn_max_linear_speed_mps = float(
            self.get_parameter('left_turn_max_linear_speed_mps').value
        )
        self.left_turn_max_speed_xy_mps = float(
            self.get_parameter('left_turn_max_speed_xy_mps').value
        )
        self.left_turn_max_angular_speed_radps = float(
            self.get_parameter('left_turn_max_angular_speed_radps').value
        )
        self.right_turn_position_tolerance_m = float(
            self.get_parameter('right_turn_position_tolerance_m').value
        )
        self.right_turn_yaw_tolerance_rad = float(
            self.get_parameter('right_turn_yaw_tolerance_rad').value
        )
        self.right_turn_settle_time_sec = float(
            self.get_parameter('right_turn_settle_time_sec').value
        )
        self.right_turn_max_linear_speed_mps = float(
            self.get_parameter('right_turn_max_linear_speed_mps').value
        )
        self.right_turn_max_speed_xy_mps = float(
            self.get_parameter('right_turn_max_speed_xy_mps').value
        )
        self.right_turn_max_angular_speed_radps = float(
            self.get_parameter('right_turn_max_angular_speed_radps').value
        )
        self.park_position_tolerance_m = float(
            self.get_parameter('park_position_tolerance_m').value
        )
        self.park_yaw_tolerance_rad = float(
            self.get_parameter('park_yaw_tolerance_rad').value
        )
        self.park_settle_time_sec = float(
            self.get_parameter('park_settle_time_sec').value
        )
        self.park_max_linear_speed_mps = float(
            self.get_parameter('park_max_linear_speed_mps').value
        )
        self.park_max_speed_xy_mps = float(
            self.get_parameter('park_max_speed_xy_mps').value
        )
        self.park_max_angular_speed_radps = float(
            self.get_parameter('park_max_angular_speed_radps').value
        )
        self.cruise_max_linear_speed_mps = float(
            self.get_parameter('cruise_max_linear_speed_mps').value
        )
        self.cruise_max_speed_xy_mps = float(
            self.get_parameter('cruise_max_speed_xy_mps').value
        )
        self.cruise_max_angular_speed_radps = float(
            self.get_parameter('cruise_max_angular_speed_radps').value
        )
        self.cruise_xy_goal_tolerance_m = float(
            self.get_parameter('cruise_xy_goal_tolerance_m').value
        )
        self.cruise_yaw_goal_tolerance_rad = float(
            self.get_parameter('cruise_yaw_goal_tolerance_rad').value
        )
        self.action_control_rate_hz = float(
            self.get_parameter('action_control_rate_hz').value
        )
        self.action_heading_kp = float(self.get_parameter('action_heading_kp').value)
        self.action_linear_kp = float(self.get_parameter('action_linear_kp').value)
        self.action_min_angular_speed_radps = float(
            self.get_parameter('action_min_angular_speed_radps').value
        )
        self.action_min_linear_speed_mps = float(
            self.get_parameter('action_min_linear_speed_mps').value
        )
        self.action_heading_gate_rad = float(
            self.get_parameter('action_heading_gate_rad').value
        )
        self.special_action_entry_distance_m = float(
            self.get_parameter('special_action_entry_distance_m').value
        )
        self.turn_action_timeout_sec = float(
            self.get_parameter('turn_action_timeout_sec').value
        )
        self.park_action_timeout_sec = float(
            self.get_parameter('park_action_timeout_sec').value
        )
        self.turn_creep_max_linear_mps = float(
            self.get_parameter('turn_creep_max_linear_mps').value
        )
        self.park_final_align_max_angular_speed_radps = float(
            self.get_parameter('park_final_align_max_angular_speed_radps').value
        )
        self.special_action_retry_limit = int(
            self.get_parameter('special_action_retry_limit').value
        )

    def _on_pose_update(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self._current_pose_xy = (position.x, position.y)
        orientation = msg.pose.pose.orientation
        self._current_yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

    def _to_pose_stamped(self, waypoint: CraicWaypoint) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.waypoint_frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(waypoint.yaw * 0.5)
        pose.pose.orientation.w = math.cos(waypoint.yaw * 0.5)
        return pose

    def _log_current_waypoint(self, waypoint_index: int) -> None:
        if waypoint_index == self._last_logged_waypoint_index:
            return
        waypoint = self._waypoints[waypoint_index]
        profile_name = self._active_plan.profile_name if self._active_plan is not None else 'unknown'
        self.get_logger().info(
            'Heading to waypoint '
            f'{waypoint_index + 1}/{len(self._waypoints)} '
            f'(task_index={waypoint.index}, action={waypoint.action_label}, profile={profile_name}, '
            f'x={waypoint.x:.2f}, y={waypoint.y:.2f})'
        )
        self._last_logged_waypoint_index = waypoint_index

    def _publish_zero_velocity(self, repeat_count: int = 5) -> None:
        stop_msg = Twist()
        for _ in range(repeat_count):
            self._cmd_vel_pub.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _publish_velocity(self, linear_x: float, angular_z: float) -> None:
        cmd_msg = Twist()
        cmd_msg.linear.x = float(linear_x)
        cmd_msg.angular.z = float(angular_z)
        self._cmd_vel_pub.publish(cmd_msg)

    def _sleep_with_spin(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _clamp(self, value: float, lower_bound: float, upper_bound: float) -> float:
        return max(lower_bound, min(upper_bound, value))

    def _distance_to_waypoint(self, waypoint: CraicWaypoint) -> Optional[float]:
        if self._current_pose_xy is None:
            return None
        return distance_xy(
            self._current_pose_xy[0],
            self._current_pose_xy[1],
            waypoint.x,
            waypoint.y,
        )

    def _rotate_to_yaw(
        self,
        target_yaw: float,
        yaw_tolerance_rad: float,
        max_angular_speed_radps: float,
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=1.0 / max(self.action_control_rate_hz, 1.0))
            if self._current_yaw is None:
                continue

            yaw_error = normalize_angle(target_yaw - self._current_yaw)
            if abs(yaw_error) <= yaw_tolerance_rad:
                self._publish_zero_velocity()
                return True

            angular_speed = self._clamp(
                abs(yaw_error) * self.action_heading_kp,
                self.action_min_angular_speed_radps,
                max_angular_speed_radps,
            )
            self._publish_velocity(0.0, math.copysign(angular_speed, yaw_error))

        self._publish_zero_velocity()
        return False

    def _creep_to_waypoint(
        self,
        waypoint: CraicWaypoint,
        position_tolerance_m: float,
        max_linear_speed_mps: float,
        max_angular_speed_radps: float,
        timeout_sec: float,
    ) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=1.0 / max(self.action_control_rate_hz, 1.0))
            if self._current_pose_xy is None or self._current_yaw is None:
                continue

            dx = waypoint.x - self._current_pose_xy[0]
            dy = waypoint.y - self._current_pose_xy[1]
            distance_error = math.hypot(dx, dy)
            if distance_error <= position_tolerance_m:
                self._publish_zero_velocity()
                return True

            target_heading = math.atan2(dy, dx)
            heading_error = normalize_angle(target_heading - self._current_yaw)
            if abs(heading_error) > self.action_heading_gate_rad:
                angular_speed = self._clamp(
                    abs(heading_error) * self.action_heading_kp,
                    self.action_min_angular_speed_radps,
                    max_angular_speed_radps,
                )
                self._publish_velocity(0.0, math.copysign(angular_speed, heading_error))
                continue

            linear_speed = self._clamp(
                distance_error * self.action_linear_kp,
                self.action_min_linear_speed_mps,
                max_linear_speed_mps,
            )
            angular_speed = self._clamp(
                heading_error * self.action_heading_kp,
                -max_angular_speed_radps,
                max_angular_speed_radps,
            )
            self._publish_velocity(linear_speed, angular_speed)

        self._publish_zero_velocity()
        return False

    def _execute_turn_action(
        self,
        waypoint: CraicWaypoint,
        plan: WaypointExecutionPlan,
    ) -> bool:
        distance_error = self._distance_to_waypoint(waypoint)
        if distance_error is None:
            return False

        if distance_error > self.special_action_entry_distance_m:
            self.get_logger().warn(
                f'{plan.profile_name} is still {distance_error:.2f} m away from the action waypoint; '
                'running a short low-speed creep before final heading alignment.'
            )
            if not self._creep_to_waypoint(
                waypoint,
                plan.position_tolerance_m or 0.35,
                min(plan.max_linear_speed_mps, self.turn_creep_max_linear_mps),
                plan.max_angular_speed_radps,
                timeout_sec=self.turn_action_timeout_sec,
            ):
                return False

        self._publish_zero_velocity()
        self._sleep_with_spin(plan.settle_time_sec)
        return self._rotate_to_yaw(
            waypoint.yaw,
            plan.yaw_tolerance_rad or 0.25,
            plan.max_angular_speed_radps,
            timeout_sec=self.turn_action_timeout_sec,
        )

    def _execute_park_action(
        self,
        waypoint: CraicWaypoint,
        plan: WaypointExecutionPlan,
    ) -> bool:
        if not self._creep_to_waypoint(
            waypoint,
            plan.position_tolerance_m or 0.18,
            plan.max_linear_speed_mps,
            min(plan.max_angular_speed_radps, self.park_final_align_max_angular_speed_radps),
            timeout_sec=self.park_action_timeout_sec,
        ):
            return False

        self._publish_zero_velocity()
        self._sleep_with_spin(plan.settle_time_sec)
        return self._rotate_to_yaw(
            waypoint.yaw,
            plan.yaw_tolerance_rad or 0.12,
            min(plan.max_angular_speed_radps, self.park_final_align_max_angular_speed_radps),
            timeout_sec=self.park_action_timeout_sec,
        )

    def _wait_for_bt_navigator(self) -> None:
        self.get_logger().info(
            'Waiting for bt_navigator to become active '
            f'(timeout: {self.nav2_activation_timeout_sec:.1f}s)...'
        )
        service_name = 'bt_navigator/get_state'
        state_client = self.create_client(GetState, service_name)
        deadline = time.monotonic() + self.nav2_activation_timeout_sec

        while time.monotonic() < deadline and not state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} service not available, waiting...')

        if not state_client.service_is_ready():
            raise RuntimeError(
                'Timed out waiting for bt_navigator/get_state. '
                'Nav2 is not active. Start your navigation launch first and '
                'check bt_navigator, planner_server, and controller_server logs.'
            )

        self._waitForNodeToActivate('bt_navigator')

    def _wait_for_runtime_readiness(self) -> None:
        self.get_logger().info(
            'Waiting for runtime readiness '
            f'(pose={self.require_pose_topic}, map={self.require_map_topic}, tf={self.require_tf_ready}, '
            f'timeout={self.startup_wait_timeout_sec:.1f}s)...'
        )
        deadline = time.monotonic() + self.startup_wait_timeout_sec
        last_status_log_time = 0.0

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

            pose_ready = (not self.require_pose_topic) or (self._current_pose_xy is not None)
            topic_names = {name for name, _types in self.get_topic_names_and_types()}
            map_ready = (not self.require_map_topic) or ('/map' in topic_names)
            tf_ready = True
            if self.require_tf_ready:
                try:
                    tf_ready = self._tf_buffer.can_transform(
                        self.global_frame,
                        self.robot_base_frame,
                        rclpy.time.Time(),
                    )
                except Exception:
                    tf_ready = False

            if pose_ready and map_ready and tf_ready:
                self.get_logger().info(
                    f'Runtime ready: pose topic "{self.pose_tracking_topic}", '
                    f'global frame "{self.global_frame}", base frame "{self.robot_base_frame}".'
                )
                return

            now = time.monotonic()
            if now - last_status_log_time >= 2.0:
                self.get_logger().info(
                    f'Runtime not ready yet: pose_ready={pose_ready}, map_ready={map_ready}, tf_ready={tf_ready}'
                )
                last_status_log_time = now

        raise RuntimeError(
            'Timed out waiting for runtime readiness. '
            f'pose_ready={self._current_pose_xy is not None}, '
            f'map_ready={"/map" in {name for name, _types in self.get_topic_names_and_types()}}, '
            f'tf_ready={self._tf_buffer.can_transform(self.global_frame, self.robot_base_frame, rclpy.time.Time()) if self.require_tf_ready else True}.'
        )

    def _make_double_parameter(self, name: str, value: float) -> Parameter:
        return Parameter(
            name=name,
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value),
            ),
        )

    def _set_controller_parameters(self, plan: WaypointExecutionPlan) -> None:
        if not self._controller_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'controller_server/set_parameters not available; skipping action profile tuning.'
            )
            return

        request = SetParameters.Request()
        request.parameters = [
            self._make_double_parameter('FollowPath.max_vel_x', plan.max_linear_speed_mps),
            self._make_double_parameter('FollowPath.max_vel_theta', plan.max_angular_speed_radps),
            self._make_double_parameter(
                'general_goal_checker.xy_goal_tolerance',
                plan.xy_goal_tolerance_m,
            ),
            self._make_double_parameter(
                'general_goal_checker.yaw_goal_tolerance',
                plan.yaw_goal_tolerance_rad,
            ),
        ]
        future = self._controller_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            self.get_logger().warn('Timed out applying controller profile parameters.')
            return

        failed_reasons = [result.reason for result in future.result().results if not result.successful]
        if failed_reasons:
            self.get_logger().warn(
                f'Controller profile update for {plan.profile_name} was only partially applied: '
                f'{"; ".join(failed_reasons)}'
            )

    def _send_plan(self, plan: WaypointExecutionPlan, reset_special_retry_count: bool = True) -> bool:
        if plan.start_index >= len(self._goal_poses):
            return False

        self._set_controller_parameters(plan)
        self._active_plan = plan
        self._active_slice_start = plan.start_index
        self._current_abs_waypoint_index = plan.start_index
        accepted = self.followWaypoints(self._goal_poses[plan.start_index : plan.end_index + 1])
        if accepted:
            self._following_waypoints = True
            self._last_progress_time = time.monotonic()
            self._last_progress_pose_xy = self._current_pose_xy
            if reset_special_retry_count:
                self._special_action_retry_count = 0
            self._log_current_waypoint(plan.start_index)
        return accepted

    def _send_remaining_waypoints(self, start_index: int) -> bool:
        return self._send_plan(build_execution_plan(self._waypoints, start_index, self._behavior_config))

    def _wait_for_task_exit(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not self.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _get_recovery_time_allowance(self) -> int:
        """Return an integer time allowance accepted by Nav2 APIs."""
        return max(1, math.ceil(self.recovery_time_allowance_sec))

    def _run_recovery_behavior(self) -> bool:
        if self._recovery_attempts >= self.max_recovery_attempts:
            self.get_logger().error('Reached max recovery attempts; aborting mission.')
            return False

        if self.spin_angle_rad <= 0.0 and self.backup_distance_m <= 0.0:
            self.get_logger().error(
                'Progress watchdog triggered, but both spin_angle_rad and backup_distance_m are disabled. '
                'Enable at least one recovery behavior or increase stuck_timeout_sec while testing.'
            )
            self.cancelTask()
            self._following_waypoints = False
            self._wait_for_task_exit(timeout_sec=2.0)
            self._publish_zero_velocity()
            return False

        self._recovery_attempts += 1
        self.get_logger().warn(
            f'Planner appears stuck; running recovery attempt #{self._recovery_attempts}.'
        )

        self.cancelTask()
        self._following_waypoints = False
        self._wait_for_task_exit(timeout_sec=3.0)
        self._publish_zero_velocity()

        if self.backup_distance_m > 0.0:
            self.get_logger().info(
                f'Backing up {self.backup_distance_m:.2f} m before retrying route.'
            )
            if self.backup(
                backup_dist=self.backup_distance_m,
                backup_speed=self.backup_speed_mps,
                time_allowance=self._get_recovery_time_allowance(),
            ):
                self._wait_for_task_exit(self.recovery_time_allowance_sec)

        if self.spin_angle_rad > 0.0:
            self.get_logger().info(
                f'Spinning {self.spin_angle_rad:.2f} rad to refresh obstacle observations.'
            )
            if self.spin(
                spin_dist=self.spin_angle_rad,
                time_allowance=self._get_recovery_time_allowance(),
            ):
                self._wait_for_task_exit(self.recovery_time_allowance_sec)

        try:
            self.clearAllCostmaps()
        except Exception as exc:  # pragma: no cover - depends on Nav2 runtime
            self.get_logger().warn(f'Costmap clear skipped: {exc}')

        if self._active_plan is None:
            return self._send_remaining_waypoints(self._current_abs_waypoint_index)
        return self._send_plan(build_resume_plan(self._active_plan, self._current_abs_waypoint_index))

    def _should_trigger_final_stop(self) -> bool:
        if self._current_pose_xy is None or not self._goal_poses or self._active_plan is None:
            return False

        stop_distance_m = self._active_plan.stop_distance_m
        if stop_distance_m is None:
            return False

        final_waypoint = self._waypoints[self._active_plan.goal_index]
        return (
            distance_xy(
                self._current_pose_xy[0],
                self._current_pose_xy[1],
                final_waypoint.x,
                final_waypoint.y,
            )
            <= stop_distance_m
        )

    def _is_active_special_action_satisfied(self) -> bool:
        if self._active_plan is None or not self._active_plan.is_special_action:
            return False
        if self._current_pose_xy is None or self._current_yaw is None:
            return False

        goal_waypoint = self._waypoints[self._active_plan.goal_index]
        position_error = distance_xy(
            self._current_pose_xy[0],
            self._current_pose_xy[1],
            goal_waypoint.x,
            goal_waypoint.y,
        )
        yaw_error = abs(normalize_angle(self._current_yaw - goal_waypoint.yaw))
        if self._active_plan.position_tolerance_m is not None:
            if position_error > self._active_plan.position_tolerance_m:
                return False
        if self._active_plan.yaw_tolerance_rad is not None:
            if yaw_error > self._active_plan.yaw_tolerance_rad:
                return False
        return True

    def _handle_special_action_completion(self) -> tuple[Optional[int], bool]:
        if self._active_plan is None or not self._active_plan.is_special_action:
            return None, False

        goal_waypoint = self._waypoints[self._active_plan.goal_index]
        action_success = False
        if self._active_plan.profile_name in {'turn_left', 'turn_right'}:
            action_success = self._execute_turn_action(goal_waypoint, self._active_plan)
        elif self._active_plan.profile_name == 'park':
            action_success = self._execute_park_action(goal_waypoint, self._active_plan)
        else:
            action_success = self._is_active_special_action_satisfied()

        if action_success and self._is_active_special_action_satisfied():
            if self._active_plan.settle_time_sec > 0.0:
                self.get_logger().info(
                    f'{self._active_plan.profile_name} goal satisfied; settling for '
                    f'{self._active_plan.settle_time_sec:.2f}s.'
                )
                time.sleep(self._active_plan.settle_time_sec)
            self._publish_zero_velocity()
            next_index = self._active_plan.goal_index + 1
            if next_index >= len(self._waypoints):
                self.get_logger().info('CRAIC mission completed successfully.')
                return 0, True
            if not self._send_remaining_waypoints(next_index):
                self.get_logger().error('Nav2 rejected the follow-up waypoint mission.')
                return 1, True
            return None, True

        self._special_action_retry_count += 1
        if self._special_action_retry_count > self._active_plan.goal_retry_limit:
            self.get_logger().error(
                f'{self._active_plan.profile_name} waypoint failed strict check at '
                f'x={goal_waypoint.x:.2f}, y={goal_waypoint.y:.2f}.'
            )
            return 1, True

        self.get_logger().warn(
            f'{self._active_plan.profile_name} waypoint needs a tighter retry '
            f'({self._special_action_retry_count}/{self._active_plan.goal_retry_limit}) '
            f'at x={goal_waypoint.x:.2f}, y={goal_waypoint.y:.2f}.'
        )
        if not self._send_plan(self._active_plan, reset_special_retry_count=False):
            self.get_logger().error('Nav2 rejected the strict action retry.')
            return 1, True
        return None, True

    def _update_feedback_state(self) -> None:
        if not self._following_waypoints:
            return

        feedback = self.getFeedback()
        if feedback is None:
            return

        if not hasattr(feedback, 'current_waypoint'):
            return

        relative_index = int(feedback.current_waypoint)
        absolute_index = min(
            self._active_slice_start + relative_index,
            len(self._waypoints) - 1,
        )
        self._current_abs_waypoint_index = absolute_index
        self._log_current_waypoint(absolute_index)

    def _update_progress_watchdog(self) -> bool:
        if self._current_pose_xy is None:
            return False

        if self._last_progress_pose_xy is None:
            self._last_progress_pose_xy = self._current_pose_xy
            self._last_progress_time = time.monotonic()
            return False

        moved = distance_xy(
            self._last_progress_pose_xy[0],
            self._last_progress_pose_xy[1],
            self._current_pose_xy[0],
            self._current_pose_xy[1],
        )
        if moved >= self.stuck_progress_radius_m:
            self._last_progress_pose_xy = self._current_pose_xy
            self._last_progress_time = time.monotonic()
            return False

        return (time.monotonic() - self._last_progress_time) >= self.stuck_timeout_sec

    def run(self) -> int:
        if self.road_network_file:
            self.get_logger().info(
                f'Planned {len(self._waypoints)} waypoint(s) from road network {self.road_network_file}'
            )
            if self._planned_route_name:
                self.get_logger().info(f'Planned route: {self._planned_route_name}')
            if self._route_node_ids:
                self.get_logger().info(
                    f'Route node sequence: {" -> ".join(self._route_node_ids)}'
                )
        else:
            self.get_logger().info(
                f'Loaded {len(self._waypoints)} CRAIC waypoint(s) from {self.waypoint_file}'
            )
        self._wait_for_bt_navigator()
        self._wait_for_runtime_readiness()

        if not self._send_remaining_waypoints(0):
            self.get_logger().error('Nav2 rejected the initial waypoint mission.')
            return 1

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=self.progress_check_period_sec)
            self._update_feedback_state()

            if self._should_trigger_final_stop():
                self.get_logger().info(
                    f'Within {self._active_plan.stop_distance_m:.2f} m of final goal; forcing stop.'
                )
                self.cancelTask()
                self._wait_for_task_exit(timeout_sec=2.0)
                self._publish_zero_velocity()
                return 0

            if self.isTaskComplete():
                result = self.getResult()
                self._following_waypoints = False
                self._publish_zero_velocity()
                if result == TaskResult.SUCCEEDED:
                    special_action_result, special_action_handled = (
                        self._handle_special_action_completion()
                    )
                    if special_action_result is not None:
                        return special_action_result
                    if special_action_handled:
                        continue
                    next_index = (
                        self._active_plan.goal_index + 1 if self._active_plan is not None else len(self._waypoints)
                    )
                    if next_index >= len(self._waypoints):
                        self.get_logger().info('CRAIC mission completed successfully.')
                        return 0
                    if not self._send_remaining_waypoints(next_index):
                        self.get_logger().error('Nav2 rejected the follow-up waypoint mission.')
                        return 1
                    continue
                if result == TaskResult.CANCELED:
                    self.get_logger().warn('CRAIC mission canceled.')
                    return 1
                self.get_logger().error(f'CRAIC mission failed with result: {result}')
                return 1

            if self._update_progress_watchdog():
                if not self._run_recovery_behavior():
                    self._publish_zero_velocity()
                    return 1

        self._publish_zero_velocity()
        return 1


def main() -> None:
    rclpy.init()
    navigator: Optional[CraicMissionCommander] = None
    exit_code = 1

    try:
        navigator = CraicMissionCommander()
        exit_code = navigator.run()
    except Exception as exc:
        if navigator is not None:
            navigator.get_logger().fatal(f'CRAIC mission crashed: {exc}')
        else:
            print(f'CRAIC mission crashed before node startup: {exc}', file=sys.stderr)
        exit_code = 1
    finally:
        if navigator is not None:
            navigator.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
