"""Action-aware waypoint execution helpers."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Sequence

from venom_bringup.craic_waypoint_utils import CraicWaypoint


TURN_RIGHT_ACTION = 2
TURN_LEFT_ACTION = 3
LANE_CHANGE_LEFT_ACTION = 4
LANE_CHANGE_RIGHT_ACTION = 5
OVERTAKE_ACTION = 6
U_TURN_ACTION = 7
PARK_ACTION = 8
SPECIAL_ACTIONS = {
    TURN_RIGHT_ACTION,
    TURN_LEFT_ACTION,
    LANE_CHANGE_LEFT_ACTION,
    LANE_CHANGE_RIGHT_ACTION,
    OVERTAKE_ACTION,
    U_TURN_ACTION,
    PARK_ACTION,
}


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x_value: float, y_value: float, z_value: float, w_value: float) -> float:
    """Return planar yaw from a quaternion."""
    siny_cosp = 2.0 * (w_value * z_value + x_value * y_value)
    cosy_cosp = 1.0 - 2.0 * (y_value * y_value + z_value * z_value)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass(frozen=True)
class WaypointBehaviorConfig:
    default_final_stop_distance_m: float
    cruise_max_linear_speed_mps: float = 2.0
    cruise_max_speed_xy_mps: float = 2.0
    cruise_max_angular_speed_radps: float = 1.6
    cruise_xy_goal_tolerance_m: float = 0.5
    cruise_yaw_goal_tolerance_rad: float = 0.4
    left_turn_max_linear_speed_mps: float = 0.8
    left_turn_max_speed_xy_mps: float = 0.8
    left_turn_max_angular_speed_radps: float = 0.9
    left_turn_position_tolerance_m: float = 0.45
    left_turn_yaw_tolerance_rad: float = 0.22
    left_turn_settle_time_sec: float = 0.35
    right_turn_max_linear_speed_mps: float = 0.65
    right_turn_max_speed_xy_mps: float = 0.65
    right_turn_max_angular_speed_radps: float = 0.8
    right_turn_position_tolerance_m: float = 0.35
    right_turn_yaw_tolerance_rad: float = 0.30
    right_turn_settle_time_sec: float = 0.20
    lane_change_left_max_linear_speed_mps: float = 0.95
    lane_change_left_max_speed_xy_mps: float = 0.95
    lane_change_left_max_angular_speed_radps: float = 0.75
    lane_change_left_position_tolerance_m: float = 0.28
    lane_change_left_yaw_tolerance_rad: float = 0.20
    lane_change_left_settle_time_sec: float = 0.25
    lane_change_right_max_linear_speed_mps: float = 0.90
    lane_change_right_max_speed_xy_mps: float = 0.90
    lane_change_right_max_angular_speed_radps: float = 0.70
    lane_change_right_position_tolerance_m: float = 0.28
    lane_change_right_yaw_tolerance_rad: float = 0.20
    lane_change_right_settle_time_sec: float = 0.25
    overtake_max_linear_speed_mps: float = 1.15
    overtake_max_speed_xy_mps: float = 1.15
    overtake_max_angular_speed_radps: float = 0.75
    overtake_position_tolerance_m: float = 0.40
    overtake_yaw_tolerance_rad: float = 0.28
    overtake_settle_time_sec: float = 0.15
    u_turn_max_linear_speed_mps: float = 0.45
    u_turn_max_speed_xy_mps: float = 0.45
    u_turn_max_angular_speed_radps: float = 0.70
    u_turn_position_tolerance_m: float = 0.25
    u_turn_yaw_tolerance_rad: float = 0.16
    u_turn_settle_time_sec: float = 0.50
    park_max_linear_speed_mps: float = 0.35
    park_max_speed_xy_mps: float = 0.35
    park_max_angular_speed_radps: float = 0.45
    park_position_tolerance_m: float = 0.18
    park_yaw_tolerance_rad: float = 0.12
    park_settle_time_sec: float = 1.0
    special_action_retry_limit: int = 2


@dataclass(frozen=True)
class WaypointExecutionPlan:
    profile_name: str
    start_index: int
    end_index: int
    goal_index: int
    max_linear_speed_mps: float
    max_speed_xy_mps: float
    max_angular_speed_radps: float
    xy_goal_tolerance_m: float
    yaw_goal_tolerance_rad: float
    stop_distance_m: float | None = None
    position_tolerance_m: float | None = None
    yaw_tolerance_rad: float | None = None
    settle_time_sec: float = 0.0
    goal_retry_limit: int = 0

    @property
    def is_special_action(self) -> bool:
        return self.profile_name != 'default'


def build_execution_plan(
    waypoints: Sequence[CraicWaypoint],
    start_index: int,
    config: WaypointBehaviorConfig,
) -> WaypointExecutionPlan:
    """Choose how the next mission slice should be executed."""
    waypoint = waypoints[start_index]
    if waypoint.action == TURN_LEFT_ACTION:
        return WaypointExecutionPlan(
            profile_name='turn_left',
            start_index=start_index,
            end_index=start_index,
            goal_index=start_index,
            max_linear_speed_mps=config.left_turn_max_linear_speed_mps,
            max_speed_xy_mps=config.left_turn_max_speed_xy_mps,
            max_angular_speed_radps=config.left_turn_max_angular_speed_radps,
            xy_goal_tolerance_m=config.left_turn_position_tolerance_m,
            yaw_goal_tolerance_rad=config.left_turn_yaw_tolerance_rad,
            position_tolerance_m=config.left_turn_position_tolerance_m,
            yaw_tolerance_rad=config.left_turn_yaw_tolerance_rad,
            settle_time_sec=config.left_turn_settle_time_sec,
            goal_retry_limit=config.special_action_retry_limit,
        )
    if waypoint.action == TURN_RIGHT_ACTION:
        return WaypointExecutionPlan(
            profile_name='turn_right',
            start_index=start_index,
            end_index=start_index,
            goal_index=start_index,
            max_linear_speed_mps=config.right_turn_max_linear_speed_mps,
            max_speed_xy_mps=config.right_turn_max_speed_xy_mps,
            max_angular_speed_radps=config.right_turn_max_angular_speed_radps,
            xy_goal_tolerance_m=config.right_turn_position_tolerance_m,
            yaw_goal_tolerance_rad=config.right_turn_yaw_tolerance_rad,
            position_tolerance_m=config.right_turn_position_tolerance_m,
            yaw_tolerance_rad=config.right_turn_yaw_tolerance_rad,
            settle_time_sec=config.right_turn_settle_time_sec,
            goal_retry_limit=config.special_action_retry_limit,
        )
    if waypoint.action == LANE_CHANGE_LEFT_ACTION:
        return WaypointExecutionPlan(
            profile_name='lane_change_left',
            start_index=start_index,
            end_index=start_index,
            goal_index=start_index,
            max_linear_speed_mps=config.lane_change_left_max_linear_speed_mps,
            max_speed_xy_mps=config.lane_change_left_max_speed_xy_mps,
            max_angular_speed_radps=config.lane_change_left_max_angular_speed_radps,
            xy_goal_tolerance_m=config.lane_change_left_position_tolerance_m,
            yaw_goal_tolerance_rad=config.lane_change_left_yaw_tolerance_rad,
            position_tolerance_m=config.lane_change_left_position_tolerance_m,
            yaw_tolerance_rad=config.lane_change_left_yaw_tolerance_rad,
            settle_time_sec=config.lane_change_left_settle_time_sec,
            goal_retry_limit=config.special_action_retry_limit,
        )
    if waypoint.action == LANE_CHANGE_RIGHT_ACTION:
        return WaypointExecutionPlan(
            profile_name='lane_change_right',
            start_index=start_index,
            end_index=start_index,
            goal_index=start_index,
            max_linear_speed_mps=config.lane_change_right_max_linear_speed_mps,
            max_speed_xy_mps=config.lane_change_right_max_speed_xy_mps,
            max_angular_speed_radps=config.lane_change_right_max_angular_speed_radps,
            xy_goal_tolerance_m=config.lane_change_right_position_tolerance_m,
            yaw_goal_tolerance_rad=config.lane_change_right_yaw_tolerance_rad,
            position_tolerance_m=config.lane_change_right_position_tolerance_m,
            yaw_tolerance_rad=config.lane_change_right_yaw_tolerance_rad,
            settle_time_sec=config.lane_change_right_settle_time_sec,
            goal_retry_limit=config.special_action_retry_limit,
        )
    if waypoint.action == OVERTAKE_ACTION:
        return WaypointExecutionPlan(
            profile_name='overtake',
            start_index=start_index,
            end_index=start_index,
            goal_index=start_index,
            max_linear_speed_mps=config.overtake_max_linear_speed_mps,
            max_speed_xy_mps=config.overtake_max_speed_xy_mps,
            max_angular_speed_radps=config.overtake_max_angular_speed_radps,
            xy_goal_tolerance_m=config.overtake_position_tolerance_m,
            yaw_goal_tolerance_rad=config.overtake_yaw_tolerance_rad,
            position_tolerance_m=config.overtake_position_tolerance_m,
            yaw_tolerance_rad=config.overtake_yaw_tolerance_rad,
            settle_time_sec=config.overtake_settle_time_sec,
            goal_retry_limit=config.special_action_retry_limit,
        )
    if waypoint.action == U_TURN_ACTION:
        return WaypointExecutionPlan(
            profile_name='u_turn',
            start_index=start_index,
            end_index=start_index,
            goal_index=start_index,
            max_linear_speed_mps=config.u_turn_max_linear_speed_mps,
            max_speed_xy_mps=config.u_turn_max_speed_xy_mps,
            max_angular_speed_radps=config.u_turn_max_angular_speed_radps,
            xy_goal_tolerance_m=config.u_turn_position_tolerance_m,
            yaw_goal_tolerance_rad=config.u_turn_yaw_tolerance_rad,
            position_tolerance_m=config.u_turn_position_tolerance_m,
            yaw_tolerance_rad=config.u_turn_yaw_tolerance_rad,
            settle_time_sec=config.u_turn_settle_time_sec,
            goal_retry_limit=config.special_action_retry_limit,
        )
    if waypoint.action == PARK_ACTION:
        return WaypointExecutionPlan(
            profile_name='park',
            start_index=start_index,
            end_index=start_index,
            goal_index=start_index,
            max_linear_speed_mps=config.park_max_linear_speed_mps,
            max_speed_xy_mps=config.park_max_speed_xy_mps,
            max_angular_speed_radps=config.park_max_angular_speed_radps,
            xy_goal_tolerance_m=config.park_position_tolerance_m,
            yaw_goal_tolerance_rad=config.park_yaw_tolerance_rad,
            position_tolerance_m=config.park_position_tolerance_m,
            yaw_tolerance_rad=config.park_yaw_tolerance_rad,
            settle_time_sec=config.park_settle_time_sec,
            goal_retry_limit=config.special_action_retry_limit,
        )

    end_index = start_index
    while end_index + 1 < len(waypoints):
        next_action = waypoints[end_index + 1].action
        if next_action in SPECIAL_ACTIONS:
            break
        end_index += 1

    stop_distance_m = None
    if end_index == len(waypoints) - 1:
        stop_distance_m = config.default_final_stop_distance_m

    return WaypointExecutionPlan(
        profile_name='default',
        start_index=start_index,
        end_index=end_index,
        goal_index=end_index,
        max_linear_speed_mps=config.cruise_max_linear_speed_mps,
        max_speed_xy_mps=config.cruise_max_speed_xy_mps,
        max_angular_speed_radps=config.cruise_max_angular_speed_radps,
        xy_goal_tolerance_m=config.cruise_xy_goal_tolerance_m,
        yaw_goal_tolerance_rad=config.cruise_yaw_goal_tolerance_rad,
        stop_distance_m=stop_distance_m,
    )


def build_resume_plan(
    active_plan: WaypointExecutionPlan,
    current_waypoint_index: int,
) -> WaypointExecutionPlan:
    """Resume a partially executed plan after recovery."""
    if active_plan.is_special_action:
        return active_plan

    resume_start = min(max(current_waypoint_index, active_plan.start_index), active_plan.end_index)
    return replace(active_plan, start_index=resume_start)
