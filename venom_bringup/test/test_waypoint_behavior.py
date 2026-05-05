from venom_bringup.craic_waypoint_utils import CraicWaypoint
from venom_bringup.waypoint_behavior import (
    WaypointBehaviorConfig,
    build_execution_plan,
    build_resume_plan,
)


def make_waypoint(index: int, action: int, x_value: float) -> CraicWaypoint:
    return CraicWaypoint(
        index=index,
        x=x_value,
        y=0.0,
        yaw=0.0,
        action=action,
        source_a=x_value,
        source_b=0.0,
        action_label=str(action),
    )


def test_default_plan_groups_until_next_special_action():
    config = WaypointBehaviorConfig(default_final_stop_distance_m=1.0)
    waypoints = [
        make_waypoint(0, 1, 0.0),
        make_waypoint(1, 1, 1.0),
        make_waypoint(2, 3, 2.0),
        make_waypoint(3, 1, 3.0),
    ]

    plan = build_execution_plan(waypoints, 0, config)

    assert plan.profile_name == 'default'
    assert plan.start_index == 0
    assert plan.end_index == 1
    assert plan.goal_index == 1
    assert plan.max_linear_speed_mps == config.cruise_max_linear_speed_mps
    assert plan.xy_goal_tolerance_m == config.cruise_xy_goal_tolerance_m


def test_left_turn_plan_is_single_waypoint_with_tighter_profile():
    config = WaypointBehaviorConfig(default_final_stop_distance_m=1.0)
    waypoints = [
        make_waypoint(0, 3, 0.0),
        make_waypoint(1, 1, 1.0),
    ]

    plan = build_execution_plan(waypoints, 0, config)

    assert plan.profile_name == 'turn_left'
    assert plan.start_index == 0
    assert plan.end_index == 0
    assert plan.goal_index == 0
    assert plan.max_linear_speed_mps == config.left_turn_max_linear_speed_mps
    assert plan.xy_goal_tolerance_m == config.left_turn_position_tolerance_m
    assert plan.yaw_goal_tolerance_rad == config.left_turn_yaw_tolerance_rad


def test_right_turn_plan_uses_right_turn_profile():
    config = WaypointBehaviorConfig(default_final_stop_distance_m=1.0)
    waypoints = [make_waypoint(0, 2, 0.0)]

    plan = build_execution_plan(waypoints, 0, config)

    assert plan.profile_name == 'turn_right'
    assert plan.max_linear_speed_mps == config.right_turn_max_linear_speed_mps
    assert plan.xy_goal_tolerance_m == config.right_turn_position_tolerance_m
    assert plan.yaw_goal_tolerance_rad == config.right_turn_yaw_tolerance_rad


def test_park_plan_uses_parking_profile():
    config = WaypointBehaviorConfig(default_final_stop_distance_m=1.0)
    waypoints = [make_waypoint(0, 8, 0.0)]

    plan = build_execution_plan(waypoints, 0, config)

    assert plan.profile_name == 'park'
    assert plan.max_linear_speed_mps == config.park_max_linear_speed_mps
    assert plan.xy_goal_tolerance_m == config.park_position_tolerance_m
    assert plan.yaw_goal_tolerance_rad == config.park_yaw_tolerance_rad


def test_last_default_segment_keeps_final_stop_distance():
    config = WaypointBehaviorConfig(default_final_stop_distance_m=1.5)
    waypoints = [
        make_waypoint(0, 1, 0.0),
        make_waypoint(1, 1, 1.0),
    ]

    plan = build_execution_plan(waypoints, 0, config)

    assert plan.profile_name == 'default'
    assert plan.end_index == 1
    assert plan.stop_distance_m == 1.5


def test_resume_plan_advances_inside_default_slice():
    config = WaypointBehaviorConfig(default_final_stop_distance_m=1.0)
    waypoints = [
        make_waypoint(0, 1, 0.0),
        make_waypoint(1, 1, 1.0),
        make_waypoint(2, 1, 2.0),
    ]

    plan = build_execution_plan(waypoints, 0, config)
    resumed = build_resume_plan(plan, 1)

    assert resumed.start_index == 1
    assert resumed.end_index == 2
    assert resumed.goal_index == 2


def test_resume_plan_keeps_special_action_singleton():
    config = WaypointBehaviorConfig(default_final_stop_distance_m=1.0)
    plan = build_execution_plan([make_waypoint(0, 8, 0.0)], 0, config)

    resumed = build_resume_plan(plan, 0)

    assert resumed == plan


def test_special_action_plan_remains_singleton_for_retry():
    config = WaypointBehaviorConfig(default_final_stop_distance_m=1.0)
    waypoints = [
        make_waypoint(0, 3, 3.0),
        make_waypoint(1, 2, 5.0),
        make_waypoint(2, 1, 8.0),
    ]

    turn_left_plan = build_execution_plan(waypoints, 0, config)
    retried_turn_left_plan = build_resume_plan(turn_left_plan, 0)
    next_plan = build_execution_plan(waypoints, 1, config)

    assert retried_turn_left_plan.start_index == 0
    assert retried_turn_left_plan.goal_index == 0
    assert retried_turn_left_plan.profile_name == 'turn_left'
    assert next_plan.start_index == 1
    assert next_plan.profile_name == 'turn_right'
