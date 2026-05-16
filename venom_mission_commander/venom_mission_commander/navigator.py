import math
import time
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter

from venom_mission_commander.models import WaypointSpec


class MockWaypointNavigator:
    def __init__(self, node: Any, delay_sec: float = 0.5):
        self.node = node
        self.delay_sec = delay_sec
        self._ready = False

    def wait_until_ready(self, timeout_sec: float | None = None) -> bool:
        if self._ready:
            return True
        self.node.get_logger().info('Mock navigator is ready; no Nav2 server required.')
        self._ready = True
        return True

    def is_ready(self) -> bool:
        return self._ready

    def go_to_waypoint(self, waypoint: WaypointSpec) -> None:
        self.node.get_logger().info(
            f'[MOCK NAV] Going to {waypoint.name}: '
            f'x={waypoint.x:.2f}, y={waypoint.y:.2f}, yaw={waypoint.yaw:.3f}'
        )

    def wait_until_done(self, timeout_sec: float | None = None) -> bool:
        delay_sec = max(self.delay_sec, 0.0)
        if timeout_sec is not None and delay_sec > timeout_sec:
            time.sleep(max(timeout_sec, 0.0))
            self.node.get_logger().error(
                f'[MOCK NAV] Navigation timeout after {timeout_sec:.1f}s.'
            )
            return False

        time.sleep(delay_sec)
        self.node.get_logger().info('[MOCK NAV] Navigation succeeded.')
        return True

    def cancel(self, timeout_sec: float = 2.0) -> bool:
        self.node.get_logger().warn('[MOCK NAV] Cancel requested.')
        return True

    def recover(self) -> bool:
        self.node.get_logger().warn('[MOCK NAV] Recovery requested.')
        return True

    def shutdown(self) -> None:
        return None


class Nav2WaypointNavigator:
    def __init__(self, node: Any, wait_mode: str = 'bt_navigator', use_sim_time: bool = False):
        from nav2_simple_commander.robot_navigator import BasicNavigator

        self.node = node
        self.wait_mode = wait_mode
        self.navigator = BasicNavigator(node_name='mission_commander_nav2')
        self._ready = False
        self._configure_use_sim_time(use_sim_time)

    def wait_until_ready(self, timeout_sec: float | None = None) -> bool:
        if self._ready:
            return True
        self.node.get_logger().info(f'Waiting for Nav2 with mode: {self.wait_mode}')
        if timeout_sec is not None:
            return self._wait_until_ready_with_timeout(timeout_sec)

        if self.wait_mode == 'full':
            self.navigator.waitUntilNav2Active()
            self._ready = True
            return True

        if hasattr(self.navigator, '_waitForNodeToActivate'):
            self.navigator._waitForNodeToActivate('bt_navigator')
            self._ready = True
            return True

        self.navigator.waitUntilNav2Active()
        self._ready = True
        return True

    def is_ready(self) -> bool:
        return self._ready

    def _configure_use_sim_time(self, use_sim_time: bool) -> None:
        if self.navigator.has_parameter('use_sim_time'):
            self.navigator.set_parameters([
                Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time),
            ])
            return

        self.navigator.declare_parameter('use_sim_time', use_sim_time)

    def go_to_waypoint(self, waypoint: WaypointSpec) -> None:
        pose = self.waypoint_to_pose(waypoint)
        self.node.get_logger().info(
            f'[NAV2] Going to {waypoint.name}: '
            f'x={waypoint.x:.2f}, y={waypoint.y:.2f}, yaw={waypoint.yaw:.3f}'
        )
        self.navigator.goToPose(pose)

    def wait_until_done(self, timeout_sec: float | None = None) -> bool:
        from nav2_simple_commander.robot_navigator import TaskResult as Nav2TaskResult

        started_at = time.monotonic()
        poll_count = 0
        while not self.navigator.isTaskComplete():
            poll_count += 1
            elapsed_sec = time.monotonic() - started_at
            if timeout_sec is not None and elapsed_sec > timeout_sec:
                self.node.get_logger().error(
                    f'[NAV2] Navigation timeout after {timeout_sec:.1f}s.'
                )
                return False

            feedback = self.navigator.getFeedback()
            if feedback is not None and poll_count % 10 == 0:
                self.node.get_logger().info(
                    f'[NAV2] Still navigating: {self._format_feedback(feedback, elapsed_sec)}'
                )

        result = self.navigator.getResult()
        if result == Nav2TaskResult.SUCCEEDED:
            self.node.get_logger().info('[NAV2] Navigation succeeded.')
            return True

        self.node.get_logger().error(f'[NAV2] Navigation failed: {result}')
        return False

    def waypoint_to_pose(self, waypoint: WaypointSpec) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = waypoint.frame_id
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        return pose

    def cancel(self, timeout_sec: float = 2.0) -> bool:
        self.navigator.cancelTask()
        return self._wait_for_task_done(timeout_sec)

    def recover(self) -> bool:
        if not hasattr(self.navigator, 'clearAllCostmaps'):
            self.node.get_logger().warning('[NAV2] Recovery skipped; clearAllCostmaps unavailable.')
            return False

        self.node.get_logger().warning('[NAV2] Clearing costmaps before navigation retry.')
        try:
            self.navigator.clearAllCostmaps()
        except Exception as exc:
            self.node.get_logger().error(f'[NAV2] Costmap recovery failed: {exc}')
            return False

        return True

    def shutdown(self) -> None:
        self.navigator.destroy_node()

    def _wait_for_task_done(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while time.monotonic() <= deadline:
            if self.navigator.isTaskComplete():
                self.node.get_logger().warning('[NAV2] Current task is canceled or complete.')
                return True

        self.node.get_logger().warning(
            f'[NAV2] Timed out waiting {timeout_sec:.1f}s for task cancellation.'
        )
        return False

    def _wait_until_ready_with_timeout(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        if self.wait_mode == 'full':
            if not self._wait_for_lifecycle_node_active('amcl', deadline):
                self.node.get_logger().error(
                    f'[NAV2] amcl did not become active within {timeout_sec:.1f}s.'
                )
                return False
            if not self._wait_for_initial_pose(deadline):
                self.node.get_logger().error(
                    f'[NAV2] initial pose was not received within {timeout_sec:.1f}s.'
                )
                return False

        if not self._wait_for_lifecycle_node_active('bt_navigator', deadline):
            self.node.get_logger().error(
                f'[NAV2] bt_navigator did not become active within {timeout_sec:.1f}s.'
            )
            return False

        self.node.get_logger().info('[NAV2] Nav2 is ready for use.')
        self._ready = True
        return True

    def _wait_for_initial_pose(self, deadline: float) -> bool:
        if not hasattr(self.navigator, 'initial_pose_received'):
            return True

        while time.monotonic() < deadline:
            if self.navigator.initial_pose_received:
                return True

            if hasattr(self.navigator, '_setInitialPose'):
                self.node.get_logger().info('[NAV2] Setting initial pose and waiting for amcl_pose...')
                self.navigator._setInitialPose()

            remaining_sec = max(deadline - time.monotonic(), 0.0)
            rclpy.spin_once(self.navigator, timeout_sec=min(1.0, remaining_sec))

        return bool(self.navigator.initial_pose_received)

    def _wait_for_lifecycle_node_active(self, node_name: str, deadline: float) -> bool:
        from lifecycle_msgs.srv import GetState

        node_service = f'{node_name}/get_state'
        state_client = self.navigator.create_client(GetState, node_service)

        while time.monotonic() < deadline:
            remaining_sec = max(deadline - time.monotonic(), 0.0)
            if state_client.wait_for_service(timeout_sec=min(0.5, remaining_sec)):
                break
            self.node.get_logger().info(f'[NAV2] {node_service} service not available, waiting...')
        else:
            return False

        req = GetState.Request()
        while time.monotonic() < deadline:
            future = state_client.call_async(req)
            remaining_sec = max(deadline - time.monotonic(), 0.0)
            rclpy.spin_until_future_complete(
                self.navigator,
                future,
                timeout_sec=min(0.5, remaining_sec),
            )
            if future.done() and future.result() is not None:
                state = future.result().current_state.label
                if state == 'active':
                    return True
                self.node.get_logger().info(f'[NAV2] {node_name} state={state}; waiting...')
            time.sleep(0.2)

        return False

    def _format_feedback(self, feedback: Any, elapsed_sec: float) -> str:
        parts = [f'elapsed={elapsed_sec:.1f}s']
        nav_time_sec = self._duration_to_seconds(getattr(feedback, 'navigation_time', None))
        eta_sec = self._duration_to_seconds(getattr(feedback, 'estimated_time_remaining', None))

        if nav_time_sec is not None:
            parts.append(f'nav_time={nav_time_sec:.1f}s')
        if eta_sec is not None:
            parts.append(f'eta={eta_sec:.1f}s')

        current_waypoint = getattr(feedback, 'current_waypoint', None)
        if current_waypoint is not None:
            parts.append(f'current_waypoint={current_waypoint}')

        return ', '.join(parts)

    def _duration_to_seconds(self, duration: Any) -> float | None:
        if duration is None:
            return None

        if hasattr(duration, 'nanoseconds'):
            return float(duration.nanoseconds) / 1_000_000_000.0

        sec = getattr(duration, 'sec', None)
        nanosec = getattr(duration, 'nanosec', None)
        if sec is None or nanosec is None:
            return None

        return float(sec) + float(nanosec) / 1_000_000_000.0
