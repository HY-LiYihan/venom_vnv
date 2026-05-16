from typing import Any

from venom_mission_commander.mission_manager import MissionManager
from venom_mission_commander.startup_checks import StartupCheckResult


class MissionStatusReporter:
    def __init__(self, logger: Any):
        self.logger = logger

    def log_startup_check_results(self, results: list[StartupCheckResult]) -> None:
        for result in results:
            status = 'PASS' if result.success else 'FAIL'
            line = (
                f"[STARTUP] {status} {result.name}: "
                f"{result.message}"
            )
            if status == 'FAIL':
                self.logger.error(line)
            else:
                self.logger.info(line)

            warnings = result.data.get('warnings', [])
            for warning in warnings:
                self.logger.warning(f"[STARTUP] WARN {result.name}: {warning}")

    def log_snapshot(self, label: str, mission_manager: MissionManager) -> None:
        snapshot = self.build_snapshot(label, mission_manager)
        self.logger.info(
            '[STATUS] '
            f"{snapshot['label']}: "
            f"state={snapshot['state']} "
            f"phase={snapshot['phase']} "
            f"waypoint={snapshot['waypoint']} "
            f"task={snapshot['task']} "
            f"nav={snapshot['navigation']} "
            f"last_task={snapshot['last_task']} "
            f"startup_checks={snapshot['startup_checks']}"
        )

    def build_snapshot(self, label: str, mission_manager: MissionManager) -> dict[str, str]:
        state_data = mission_manager.restore_state()
        return {
            'label': label,
            'state': mission_manager.state.value,
            'phase': str(state_data.get('phase', 'unknown')),
            'waypoint': self._format_waypoint(state_data),
            'task': self._format_task(state_data),
            'navigation': self._format_navigation(state_data),
            'last_task': self._format_last_task(state_data),
            'startup_checks': str(state_data.get('startup_checks_status', 'not_run')),
        }

    def _format_waypoint(self, state_data: dict[str, Any]) -> str:
        index = state_data.get('current_waypoint_index')
        total = state_data.get('total_waypoints')
        name = state_data.get('current_waypoint_name')
        kind = state_data.get('current_waypoint_kind')
        if name is None:
            return '-'

        if index is not None and total:
            prefix = f'{int(index) + 1}/{total}'
        else:
            prefix = '?'
        return f'{prefix}:{name}({kind or "unknown"})'

    def _format_task(self, state_data: dict[str, Any]) -> str:
        index = state_data.get('current_task_index')
        name = state_data.get('current_task_name')
        if name is None:
            return '-'
        if index is None:
            return str(name)
        return f'{int(index) + 1}:{name}'

    def _format_navigation(self, state_data: dict[str, Any]) -> str:
        attempt = state_data.get('navigation_attempt')
        attempts = state_data.get('navigation_attempts')
        timeout_sec = state_data.get('navigation_timeout_sec')
        last_success = state_data.get('last_navigation_success')
        if attempt is None:
            return '-'

        timeout_text = 'none' if timeout_sec is None else f'{float(timeout_sec):.1f}s'
        return f'attempt={attempt}/{attempts} timeout={timeout_text} last_success={last_success}'

    def _format_last_task(self, state_data: dict[str, Any]) -> str:
        name = state_data.get('last_task_name')
        if name is None:
            return '-'

        success = state_data.get('last_task_success')
        message = state_data.get('last_task_message', '')
        return f'{name}:{success}:{message}'
