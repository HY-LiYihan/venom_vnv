from typing import Any

from venom_mission_commander.mission_manager import MissionManager
from venom_mission_commander.startup_checks import StartupCheckResult


class MissionStatusReporter:
    def __init__(self, logger: Any):
        self.logger = logger

    def log_startup_check_results(self, results: list[StartupCheckResult]) -> None:
        for result in results:
            status = 'PASS' if result.success else 'FAIL'
            line = '\n'.join([
                f'[STARTUP] {status} {result.name}',
                f'\tmessage: {result.message}',
            ])
            if status == 'FAIL':
                self.logger.error(line)
            else:
                self.logger.info(line)

            warnings = result.data.get('warnings', [])
            for warning in warnings:
                self.logger.warning('\n'.join([
                    f'[STARTUP] WARN {result.name}',
                    f'\tmessage: {warning}',
                ]))

    def log_snapshot(self, label: str, mission_manager: MissionManager) -> None:
        snapshot = self.build_snapshot(label, mission_manager)
        self.logger.info('\n'.join([
            f"[STATUS] {snapshot['label']}",
            f"\tstate: {snapshot['state']} | phase: {snapshot['phase']} | "
            f"startup_checks: {snapshot['startup_checks']}",
            f"\twaypoint: {snapshot['waypoint']}",
            f"\ttask: {snapshot['task']}",
            f"\tnav: {snapshot['navigation']}",
            f"\tlast_task: {snapshot['last_task']}",
        ]))

    def log_final_summary(self, mission_manager: MissionManager) -> None:
        if mission_manager.mission_id is None:
            self.logger.info('\n'.join([
                '[SUMMARY] mission',
                '\tstatus: not_configured',
                f'\tstate: {mission_manager.state.value}',
            ]))
            return

        state_data = mission_manager.restore_state()
        completed = state_data.get('completed_waypoint_count', 0)
        total = state_data.get('total_waypoints', '-')
        lines = [
            '[SUMMARY] mission',
            f'\tid: {mission_manager.mission_id or "-"}',
            f'\tstate: {mission_manager.state.value}',
            f'\tphase: {state_data.get("phase", "unknown")}',
            f'\tprogress: {completed}/{total} waypoint(s)',
            f'\tstartup_checks: {state_data.get("startup_checks_status", "not_run")}',
            f'\tlast_waypoint: {state_data.get("last_completed_waypoint", "-")}',
            f'\tlast_task: {self._format_last_task(state_data)}',
            f'\tnavigation: {self._format_final_navigation(state_data)}',
        ]

        failure_reason = state_data.get('failure_reason')
        if failure_reason:
            lines.append(f'\tfailure_reason: {failure_reason}')

        startup_failures = state_data.get('startup_check_failures', [])
        if startup_failures:
            lines.append(f'\tstartup_check_failures: {", ".join(startup_failures)}')

        failed_tasks = state_data.get('failed_tasks', [])
        if failed_tasks:
            lines.append('\tfailed_tasks:')
            for task in failed_tasks:
                lines.append(
                    f"\t\t- {task.get('task_name', '-')} "
                    f"({task.get('task_type', '-')}) | {task.get('message', '')}"
                )

        self.logger.info('\n'.join(lines))

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

    def _format_final_navigation(self, state_data: dict[str, Any]) -> str:
        waypoint = state_data.get('last_navigation_waypoint')
        if waypoint is None:
            return '-'

        attempt = state_data.get('last_navigation_attempt')
        success = state_data.get('last_navigation_success')
        cancel_confirmed = state_data.get('last_navigation_cancel_confirmed')
        recovered = state_data.get('last_navigation_recovery_performed')
        parts = [
            f'waypoint={waypoint}',
            f'attempt={attempt}',
            f'success={success}',
        ]
        if cancel_confirmed is not None:
            parts.append(f'cancel_confirmed={cancel_confirmed}')
        if recovered is not None:
            parts.append(f'recovered={recovered}')
        return ' | '.join(parts)
