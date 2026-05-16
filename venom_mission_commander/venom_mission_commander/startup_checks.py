import math
from dataclasses import dataclass, field
from typing import Any

from venom_mission_commander.models import MissionConfig
from venom_mission_commander.task_plugins import TaskPluginRegistry


@dataclass(frozen=True)
class StartupCheckResult:
    name: str
    success: bool
    message: str
    data: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            'name': self.name,
            'success': self.success,
            'message': self.message,
            'data': dict(self.data),
        }


class StartupChecker:
    def __init__(
        self,
        mission_config: MissionConfig,
        registry: TaskPluginRegistry,
        navigator: Any,
        navigator_ready_timeout_sec: float,
    ):
        self.mission_config = mission_config
        self.registry = registry
        self.navigator = navigator
        self.navigator_ready_timeout_sec = navigator_ready_timeout_sec

    def run(self) -> list[StartupCheckResult]:
        results = [
            self.check_mission_config(),
            self.check_task_plugins_registered(),
        ]
        if all(result.success for result in results):
            results.append(self.check_navigator_ready())
        return results

    def check_mission_config(self) -> StartupCheckResult:
        errors: list[str] = []
        warnings: list[str] = []
        waypoint_names: set[str] = set()

        if not self.mission_config.waypoints:
            errors.append('mission has no waypoints')

        for waypoint in self.mission_config.waypoints:
            if waypoint.name in waypoint_names:
                errors.append(f'duplicate waypoint name: {waypoint.name}')
            waypoint_names.add(waypoint.name)

            for field_name in ('x', 'y', 'yaw'):
                if not math.isfinite(float(getattr(waypoint, field_name))):
                    errors.append(f'waypoint {waypoint.name} has non-finite {field_name}')

            task_names: set[str] = set()
            for task in waypoint.tasks:
                if task.name in task_names:
                    warnings.append(
                        f'waypoint {waypoint.name} has duplicate task name: {task.name}'
                    )
                task_names.add(task.name)

        return StartupCheckResult(
            name='mission_config',
            success=not errors,
            message='mission config is valid' if not errors else '; '.join(errors),
            data={
                'waypoint_count': len(self.mission_config.waypoints),
                'warnings': warnings,
            },
        )

    def check_task_plugins_registered(self) -> StartupCheckResult:
        required_types = sorted({
            task.task_type
            for waypoint in self.mission_config.waypoints
            for task in waypoint.tasks
        })
        missing_types = [
            task_type
            for task_type in required_types
            if not self.registry.has(task_type)
        ]

        if not required_types:
            message = 'mission has no waypoint tasks'
        elif not missing_types:
            message = 'all task plugins are registered'
        else:
            message = f'missing task plugins: {", ".join(missing_types)}'

        return StartupCheckResult(
            name='task_plugins_registered',
            success=not missing_types,
            message=message,
            data={
                'required_types': required_types,
                'available_types': self.registry.available_types(),
                'missing_types': missing_types,
            },
        )

    def check_navigator_ready(self) -> StartupCheckResult:
        try:
            if not self._navigator_is_ready():
                ready = self.navigator.wait_until_ready(
                    timeout_sec=self.navigator_ready_timeout_sec
                )
                if ready is False:
                    return self._navigator_result(
                        success=False,
                        message=(
                            'navigator did not become ready within '
                            f'{self.navigator_ready_timeout_sec:.1f}s'
                        ),
                    )
        except Exception as exc:
            return self._navigator_result(
                success=False,
                message=f'navigator ready failed: {exc}',
            )

        ready = self._navigator_is_ready()
        message = 'navigator is ready' if ready else 'navigator is not ready'
        return self._navigator_result(success=ready, message=message)

    def _navigator_is_ready(self) -> bool:
        return hasattr(self.navigator, 'is_ready') and self.navigator.is_ready()

    def _navigator_result(self, success: bool, message: str) -> StartupCheckResult:
        return StartupCheckResult(
            name='navigator_ready',
            success=success,
            message=message,
            data={'timeout_sec': self.navigator_ready_timeout_sec},
        )
