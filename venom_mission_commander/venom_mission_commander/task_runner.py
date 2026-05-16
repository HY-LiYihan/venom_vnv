from venom_mission_commander.mission_manager import MissionManager
from venom_mission_commander.models import TaskContext, TaskSpec
from venom_mission_commander.task_plugins import TaskPluginRegistry


class WaypointTaskRunner:
    def __init__(
        self,
        registry: TaskPluginRegistry,
        mission_manager: MissionManager,
        status_reporter=None,
    ):
        self.registry = registry
        self.mission_manager = mission_manager
        self.status_reporter = status_reporter

    def run_tasks(
        self,
        context: TaskContext,
        tasks: list[TaskSpec],
        stop_on_failure: bool = True,
    ) -> bool:
        for task_index, task_spec in enumerate(tasks):
            context.task_index = task_index
            context.node.get_logger().debug(
                f'Running task {task_index + 1}/{len(tasks)}: '
                f'{task_spec.name} ({task_spec.task_type})'
            )
            self.mission_manager.mark_task_started(task_index, task_spec.name)
            self._log_status('task_started')

            try:
                result = self.registry.get(task_spec.task_type).execute(context, task_spec)
            except Exception as exc:
                result_message = f'task raised exception: {exc}'
                context.node.get_logger().error(result_message)
                self.mission_manager.mark_task_failed(
                    task_name=task_spec.name,
                    task_type=task_spec.task_type,
                    message=result_message,
                )
                self._log_status('task_failed')
                if stop_on_failure:
                    self.mission_manager.fail(result_message)
                    return False
                continue

            self.mission_manager.save_state(
                last_task_name=task_spec.name,
                last_task_type=task_spec.task_type,
                last_task_success=result.success,
                last_task_message=result.message,
                last_task_data=result.data,
            )

            if result.success:
                self._log_status('task_completed')
                context.node.get_logger().debug(f'Task succeeded: {result.message}')
                continue

            context.node.get_logger().error(f'Task failed: {result.message}')
            self.mission_manager.mark_task_failed(
                task_name=task_spec.name,
                task_type=task_spec.task_type,
                message=result.message,
            )
            self._log_status('task_failed')
            if stop_on_failure:
                self.mission_manager.fail(result.message)
                return False

        return True

    def _log_status(self, label: str) -> None:
        if self.status_reporter is None:
            return
        self.status_reporter.log_snapshot(label, self.mission_manager)
