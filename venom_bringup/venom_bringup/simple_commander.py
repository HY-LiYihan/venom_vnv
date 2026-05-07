"""Compatibility wrapper for the unified CRAIC mission commander.

`simple_commander` is kept as a legacy entry point because team scripts and
launch files already use it, but the real implementation now lives in
`craic_mission_main.py`.
"""

from __future__ import annotations

import sys

import rclpy

from venom_bringup.craic_mission_main import CraicMissionCommander


class SimpleCommander(CraicMissionCommander):
    """Legacy simple commander alias backed by the unified CRAIC commander."""

    def __init__(self) -> None:
        super().__init__(
            node_name='simple_commander',
            default_coordinate_mode='auto',
            auto_discover_waypoint_file=True,
        )


def main() -> None:
    rclpy.init()
    navigator: SimpleCommander | None = None
    exit_code = 1

    try:
        navigator = SimpleCommander()
        exit_code = navigator.run()
    except Exception as exc:
        if navigator is not None:
            navigator.get_logger().fatal(f'Simple commander crashed: {exc}')
        else:
            print(f'Simple commander crashed before node startup: {exc}', file=sys.stderr)
        exit_code = 1
    finally:
        if navigator is not None:
            navigator.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
