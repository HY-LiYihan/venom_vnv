"""CLI for converting a road-network route into a Nav2 waypoint YAML file."""

from __future__ import annotations

import argparse
import sys

from venom_bringup.road_network_waypoint_utils import (
    load_planned_road_route,
    write_waypoints_yaml,
    route_to_nav2_waypoints,
    write_competition_waypoint_txt,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Convert a road-network route into Nav2 waypoints YAML.'
    )
    parser.add_argument(
        '--road-network-file',
        required=True,
        help='Input YAML file describing nodes and routes.',
    )
    parser.add_argument(
        '--output-file',
        required=True,
        help='Output Nav2 waypoints YAML file.',
    )
    parser.add_argument(
        '--route-name',
        default='',
        help='Named route to extract from the road network file.',
    )
    parser.add_argument(
        '--route-nodes',
        default='',
        help='Explicit route node list, for example "A,B,C" or "A>B>C".',
    )
    parser.add_argument(
        '--frame-id',
        default='map',
        help='Fallback TF frame for route entries that do not specify one.',
    )
    parser.add_argument(
        '--coordinate-mode',
        default='auto',
        help='Road-network coordinate mode: auto, geodetic, cartesian_m, or cartesian_cm.',
    )
    parser.add_argument(
        '--start-node-id',
        default='',
        help='Graph-search start node id.',
    )
    parser.add_argument(
        '--goal-node-id',
        default='',
        help='Graph-search goal node id.',
    )
    parser.add_argument(
        '--blocked-edges',
        default='',
        help='Blocked edges, for example "A->B;B->C".',
    )
    parser.add_argument(
        '--competition-output-file',
        default='',
        help='Optional CRAIC-style waypoint.txt output path.',
    )
    parser.add_argument(
        '--competition-coordinate-mode',
        default='cartesian_cm',
        help='Competition output mode: geodetic, cartesian_cm, or cartesian_m.',
    )
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    route = load_planned_road_route(
        file_path=args.road_network_file,
        route_name=args.route_name or None,
        route_nodes=args.route_nodes or None,
        coordinate_mode=args.coordinate_mode,
        default_frame_id=args.frame_id,
        start_node_id=args.start_node_id or None,
        goal_node_id=args.goal_node_id or None,
        blocked_edges=args.blocked_edges or None,
    )
    waypoints = route_to_nav2_waypoints(route)
    write_waypoints_yaml(waypoints, args.output_file, route_name=route.route_name)
    if args.competition_output_file:
        write_competition_waypoint_txt(
            route,
            args.competition_output_file,
            output_coordinate_mode=args.competition_coordinate_mode,
        )

    print(
        f'Wrote {len(waypoints)} waypoint(s) from {args.road_network_file} '
        f'to {args.output_file}'
    )


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:  # pragma: no cover - CLI guard
        print(f'Conversion failed: {exc}', file=sys.stderr)
        sys.exit(1)
