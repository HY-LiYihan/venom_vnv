from pathlib import Path

import yaml

from venom_bringup.road_network_waypoint_utils import (
    load_planned_road_route,
    load_route_waypoints,
    route_to_nav2_waypoints,
    write_competition_waypoint_txt,
    write_waypoints_yaml,
)


def _write_yaml(path: Path, payload):
    path.write_text(
        yaml.safe_dump(payload, sort_keys=False),
        encoding='utf-8',
    )


def test_named_route_preserves_action_and_order(tmp_path: Path):
    road_network_file = tmp_path / 'road_network.yaml'
    _write_yaml(
        road_network_file,
        {
            'coordinate_mode': 'cartesian_cm',
            'nodes': {
                'A': {'x': 0, 'y': 0, 'attr': 1},
                'B': {'x': 400, 'y': 0, 'attr': 2},
                'C': {'x': 400, 'y': 200, 'attr': 8},
            },
            'routes': {
                'main': ['A', 'B', 'C'],
            },
        },
    )

    route = load_planned_road_route(str(road_network_file), route_name='main')

    assert route.route_node_ids == ['A', 'B', 'C']
    assert route.waypoints[1].action == 2
    assert route.waypoints[2].action == 8


def test_graph_search_uses_edges_and_replans_when_blocked(tmp_path: Path):
    road_network_file = tmp_path / 'road_network.yaml'
    _write_yaml(
        road_network_file,
        {
            'coordinate_mode': 'cartesian_cm',
            'nodes': {
                'A': {'x': 0, 'y': 0, 'attr': 1},
                'B': {'x': 300, 'y': 0, 'attr': 1},
                'C': {'x': 300, 'y': 300, 'attr': 3},
                'D': {'x': 0, 'y': 300, 'attr': 1},
            },
            'edges': [
                {'from': 'A', 'to': 'B'},
                {'from': 'B', 'to': 'C'},
                {'from': 'A', 'to': 'D'},
                {'from': 'D', 'to': 'C'},
            ],
        },
    )

    default_route = load_planned_road_route(
        str(road_network_file),
        start_node_id='A',
        goal_node_id='C',
    )
    blocked_route = load_planned_road_route(
        str(road_network_file),
        start_node_id='A',
        goal_node_id='C',
        blocked_edges='A->B',
    )

    assert default_route.route_node_ids == ['A', 'B', 'C']
    assert blocked_route.route_node_ids == ['A', 'D', 'C']


def test_nav2_waypoint_export_contains_action_metadata(tmp_path: Path):
    road_network_file = tmp_path / 'road_network.yaml'
    _write_yaml(
        road_network_file,
        {
            'coordinate_mode': 'cartesian_cm',
            'nodes': {
                'A': {'x': 0, 'y': 0, 'attr': 1},
                'B': {'x': 200, 'y': 0, 'attr': 6},
            },
            'routes': {'main': ['A', 'B']},
        },
    )

    nav2_waypoints = load_route_waypoints(str(road_network_file), route_name='main')

    assert nav2_waypoints[1]['action'] == 6
    assert nav2_waypoints[1]['node_id'] == 'B'


def test_competition_waypoint_txt_uses_attr_and_cm(tmp_path: Path):
    road_network_file = tmp_path / 'road_network.yaml'
    _write_yaml(
        road_network_file,
        {
            'coordinate_mode': 'cartesian_cm',
            'nodes': {
                'start': {'x': 0, 'y': 0, 'attr': 1},
                'goal': {'x': 580, 'y': 400, 'attr': 8},
            },
            'routes': {'main': ['start', 'goal']},
        },
    )

    route = load_planned_road_route(str(road_network_file), route_name='main')
    output_file = tmp_path / 'waypoint.txt'
    write_competition_waypoint_txt(route, str(output_file), output_coordinate_mode='cartesian_cm')

    lines = output_file.read_text(encoding='utf-8').splitlines()

    assert lines[1] == '0 0 0 1'
    assert lines[2] == '1 580 400 8'


def test_write_waypoints_yaml_round_trip(tmp_path: Path):
    road_network_file = tmp_path / 'road_network.yaml'
    _write_yaml(
        road_network_file,
        {
            'coordinate_mode': 'cartesian_cm',
            'nodes': {
                'A': {'x': 0, 'y': 0, 'attr': 1},
                'B': {'x': 100, 'y': 0, 'attr': 1},
            },
            'routes': {'demo': ['A', 'B']},
        },
    )

    route = load_planned_road_route(str(road_network_file), route_name='demo')
    output_file = tmp_path / 'waypoints.yaml'
    write_waypoints_yaml(route_to_nav2_waypoints(route), str(output_file), route_name='demo')

    payload = yaml.safe_load(output_file.read_text(encoding='utf-8'))

    assert payload['route_name'] == 'demo'
    assert payload['waypoints'][0]['x'] == 0.0
