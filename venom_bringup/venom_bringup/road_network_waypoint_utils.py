"""Road-network parsing, route planning, and competition waypoint export."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from pathlib import Path
import re
from typing import Any, Dict, Iterable, List, Optional, Sequence

import yaml

from venom_bringup.craic_waypoint_utils import (
    ACTION_LABELS,
    CraicWaypoint,
    geodetic_to_local_xy,
)


EARTH_RADIUS_METERS = 6378137.0


@dataclass(frozen=True)
class RoadNetworkNode:
    node_id: str
    source_a: float
    source_b: float
    x_m: float
    y_m: float
    coordinate_mode: str
    action: int = 0
    yaw: Optional[float] = None
    frame_id: str = 'map'


@dataclass(frozen=True)
class PlannedRoadWaypoint:
    index: int
    node_id: str
    x_m: float
    y_m: float
    yaw: float
    action: int
    source_a: float
    source_b: float
    coordinate_mode: str
    frame_id: str = 'map'

    @property
    def action_label(self) -> str:
        return ACTION_LABELS.get(self.action, f'action_{self.action}')


@dataclass(frozen=True)
class PlannedRoadRoute:
    route_node_ids: List[str]
    waypoints: List[PlannedRoadWaypoint]
    route_name: Optional[str] = None


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _distance_xy(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def _compute_yaws(points_xy: Sequence[tuple[float, float]]) -> List[float]:
    if not points_xy:
        return []
    if len(points_xy) == 1:
        return [0.0]

    yaws: List[float] = []
    for idx, (x_value, y_value) in enumerate(points_xy):
        if idx < len(points_xy) - 1:
            next_x, next_y = points_xy[idx + 1]
            yaw = math.atan2(next_y - y_value, next_x - x_value)
        else:
            prev_x, prev_y = points_xy[idx - 1]
            yaw = math.atan2(y_value - prev_y, x_value - prev_x)
        yaws.append(_normalize_angle(yaw))
    return yaws


def _split_route_nodes(route_nodes: str) -> List[str]:
    cleaned = route_nodes.replace('->', ',').replace('>', ',')
    return [part.strip() for part in re.split(r'[,\s;|]+', cleaned) if part.strip()]


def _coerce_route_node_list(route_nodes: Optional[Sequence[Any] | str]) -> List[Any]:
    if route_nodes is None:
        return []
    if isinstance(route_nodes, str):
        stripped = route_nodes.strip()
        if not stripped:
            return []
        if stripped.startswith('[') and stripped.endswith(']'):
            parsed = yaml.safe_load(stripped)
            if isinstance(parsed, list):
                return [item for item in parsed if item is not None]
        return _split_route_nodes(route_nodes)
    return [item for item in route_nodes if item is not None]


def _load_yaml(file_path: str) -> Dict[str, Any]:
    path = Path(file_path)
    if not path.is_file():
        raise FileNotFoundError(f'Road network file not found: {file_path}')

    with path.open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}

    if not isinstance(data, dict):
        raise ValueError(f'Road network file must contain a YAML mapping: {file_path}')
    return data


def _infer_coordinate_mode(value_a: float, value_b: float) -> str:
    if -180.0 <= value_a <= 180.0 and -90.0 <= value_b <= 90.0:
        if abs(value_a) > 20.0 or abs(value_b) > 20.0:
            return 'geodetic'
    if abs(value_a) >= 100 or abs(value_b) >= 100:
        return 'cartesian_cm'
    return 'cartesian_m'


def _resolve_coordinate_mode(
    node_data: Dict[str, Any],
    explicit_mode: Optional[str],
    file_mode: Optional[str],
    value_a: float,
    value_b: float,
) -> str:
    for candidate in (
        explicit_mode,
        node_data.get('coordinate_mode'),
        file_mode,
        'auto',
    ):
        if not candidate:
            continue
        mode = str(candidate)
        if mode == 'auto':
            return _infer_coordinate_mode(value_a, value_b)
        if mode in {'geodetic', 'cartesian_m', 'cartesian_cm'}:
            return mode
        raise ValueError(f'Unsupported coordinate_mode "{mode}" in road network file')
    return _infer_coordinate_mode(value_a, value_b)


def _project_to_local_xy(
    value_a: float,
    value_b: float,
    coordinate_mode: str,
    map_origin_longitude_deg: float,
    map_origin_latitude_deg: float,
    map_origin_yaw_rad: float,
    map_origin_x_m: float,
    map_origin_y_m: float,
) -> tuple[float, float]:
    if coordinate_mode == 'geodetic':
        return geodetic_to_local_xy(
            longitude_deg=value_a,
            latitude_deg=value_b,
            origin_longitude_deg=map_origin_longitude_deg,
            origin_latitude_deg=map_origin_latitude_deg,
            map_origin_yaw_rad=map_origin_yaw_rad,
            map_origin_x_m=map_origin_x_m,
            map_origin_y_m=map_origin_y_m,
        )
    if coordinate_mode == 'cartesian_cm':
        return value_a * 0.01, value_b * 0.01
    return value_a, value_b


def _extract_node_id(node_data: Dict[str, Any]) -> str:
    node_id = node_data.get('id') or node_data.get('name') or node_data.get('node_id')
    if node_id is None:
        raise ValueError(f'Node entry is missing an id/name field: {node_data}')
    return str(node_id)


def _extract_source_pair(node_data: Dict[str, Any]) -> tuple[float, float]:
    if 'longitude' in node_data and 'latitude' in node_data:
        return float(node_data['longitude']), float(node_data['latitude'])
    if 'lon' in node_data and 'lat' in node_data:
        return float(node_data['lon']), float(node_data['lat'])
    if 'x' in node_data and 'y' in node_data:
        return float(node_data['x']), float(node_data['y'])
    if 'source_a' in node_data and 'source_b' in node_data:
        return float(node_data['source_a']), float(node_data['source_b'])
    raise ValueError(f'Node entry is missing coordinates: {node_data}')


def _normalize_edge_entry(edge: Any) -> tuple[str, Optional[float]]:
    if isinstance(edge, str):
        return edge, None
    if isinstance(edge, dict):
        target = edge.get('to') or edge.get('target') or edge.get('node') or edge.get('id')
        if target is None:
            raise ValueError(f'Edge entry is missing target node: {edge}')
        weight = edge.get('weight')
        return str(target), float(weight) if weight is not None else None
    raise ValueError(f'Unsupported edge entry: {edge!r}')


def _append_bidirectional_edge(
    adjacency: Dict[str, List[tuple[str, float]]],
    node_lookup: Dict[str, RoadNetworkNode],
    start_id: str,
    end_id: str,
    weight: Optional[float],
    bidirectional: bool = True,
) -> None:
    if start_id not in node_lookup:
        raise KeyError(f'Unknown road-network node id: {start_id}')
    if end_id not in node_lookup:
        raise KeyError(f'Unknown road-network node id: {end_id}')

    start = node_lookup[start_id]
    end = node_lookup[end_id]
    edge_weight = weight
    if edge_weight is None:
        edge_weight = _distance_xy(start.x_m, start.y_m, end.x_m, end.y_m)

    adjacency.setdefault(start_id, []).append((end_id, edge_weight))
    if bidirectional:
        adjacency.setdefault(end_id, []).append((start_id, edge_weight))


def _build_adjacency(
    data: Dict[str, Any],
    node_lookup: Dict[str, RoadNetworkNode],
) -> Dict[str, List[tuple[str, float]]]:
    adjacency: Dict[str, List[tuple[str, float]]] = {node_id: [] for node_id in node_lookup}

    edges = data.get('edges', [])
    if isinstance(edges, list):
        for edge in edges:
            if not isinstance(edge, dict):
                raise ValueError(f'Edge entry must be a mapping: {edge!r}')
            start_id = edge.get('from') or edge.get('source') or edge.get('start')
            end_id = edge.get('to') or edge.get('target') or edge.get('end')
            if start_id is None or end_id is None:
                raise ValueError(f'Edge entry must define from/to: {edge}')
            bidirectional = bool(edge.get('bidirectional', True))
            weight = edge.get('weight')
            _append_bidirectional_edge(
                adjacency,
                node_lookup,
                str(start_id),
                str(end_id),
                float(weight) if weight is not None else None,
                bidirectional=bidirectional,
            )
    elif edges:
        raise ValueError('edges must be a list when present')

    for node_id, node_data in _iter_node_entries(data):
        connections = (
            node_data.get('neighbors')
            or node_data.get('adjacent')
            or node_data.get('connections')
        )
        if not connections:
            continue
        if not isinstance(connections, list):
            raise ValueError(f'Node "{node_id}" connections must be a list')
        for connection in connections:
            target_id, weight = _normalize_edge_entry(connection)
            _append_bidirectional_edge(adjacency, node_lookup, node_id, target_id, weight)

    routes = data.get('routes', {})
    if isinstance(routes, dict):
        for route_path in routes.values():
            if isinstance(route_path, list):
                _connect_path_nodes(route_path, adjacency, node_lookup)
    elif isinstance(routes, list):
        for route_data in routes:
            if not isinstance(route_data, dict):
                raise ValueError('Each route entry must be a mapping')
            route_path = route_data.get('path') or route_data.get('nodes') or route_data.get('waypoints')
            if isinstance(route_path, list):
                _connect_path_nodes(route_path, adjacency, node_lookup)

    direct_route = data.get('route') or data.get('path') or data.get('waypoints')
    if isinstance(direct_route, list):
        _connect_path_nodes(direct_route, adjacency, node_lookup)

    if not any(adjacency.values()):
        raise ValueError('Road network file does not define any traversable edges')
    return adjacency


def _connect_path_nodes(
    route_path: Sequence[Any],
    adjacency: Dict[str, List[tuple[str, float]]],
    node_lookup: Dict[str, RoadNetworkNode],
) -> None:
    for idx in range(len(route_path) - 1):
        start_entry = route_path[idx]
        end_entry = route_path[idx + 1]
        if not isinstance(start_entry, str) or not isinstance(end_entry, str):
            continue
        _append_bidirectional_edge(adjacency, node_lookup, start_entry, end_entry, weight=None)


def _iter_node_entries(data: Dict[str, Any]) -> Iterable[tuple[str, Dict[str, Any]]]:
    nodes = data.get('nodes', {})
    if isinstance(nodes, dict):
        for node_id, node_data in nodes.items():
            if not isinstance(node_data, dict):
                raise ValueError(f'Node "{node_id}" must be a mapping')
            yield str(node_id), node_data
        return
    if isinstance(nodes, list):
        for node_data in nodes:
            if not isinstance(node_data, dict):
                raise ValueError('Each node entry must be a mapping')
            yield _extract_node_id(node_data), node_data
        return
    if nodes:
        raise ValueError('nodes must be either a mapping or a list')


def _collect_nodes(
    data: Dict[str, Any],
    default_frame_id: str,
    coordinate_mode: str,
    map_origin_longitude_deg: float,
    map_origin_latitude_deg: float,
    map_origin_yaw_rad: float,
    map_origin_x_m: float,
    map_origin_y_m: float,
) -> Dict[str, RoadNetworkNode]:
    node_lookup: Dict[str, RoadNetworkNode] = {}
    file_mode = data.get('coordinate_mode')

    for node_id, node_data in _iter_node_entries(data):
        source_a, source_b = _extract_source_pair(node_data)
        resolved_mode = _resolve_coordinate_mode(
            node_data=node_data,
            explicit_mode=coordinate_mode,
            file_mode=str(file_mode) if file_mode is not None else None,
            value_a=source_a,
            value_b=source_b,
        )
        x_m, y_m = _project_to_local_xy(
            value_a=source_a,
            value_b=source_b,
            coordinate_mode=resolved_mode,
            map_origin_longitude_deg=map_origin_longitude_deg,
            map_origin_latitude_deg=map_origin_latitude_deg,
            map_origin_yaw_rad=map_origin_yaw_rad,
            map_origin_x_m=map_origin_x_m,
            map_origin_y_m=map_origin_y_m,
        )
        action = int(node_data.get('action', node_data.get('attr', 0)))
        yaw_value = node_data.get('yaw')
        node_lookup[node_id] = RoadNetworkNode(
            node_id=node_id,
            source_a=source_a,
            source_b=source_b,
            x_m=x_m,
            y_m=y_m,
            coordinate_mode=resolved_mode,
            action=action,
            yaw=float(yaw_value) if yaw_value is not None else None,
            frame_id=str(node_data.get('frame_id', default_frame_id)),
        )

    if not node_lookup:
        raise ValueError('Road network file does not define any nodes')
    return node_lookup


def _collect_named_routes(data: Dict[str, Any]) -> Dict[str, List[Any]]:
    routes = data.get('routes', {})
    route_lookup: Dict[str, List[Any]] = {}

    if isinstance(routes, dict):
        for route_name, route_path in routes.items():
            if not isinstance(route_path, list):
                raise ValueError(f'Route "{route_name}" must be a list')
            route_lookup[str(route_name)] = route_path
        return route_lookup

    if isinstance(routes, list):
        for route_data in routes:
            if not isinstance(route_data, dict):
                raise ValueError('Each route entry must be a mapping')
            route_name = route_data.get('name') or route_data.get('id') or route_data.get('route_name')
            route_path = route_data.get('path') or route_data.get('nodes') or route_data.get('waypoints')
            if route_name is None or route_path is None:
                raise ValueError(f'Route entry must define name and path: {route_data}')
            if not isinstance(route_path, list):
                raise ValueError(f'Route "{route_name}" path must be a list')
            route_lookup[str(route_name)] = route_path
        return route_lookup

    if routes:
        raise ValueError('routes must be either a mapping or a list')
    return route_lookup


def _find_nearest_node_id(
    node_lookup: Dict[str, RoadNetworkNode],
    query: Optional[str] = None,
    x_m: Optional[float] = None,
    y_m: Optional[float] = None,
) -> str:
    if query:
        if query in node_lookup:
            return query
        raise KeyError(f'Unknown road-network node id: {query}')

    if x_m is None or y_m is None:
        raise ValueError('Nearest-node lookup requires both x_m and y_m')

    return min(
        node_lookup,
        key=lambda node_id: _distance_xy(
            node_lookup[node_id].x_m,
            node_lookup[node_id].y_m,
            x_m,
            y_m,
        ),
    )


def _parse_blocked_edges(blocked_edges: Optional[Sequence[str] | str]) -> set[tuple[str, str]]:
    if not blocked_edges:
        return set()
    if isinstance(blocked_edges, str):
        tokens = [token.strip() for token in re.split(r'[;,]+', blocked_edges) if token.strip()]
    else:
        tokens = [str(token).strip() for token in blocked_edges if str(token).strip()]

    blocked: set[tuple[str, str]] = set()
    for token in tokens:
        if '->' in token:
            start_id, end_id = [part.strip() for part in token.split('->', maxsplit=1)]
        elif ':' in token:
            start_id, end_id = [part.strip() for part in token.split(':', maxsplit=1)]
        else:
            raise ValueError(
                f'Blocked edge "{token}" must use "A->B" or "A:B" format'
            )
        blocked.add((start_id, end_id))
        blocked.add((end_id, start_id))
    return blocked


def _shortest_path(
    adjacency: Dict[str, List[tuple[str, float]]],
    start_id: str,
    goal_id: str,
    blocked_edges: set[tuple[str, str]],
) -> List[str]:
    frontier: List[tuple[float, str]] = [(0.0, start_id)]
    distances: Dict[str, float] = {start_id: 0.0}
    previous: Dict[str, Optional[str]] = {start_id: None}

    while frontier:
        current_cost, current_id = heapq.heappop(frontier)
        if current_id == goal_id:
            break
        if current_cost > distances.get(current_id, math.inf):
            continue

        for neighbor_id, weight in adjacency.get(current_id, []):
            if (current_id, neighbor_id) in blocked_edges:
                continue
            next_cost = current_cost + weight
            if next_cost >= distances.get(neighbor_id, math.inf):
                continue
            distances[neighbor_id] = next_cost
            previous[neighbor_id] = current_id
            heapq.heappush(frontier, (next_cost, neighbor_id))

    if goal_id not in previous:
        raise ValueError(f'No route found from {start_id} to {goal_id}')

    path: List[str] = []
    current_id: Optional[str] = goal_id
    while current_id is not None:
        path.append(current_id)
        current_id = previous[current_id]
    path.reverse()
    return path


def _resolve_explicit_route_nodes(
    route_entries: Sequence[Any],
    node_lookup: Dict[str, RoadNetworkNode],
    default_frame_id: str,
    map_origin_longitude_deg: float,
    map_origin_latitude_deg: float,
    map_origin_yaw_rad: float,
    map_origin_x_m: float,
    map_origin_y_m: float,
    coordinate_mode: str,
) -> List[RoadNetworkNode]:
    nodes: List[RoadNetworkNode] = []
    for entry in route_entries:
        if isinstance(entry, str):
            node = node_lookup.get(entry)
            if node is None:
                raise KeyError(f'Unknown road-network node id: {entry}')
            nodes.append(node)
            continue
        if not isinstance(entry, dict):
            raise ValueError(f'Unsupported route entry: {entry!r}')

        node_id = entry.get('id') or entry.get('name') or entry.get('node_id') or entry.get('node')
        if node_id is not None and str(node_id) in node_lookup:
            nodes.append(node_lookup[str(node_id)])
            continue

        source_a, source_b = _extract_source_pair(entry)
        resolved_mode = _resolve_coordinate_mode(
            node_data=entry,
            explicit_mode=coordinate_mode,
            file_mode=None,
            value_a=source_a,
            value_b=source_b,
        )
        x_m, y_m = _project_to_local_xy(
            value_a=source_a,
            value_b=source_b,
            coordinate_mode=resolved_mode,
            map_origin_longitude_deg=map_origin_longitude_deg,
            map_origin_latitude_deg=map_origin_latitude_deg,
            map_origin_yaw_rad=map_origin_yaw_rad,
            map_origin_x_m=map_origin_x_m,
            map_origin_y_m=map_origin_y_m,
        )
        nodes.append(
            RoadNetworkNode(
                node_id=str(node_id) if node_id is not None else f'inline_{len(nodes)}',
                source_a=source_a,
                source_b=source_b,
                x_m=x_m,
                y_m=y_m,
                coordinate_mode=resolved_mode,
                action=int(entry.get('action', entry.get('attr', 0))),
                yaw=float(entry['yaw']) if entry.get('yaw') is not None else None,
                frame_id=str(entry.get('frame_id', default_frame_id)),
            )
        )
    return nodes


def _materialize_planned_route(
    nodes: Sequence[RoadNetworkNode],
    route_name: Optional[str] = None,
) -> PlannedRoadRoute:
    if not nodes:
        raise ValueError('Planned route is empty')

    points_xy = [(node.x_m, node.y_m) for node in nodes]
    inferred_yaws = _compute_yaws(points_xy)

    waypoints: List[PlannedRoadWaypoint] = []
    for idx, node in enumerate(nodes):
        yaw = node.yaw if node.yaw is not None else inferred_yaws[idx]
        waypoints.append(
            PlannedRoadWaypoint(
                index=idx,
                node_id=node.node_id,
                x_m=node.x_m,
                y_m=node.y_m,
                yaw=float(yaw),
                action=node.action,
                source_a=node.source_a,
                source_b=node.source_b,
                coordinate_mode=node.coordinate_mode,
                frame_id=node.frame_id,
            )
        )

    return PlannedRoadRoute(
        route_node_ids=[node.node_id for node in nodes],
        waypoints=waypoints,
        route_name=route_name,
    )


def load_planned_road_route(
    file_path: str,
    route_name: Optional[str] = None,
    route_nodes: Optional[Sequence[Any] | str] = None,
    default_frame_id: str = 'map',
    coordinate_mode: str = 'auto',
    map_origin_longitude_deg: float = 0.0,
    map_origin_latitude_deg: float = 0.0,
    map_origin_yaw_rad: float = 0.0,
    map_origin_x_m: float = 0.0,
    map_origin_y_m: float = 0.0,
    start_node_id: Optional[str] = None,
    goal_node_id: Optional[str] = None,
    start_x_m: Optional[float] = None,
    start_y_m: Optional[float] = None,
    goal_x_m: Optional[float] = None,
    goal_y_m: Optional[float] = None,
    blocked_edges: Optional[Sequence[str] | str] = None,
) -> PlannedRoadRoute:
    """Parse a road network file and produce a planned competition route."""
    data = _load_yaml(file_path)
    node_lookup = _collect_nodes(
        data=data,
        default_frame_id=default_frame_id,
        coordinate_mode=coordinate_mode,
        map_origin_longitude_deg=map_origin_longitude_deg,
        map_origin_latitude_deg=map_origin_latitude_deg,
        map_origin_yaw_rad=map_origin_yaw_rad,
        map_origin_x_m=map_origin_x_m,
        map_origin_y_m=map_origin_y_m,
    )
    named_routes = _collect_named_routes(data)

    explicit_route_entries = _coerce_route_node_list(route_nodes) if route_nodes else []
    if explicit_route_entries:
        route_nodes_resolved = _resolve_explicit_route_nodes(
            route_entries=explicit_route_entries,
            node_lookup=node_lookup,
            default_frame_id=default_frame_id,
            map_origin_longitude_deg=map_origin_longitude_deg,
            map_origin_latitude_deg=map_origin_latitude_deg,
            map_origin_yaw_rad=map_origin_yaw_rad,
            map_origin_x_m=map_origin_x_m,
            map_origin_y_m=map_origin_y_m,
            coordinate_mode=coordinate_mode,
        )
        return _materialize_planned_route(route_nodes_resolved, route_name=route_name)

    if route_name:
        if route_name not in named_routes:
            raise KeyError(f'Route "{route_name}" not found in {file_path}')
        named_route_nodes = _resolve_explicit_route_nodes(
            route_entries=named_routes[route_name],
            node_lookup=node_lookup,
            default_frame_id=default_frame_id,
            map_origin_longitude_deg=map_origin_longitude_deg,
            map_origin_latitude_deg=map_origin_latitude_deg,
            map_origin_yaw_rad=map_origin_yaw_rad,
            map_origin_x_m=map_origin_x_m,
            map_origin_y_m=map_origin_y_m,
            coordinate_mode=coordinate_mode,
        )
        return _materialize_planned_route(named_route_nodes, route_name=route_name)

    direct_route = data.get('route') or data.get('path') or data.get('waypoints')
    if isinstance(direct_route, list) and direct_route:
        direct_nodes = _resolve_explicit_route_nodes(
            route_entries=direct_route,
            node_lookup=node_lookup,
            default_frame_id=default_frame_id,
            map_origin_longitude_deg=map_origin_longitude_deg,
            map_origin_latitude_deg=map_origin_latitude_deg,
            map_origin_yaw_rad=map_origin_yaw_rad,
            map_origin_x_m=map_origin_x_m,
            map_origin_y_m=map_origin_y_m,
            coordinate_mode=coordinate_mode,
        )
        return _materialize_planned_route(direct_nodes)

    resolved_start_id = start_node_id or data.get('start_node_id') or data.get('start')
    resolved_goal_id = goal_node_id or data.get('goal_node_id') or data.get('goal')

    if resolved_start_id is None and start_x_m is None:
        raise ValueError(
            'No route selected. Provide route_name, route_nodes, or start/goal inputs for graph search.'
        )
    if resolved_goal_id is None and goal_x_m is None:
        raise ValueError(
            'No goal selected. Provide route_name, route_nodes, or start/goal inputs for graph search.'
        )

    adjacency = _build_adjacency(data, node_lookup)
    start_id = _find_nearest_node_id(
        node_lookup=node_lookup,
        query=str(resolved_start_id) if resolved_start_id is not None else None,
        x_m=start_x_m,
        y_m=start_y_m,
    )
    goal_id = _find_nearest_node_id(
        node_lookup=node_lookup,
        query=str(resolved_goal_id) if resolved_goal_id is not None else None,
        x_m=goal_x_m,
        y_m=goal_y_m,
    )
    blocked = _parse_blocked_edges(blocked_edges or data.get('blocked_edges'))
    path_node_ids = _shortest_path(adjacency, start_id, goal_id, blocked)
    return _materialize_planned_route(
        [node_lookup[node_id] for node_id in path_node_ids],
        route_name=f'{start_id}_to_{goal_id}',
    )


def route_to_nav2_waypoints(route: PlannedRoadRoute) -> List[Dict[str, Any]]:
    return [
        {
            'frame_id': waypoint.frame_id,
            'x': waypoint.x_m,
            'y': waypoint.y_m,
            'yaw': waypoint.yaw,
            'action': waypoint.action,
            'node_id': waypoint.node_id,
            'source_a': waypoint.source_a,
            'source_b': waypoint.source_b,
        }
        for waypoint in route.waypoints
    ]


def route_to_craic_waypoints(route: PlannedRoadRoute) -> List[CraicWaypoint]:
    return [
        CraicWaypoint(
            index=waypoint.index,
            x=waypoint.x_m,
            y=waypoint.y_m,
            yaw=waypoint.yaw,
            action=waypoint.action,
            source_a=waypoint.source_a,
            source_b=waypoint.source_b,
            action_label=waypoint.action_label,
        )
        for waypoint in route.waypoints
    ]


def load_route_waypoints(
    file_path: str,
    route_name: Optional[str] = None,
    route_nodes: Optional[Sequence[Any] | str] = None,
    default_frame_id: str = 'map',
    coordinate_mode: str = 'auto',
    map_origin_longitude_deg: float = 0.0,
    map_origin_latitude_deg: float = 0.0,
    map_origin_yaw_rad: float = 0.0,
    map_origin_x_m: float = 0.0,
    map_origin_y_m: float = 0.0,
    start_node_id: Optional[str] = None,
    goal_node_id: Optional[str] = None,
    start_x_m: Optional[float] = None,
    start_y_m: Optional[float] = None,
    goal_x_m: Optional[float] = None,
    goal_y_m: Optional[float] = None,
    blocked_edges: Optional[Sequence[str] | str] = None,
) -> List[Dict[str, Any]]:
    route = load_planned_road_route(
        file_path=file_path,
        route_name=route_name,
        route_nodes=route_nodes,
        default_frame_id=default_frame_id,
        coordinate_mode=coordinate_mode,
        map_origin_longitude_deg=map_origin_longitude_deg,
        map_origin_latitude_deg=map_origin_latitude_deg,
        map_origin_yaw_rad=map_origin_yaw_rad,
        map_origin_x_m=map_origin_x_m,
        map_origin_y_m=map_origin_y_m,
        start_node_id=start_node_id,
        goal_node_id=goal_node_id,
        start_x_m=start_x_m,
        start_y_m=start_y_m,
        goal_x_m=goal_x_m,
        goal_y_m=goal_y_m,
        blocked_edges=blocked_edges,
    )
    return route_to_nav2_waypoints(route)


def write_waypoints_yaml(
    waypoints: Iterable[Dict[str, Any]],
    output_file: str,
    route_name: Optional[str] = None,
) -> None:
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    payload: Dict[str, Any] = {'waypoints': list(waypoints)}
    if route_name:
        payload['route_name'] = route_name

    with output_path.open('w', encoding='utf-8') as handle:
        yaml.safe_dump(payload, handle, sort_keys=False)


def write_competition_waypoint_txt(
    route: PlannedRoadRoute,
    output_file: str,
    output_coordinate_mode: str = 'cartesian_cm',
) -> None:
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    lines = ['# seq source_a source_b attr']
    for waypoint in route.waypoints:
        if output_coordinate_mode == 'geodetic':
            value_a = f'{waypoint.source_a:.5f}'
            value_b = f'{waypoint.source_b:.5f}'
        elif output_coordinate_mode == 'cartesian_cm':
            value_a = str(int(round(waypoint.x_m * 100.0)))
            value_b = str(int(round(waypoint.y_m * 100.0)))
        elif output_coordinate_mode == 'cartesian_m':
            value_a = f'{waypoint.x_m:.3f}'
            value_b = f'{waypoint.y_m:.3f}'
        else:
            raise ValueError(
                'output_coordinate_mode must be one of geodetic, cartesian_cm, cartesian_m'
            )
        lines.append(f'{waypoint.index} {value_a} {value_b} {waypoint.action}')

    output_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
