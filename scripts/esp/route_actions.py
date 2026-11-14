"""Convert a planned graph route to the ESP's junction-action sequence."""

import math
from typing import Any


def _world_point(point: list[float], transform: dict[str, Any]) -> tuple[float, float]:
    origin_x, origin_y = transform["world_origin"]
    scale_x, scale_y = transform["world_units_per_pixel"]
    return origin_x + point[0] * scale_x, origin_y + point[1] * scale_y


def _oriented_geometry(edge: dict[str, Any], source: str, target: str) -> list[list[float]]:
    geometry = edge.get("geometry")
    if not isinstance(geometry, list) or len(geometry) < 2:
        raise ValueError(f"Route edge {edge.get('id')!r} has no usable geometry.")

    if edge.get("from") == source and edge.get("to") == target:
        return geometry
    if (
        edge.get("bidirectional", True)
        and edge.get("from") == target
        and edge.get("to") == source
    ):
        return list(reversed(geometry))
    raise ValueError(
        f"Route traverses edge {edge.get('id')!r} in a forbidden direction."
    )


def _junction_turn(
    node: dict[str, Any],
    incoming_geometry: list[list[float]],
    outgoing_geometry: list[list[float]],
    transform: dict[str, Any],
    straight_threshold_degrees: float,
) -> str:
    vertex = _world_point([node["x"], node["y"]], transform)

    incoming = None
    for point in reversed(incoming_geometry[:-1]):
        previous = _world_point(point, transform)
        vector = (vertex[0] - previous[0], vertex[1] - previous[1])
        if math.hypot(*vector) > 1e-9:
            incoming = vector
            break

    outgoing = None
    for point in outgoing_geometry[1:]:
        following = _world_point(point, transform)
        vector = (following[0] - vertex[0], following[1] - vertex[1])
        if math.hypot(*vector) > 1e-9:
            outgoing = vector
            break

    if incoming is None or outgoing is None:
        raise ValueError(f"Cannot determine route direction at node {node['id']!r}.")

    cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
    dot = incoming[0] * outgoing[0] + incoming[1] * outgoing[1]
    angle = math.degrees(math.atan2(cross, dot))
    if abs(angle) <= straight_threshold_degrees:
        return "S"
    return "L" if angle > 0 else "R"


def _polyline_length(geometry: list[list[float]], transform: dict[str, Any]) -> float:
    points = [_world_point(point, transform) for point in geometry]
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def route_action_plan(
    map_data: dict[str, Any],
    route: Any,
    straight_threshold_degrees: float = 35.0,
    distance_scale: float = 1.0,
) -> list[tuple[str, float]]:
    """Return ``(action, distance_m)`` per intermediate junction, then ``D``.

    ``distance_m`` is the distance along the route's edge centerlines from the
    start node to that junction (or to the destination), multiplied by
    ``distance_scale``. The robot cuts inside curves, so its wheel odometry
    reads less than the drawn centerline; the scale converts map distance to
    the expected odometry distance. The start node is omitted: the robot
    begins on it.
    """
    if distance_scale <= 0.0:
        raise ValueError("distance_scale must be positive.")
    actions = route_actions(map_data, route, straight_threshold_degrees)
    edges = {edge["id"]: edge for edge in map_data.get("edges", [])}
    transform = map_data["world_transform"]

    distances = []
    travelled = 0.0
    for step in route.steps:
        geometry = _oriented_geometry(edges[step.edge_id], step.source, step.target)
        travelled += _polyline_length(geometry, transform)
        distances.append(travelled * distance_scale)
    return list(zip(actions, distances))


def format_route_command(plan: list[tuple[str, float]]) -> str:
    """Format a plan as the firmware's route payload, e.g. ``R@0.265,D@0.779``."""
    return ",".join(f"{action}@{distance:.3f}" for action, distance in plan)


def route_actions(
    map_data: dict[str, Any],
    route: Any,
    straight_threshold_degrees: float = 35.0,
) -> str:
    """Return one relative action per intermediate junction, then destination.

    The route's start node is intentionally omitted: the robot begins on it.
    """
    if not 0.0 < straight_threshold_degrees < 90.0:
        raise ValueError("straight_threshold_degrees must be between 0 and 90.")
    if len(route.nodes) != len(route.steps) + 1:
        raise ValueError("Route nodes and steps do not form a continuous path.")

    nodes = {node["id"]: node for node in map_data.get("nodes", [])}
    edges = {edge["id"]: edge for edge in map_data.get("edges", [])}
    transform = map_data.get("world_transform")
    if not isinstance(transform, dict):
        raise ValueError("Map has no world_transform for turn conversion.")

    actions = []
    for index, node_id in enumerate(route.nodes[1:-1], start=1):
        node = nodes.get(node_id)
        incoming_step = route.steps[index - 1]
        outgoing_step = route.steps[index]
        if node is None:
            raise ValueError(f"Route references unknown node {node_id!r}.")
        try:
            incoming_edge = edges[incoming_step.edge_id]
            outgoing_edge = edges[outgoing_step.edge_id]
        except KeyError as error:
            raise ValueError(f"Route references unknown edge {error.args[0]!r}.") from error

        incoming_geometry = _oriented_geometry(
            incoming_edge, incoming_step.source, incoming_step.target
        )
        outgoing_geometry = _oriented_geometry(
            outgoing_edge, outgoing_step.source, outgoing_step.target
        )
        actions.append(
            _junction_turn(
                node,
                incoming_geometry,
                outgoing_geometry,
                transform,
                straight_threshold_degrees,
            )
        )

    actions.append("D")
    return "".join(actions)
