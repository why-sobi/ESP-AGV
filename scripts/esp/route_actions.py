"""Convert a planned graph route to the ESP's junction-action sequence."""

import math
from typing import Any

import numpy as np


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


def _distance_to_segments(points: Any, starts: Any, ends: Any) -> Any:
    """Distance from each point (P, 2) to the nearest segment (S, 2)-(S, 2)."""
    direction = ends - starts
    length_sq = np.maximum((direction ** 2).sum(axis=1), 1e-18)
    offset = points[:, None, :] - starts[None, :, :]
    t = np.clip((offset * direction[None]).sum(axis=2) / length_sq[None], 0.0, 1.0)
    nearest = starts[None] + t[..., None] * direction[None]
    return np.sqrt(((points[:, None, :] - nearest) ** 2).sum(axis=2)).min(axis=1)


def pull_over_clearance(
    map_data: dict[str, Any],
    route: Any,
    drive_m: float = 0.08,
    robot_radius_m: float = 0.037,
    junction_gap_m: float = 0.06,
    sample_m: float = 0.01,
    arena_half_m: float = 0.5,
) -> list[dict[str, float]]:
    """Per route step, the worst clearance of a pull-over to each side.

    The ESP pulls over by turning 90 degrees in place and driving ``drive_m``
    forward. From points every ``sample_m`` along the step's edge (except
    within ``junction_gap_m`` of its end nodes, where other lines meet anyway),
    the robot's path from one robot radius out to ``drive_m + robot_radius_m``
    is checked against every other edge's centerline and the arena walls; the
    result is the smallest distance per side, in metres (negative: past a
    wall).
    """
    edges = {edge["id"]: edge for edge in map_data.get("edges", [])}
    transform = map_data["world_transform"]
    world = {
        edge_id: np.array([_world_point(point, transform) for point in edge["geometry"]])
        for edge_id, edge in edges.items()
    }
    reach = drive_m + robot_radius_m
    probe_steps = robot_radius_m + (reach - robot_radius_m) * np.arange(1, 9) / 8
    result = []
    for step in route.steps:
        points = np.array([
            _world_point(point, transform)
            for point in _oriented_geometry(edges[step.edge_id], step.source, step.target)
        ])
        others = [line for edge_id, line in world.items() if edge_id != step.edge_id]
        seg_starts = np.vstack([line[:-1] for line in others])
        seg_ends = np.vstack([line[1:] for line in others])
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        keep = lengths > 0
        a, direction, lengths = points[:-1][keep], np.diff(points, axis=0)[keep], lengths[keep]
        cumulative = np.concatenate([[0.0], np.cumsum(lengths)])
        samples = np.arange(junction_gap_m, cumulative[-1] - junction_gap_m + 1e-12, sample_m)
        clearance = {"R": float("inf"), "L": float("inf")}
        if len(samples):
            index = np.clip(np.searchsorted(cumulative, samples, side="right") - 1,
                            0, len(lengths) - 1)
            tangent = direction[index] / lengths[index, None]
            here = a[index] + tangent * (samples - cumulative[index])[:, None]
            for side, normal in (("R", np.column_stack([tangent[:, 1], -tangent[:, 0]])),
                                 ("L", np.column_stack([-tangent[:, 1], tangent[:, 0]]))):
                out = here + reach * normal
                wall = arena_half_m - np.abs(out).max(axis=1)
                probes = (here[:, None, :] + probe_steps[None, :, None] * normal[:, None, :])
                lines = _distance_to_segments(probes.reshape(-1, 2), seg_starts, seg_ends)
                nearest = np.minimum(wall, lines.reshape(len(samples), -1).min(axis=1))
                clearance[side] = float(nearest.min())
        result.append(clearance)
    return result


def pull_over_sides(
    map_data: dict[str, Any], route: Any, margin_m: float = 0.03, **kwargs: float
) -> str:
    """One pull-over side per route step: 'R' unless the right side comes
    within ``margin_m`` of another line or a wall and the left is clearer."""
    sides = []
    for clearance in pull_over_clearance(map_data, route, **kwargs):
        right_ok = clearance["R"] >= margin_m
        sides.append("R" if right_ok or clearance["R"] >= clearance["L"] else "L")
    return "".join(sides)


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
