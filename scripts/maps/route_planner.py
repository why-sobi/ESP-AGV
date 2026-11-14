"""Shortest-path planning over a named, weighted track graph."""

from __future__ import annotations

import argparse
import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Edge:
    edge_id: str
    source: str
    target: str
    distance: float
    speed_limit: float | None
    bidirectional: bool


@dataclass(frozen=True)
class RouteStep:
    edge_id: str
    source: str
    target: str
    speed_limit: float | None


@dataclass(frozen=True)
class Route:
    map_id: str
    map_version: int
    start: str
    destination: str
    nodes: tuple[str, ...]
    steps: tuple[RouteStep, ...]
    total_distance: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "map_id": self.map_id,
            "map_version": self.map_version,
            "start": self.start,
            "destination": self.destination,
            "nodes": list(self.nodes),
            "steps": [
                {
                    "edge_id": step.edge_id,
                    "from": step.source,
                    "to": step.target,
                    "speed_limit": step.speed_limit,
                }
                for step in self.steps
            ],
            "total_distance": self.total_distance,
        }


@dataclass(frozen=True)
class TrackMap:
    map_id: str
    version: int
    nodes: frozenset[str]
    edges: tuple[Edge, ...]

    @classmethod
    def from_dict(cls, data: Any) -> TrackMap:
        if not isinstance(data, dict):
            raise ValueError("Map must be a JSON object.")

        map_id = data.get("map_id")
        version = data.get("version")
        raw_nodes = data.get("nodes")
        raw_edges = data.get("edges")

        if not isinstance(map_id, str) or not map_id.strip():
            raise ValueError("'map_id' must be a non-empty string.")
        if isinstance(version, bool) or not isinstance(version, int) or version < 1:
            raise ValueError("'version' must be a positive integer.")
        if not isinstance(raw_nodes, list) or not raw_nodes:
            raise ValueError("'nodes' must be a non-empty array.")
        if not isinstance(raw_edges, list):
            raise ValueError("'edges' must be an array.")

        nodes: set[str] = set()
        for index, raw_node in enumerate(raw_nodes):
            if not isinstance(raw_node, dict):
                raise ValueError(f"Node at index {index} must be an object.")
            node_id = raw_node.get("id")
            if not isinstance(node_id, str) or not node_id.strip():
                raise ValueError(f"Node at index {index} needs a non-empty 'id'.")
            if node_id in nodes:
                raise ValueError(f"Duplicate node id: {node_id!r}.")
            nodes.add(node_id)

        edges: list[Edge] = []
        edge_ids: set[str] = set()
        for index, raw_edge in enumerate(raw_edges):
            if not isinstance(raw_edge, dict):
                raise ValueError(f"Edge at index {index} must be an object.")

            edge_id = raw_edge.get("id")
            source = raw_edge.get("from")
            target = raw_edge.get("to")
            if not isinstance(edge_id, str) or not edge_id.strip():
                raise ValueError(f"Edge at index {index} needs a non-empty 'id'.")
            if edge_id in edge_ids:
                raise ValueError(f"Duplicate edge id: {edge_id!r}.")
            edge_ids.add(edge_id)
            if (
                not isinstance(source, str)
                or not isinstance(target, str)
                or source not in nodes
                or target not in nodes
            ):
                raise ValueError(
                    f"Edge {edge_id!r} references an unknown node "
                    f"({source!r} -> {target!r})."
                )

            distance = _positive_number(
                raw_edge.get("distance"), f"edge {edge_id!r} distance"
            )
            speed_limit = raw_edge.get("speed_limit")
            if speed_limit is not None:
                speed_limit = _positive_number(
                    speed_limit, f"edge {edge_id!r} speed_limit"
                )

            bidirectional = raw_edge.get("bidirectional", True)
            if not isinstance(bidirectional, bool):
                raise ValueError(f"Edge {edge_id!r} 'bidirectional' must be boolean.")

            edges.append(
                Edge(edge_id, source, target, distance, speed_limit, bidirectional)
            )

        return cls(map_id.strip(), version, frozenset(nodes), tuple(edges))


def _positive_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a positive finite number.")
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise ValueError(f"{label} must be a positive finite number.")
    return number


def load_track_map(path: str | Path) -> TrackMap:
    with Path(path).open(encoding="utf-8") as map_file:
        return TrackMap.from_dict(json.load(map_file))


def shortest_route(track_map: TrackMap, start: str, destination: str) -> Route:
    if start not in track_map.nodes:
        raise ValueError(f"Unknown start node: {start!r}.")
    if destination not in track_map.nodes:
        raise ValueError(f"Unknown destination node: {destination!r}.")

    adjacency: dict[str, list[tuple[str, Edge]]] = {
        node_id: [] for node_id in track_map.nodes
    }
    for edge in track_map.edges:
        adjacency[edge.source].append((edge.target, edge))
        if edge.bidirectional:
            adjacency[edge.target].append((edge.source, edge))

    distances = {start: 0.0}
    previous: dict[str, tuple[str, Edge]] = {}
    queue: list[tuple[float, str]] = [(0.0, start)]

    while queue:
        distance_so_far, node = heapq.heappop(queue)
        if distance_so_far != distances.get(node):
            continue
        if node == destination:
            break

        for neighbor, edge in sorted(
            adjacency[node], key=lambda item: (item[0], item[1].edge_id)
        ):
            candidate_distance = distance_so_far + edge.distance
            if candidate_distance < distances.get(neighbor, math.inf):
                distances[neighbor] = candidate_distance
                previous[neighbor] = (node, edge)
                heapq.heappush(queue, (candidate_distance, neighbor))

    if destination not in distances:
        raise ValueError(f"No route exists from {start!r} to {destination!r}.")

    reversed_nodes = [destination]
    reversed_steps: list[RouteStep] = []
    current = destination
    while current != start:
        previous_node, edge = previous[current]
        reversed_steps.append(
            RouteStep(edge.edge_id, previous_node, current, edge.speed_limit)
        )
        reversed_nodes.append(previous_node)
        current = previous_node

    return Route(
        map_id=track_map.map_id,
        map_version=track_map.version,
        start=start,
        destination=destination,
        nodes=tuple(reversed(reversed_nodes)),
        steps=tuple(reversed(reversed_steps)),
        total_distance=distances[destination],
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plan the shortest route through a named track map."
    )
    parser.add_argument("map_file", type=Path)
    parser.add_argument("start")
    parser.add_argument("destination")
    parser.add_argument(
        "-out",
        type=Path,
        help="Write the route JSON to this file instead of stdout.",
    )
    args = parser.parse_args()

    route = shortest_route(load_track_map(args.map_file), args.start, args.destination)
    route_json = json.dumps(route.to_dict(), indent=2) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(route_json, encoding="utf-8")
        print(f"Route written to {args.out}")
    else:
        print(route_json, end="")


if __name__ == "__main__":
    main()
