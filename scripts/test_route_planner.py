import unittest
import json
import tempfile
from pathlib import Path

from route_planner import TrackMap, shortest_route


def sample_map() -> TrackMap:
    return TrackMap.from_dict(
        {
            "map_id": "test-track",
            "version": 1,
            "nodes": [{"id": node} for node in ("start", "junction", "loop", "goal", "isolated")],
            "edges": [
                {
                    "id": "approach",
                    "from": "start",
                    "to": "junction",
                    "distance": 2.0,
                    "speed_limit": 3.0,
                },
                {
                    "id": "direct",
                    "from": "junction",
                    "to": "goal",
                    "distance": 8.0,
                },
                {
                    "id": "loop-entry",
                    "from": "junction",
                    "to": "loop",
                    "distance": 1.0,
                },
                {
                    "id": "loop-exit",
                    "from": "loop",
                    "to": "goal",
                    "distance": 1.5,
                    "bidirectional": False,
                },
            ],
        }
    )


class RoutePlannerTests(unittest.TestCase):
    def test_selects_shorter_route_through_loop(self) -> None:
        route = shortest_route(sample_map(), "start", "goal")

        self.assertEqual(route.nodes, ("start", "junction", "loop", "goal"))
        self.assertEqual(
            [step.edge_id for step in route.steps],
            ["approach", "loop-entry", "loop-exit"],
        )
        self.assertEqual(route.total_distance, 4.5)

    def test_respects_one_way_edges(self) -> None:
        one_way_map = TrackMap.from_dict(
            {
                "map_id": "one-way-test",
                "version": 1,
                "nodes": [{"id": "a"}, {"id": "b"}],
                "edges": [
                    {
                        "id": "a-to-b",
                        "from": "a",
                        "to": "b",
                        "distance": 1,
                        "bidirectional": False,
                    }
                ],
            }
        )
        self.assertEqual(shortest_route(one_way_map, "a", "b").nodes, ("a", "b"))
        with self.assertRaisesRegex(ValueError, "No route exists"):
            shortest_route(one_way_map, "b", "a")

    def test_same_start_and_destination_has_empty_route(self) -> None:
        route = shortest_route(sample_map(), "start", "start")

        self.assertEqual(route.nodes, ("start",))
        self.assertEqual(route.steps, ())
        self.assertEqual(route.total_distance, 0.0)

    def test_rejects_unknown_nodes_and_invalid_edges(self) -> None:
        with self.assertRaisesRegex(ValueError, "Unknown start node"):
            shortest_route(sample_map(), "missing", "goal")

        invalid = {
            "map_id": "bad-map",
            "version": 1,
            "nodes": [{"id": "a"}],
            "edges": [{"id": "bad", "from": "a", "to": "missing", "distance": 1}],
        }
        with self.assertRaisesRegex(ValueError, "unknown node"):
            TrackMap.from_dict(invalid)

    def test_rejects_non_positive_distance(self) -> None:
        invalid = {
            "map_id": "bad-map",
            "version": 1,
            "nodes": [{"id": "a"}, {"id": "b"}],
            "edges": [{"id": "bad", "from": "a", "to": "b", "distance": 0}],
        }
        with self.assertRaisesRegex(ValueError, "positive finite"):
            TrackMap.from_dict(invalid)

    def test_route_json_contains_each_edge_in_order(self) -> None:
        route = shortest_route(sample_map(), "start", "goal")

        payload = route.to_dict()

        self.assertEqual(payload["nodes"], ["start", "junction", "loop", "goal"])
        self.assertEqual(
            [step["edge_id"] for step in payload["steps"]],
            ["approach", "loop-entry", "loop-exit"],
        )


if __name__ == "__main__":
    unittest.main()
