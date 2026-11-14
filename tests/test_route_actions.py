import unittest

from scripts.esp.route_actions import (
    format_route_command,
    route_action_plan,
    route_actions,
)
from scripts.maps.route_planner import TrackMap, shortest_route


def build_map(outgoing):
    return {
        "map_id": "turn-test",
        "version": 1,
        "world_transform": {
            "world_origin": [0.0, 0.0],
            "world_units_per_pixel": [1.0, 1.0],
        },
        "nodes": [
            {"id": "start", "x": 0.0, "y": 0.0},
            {"id": "junction", "x": 0.0, "y": 1.0},
            {"id": "goal", "x": outgoing[0], "y": outgoing[1]},
        ],
        "edges": [
            {
                "id": "approach",
                "from": "start",
                "to": "junction",
                "distance": 1.0,
                "geometry": [[0.0, 0.0], [0.0, 1.0]],
            },
            {
                "id": "departure",
                "from": "junction",
                "to": "goal",
                "distance": 1.0,
                "geometry": [[0.0, 1.0], [outgoing[0], outgoing[1]]],
            },
        ],
    }


class RouteActionTests(unittest.TestCase):
    def test_converts_left_right_and_straight_turns(self):
        for endpoint, expected in (
            ((-1.0, 1.0), "LD"),
            ((1.0, 1.0), "RD"),
            ((0.0, 2.0), "SD"),
        ):
            with self.subTest(expected=expected):
                map_data = build_map(endpoint)
                route = shortest_route(
                    TrackMap.from_dict(map_data), "start", "goal"
                )
                self.assertEqual(route_actions(map_data, route), expected)

    def test_handles_a_route_traversing_an_edge_backwards(self):
        map_data = build_map((-1.0, 1.0))
        map_data["edges"][0]["from"], map_data["edges"][0]["to"] = (
            "junction",
            "start",
        )
        map_data["edges"][0]["geometry"].reverse()
        route = shortest_route(TrackMap.from_dict(map_data), "start", "goal")

        self.assertEqual(route_actions(map_data, route), "LD")

    def test_direct_route_only_sends_destination_action(self):
        map_data = build_map((0.0, 1.0))
        map_data["nodes"] = [
            {"id": "start", "x": 0.0, "y": 0.0},
            {"id": "goal", "x": 0.0, "y": 1.0},
        ]
        direct_edge = dict(map_data["edges"][0])
        direct_edge["to"] = "goal"
        map_data["edges"] = [direct_edge]
        route = shortest_route(TrackMap.from_dict(map_data), "start", "goal")

        self.assertEqual(route_actions(map_data, route), "D")

    def test_plan_gives_cumulative_distance_to_each_junction(self):
        map_data = build_map((-1.0, 1.0))
        map_data["world_transform"]["world_units_per_pixel"] = [0.5, 0.5]
        route = shortest_route(TrackMap.from_dict(map_data), "start", "goal")

        plan = route_action_plan(map_data, route)

        self.assertEqual([action for action, _ in plan], ["L", "D"])
        self.assertAlmostEqual(plan[0][1], 0.5)
        self.assertAlmostEqual(plan[1][1], 1.0)

    def test_plan_scales_map_distance_to_odometry(self):
        map_data = build_map((0.0, 2.0))
        route = shortest_route(TrackMap.from_dict(map_data), "start", "goal")

        plan = route_action_plan(map_data, route, distance_scale=0.9)

        self.assertEqual(format_route_command(plan), "S@0.900,D@1.800")

    def test_plan_measures_reversed_edges(self):
        map_data = build_map((1.0, 1.0))
        map_data["edges"][1]["from"], map_data["edges"][1]["to"] = "goal", "junction"
        map_data["edges"][1]["geometry"] = [[1.0, 1.0], [0.5, 1.0], [0.0, 1.0]]
        route = shortest_route(TrackMap.from_dict(map_data), "start", "goal")

        self.assertEqual(format_route_command(route_action_plan(map_data, route)), "R@1.000,D@2.000")

    def test_plan_rejects_non_positive_scale(self):
        map_data = build_map((0.0, 2.0))
        route = shortest_route(TrackMap.from_dict(map_data), "start", "goal")

        with self.assertRaises(ValueError):
            route_action_plan(map_data, route, distance_scale=0.0)


if __name__ == "__main__":
    unittest.main()
