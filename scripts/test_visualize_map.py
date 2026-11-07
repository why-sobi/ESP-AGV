import tempfile
import unittest
from pathlib import Path

from visualizeMap import visualize_map


class VisualizeMapTests(unittest.TestCase):
    def test_creates_html_with_edges_nodes_and_interaction(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            map_path = root / "example.json"
            map_path.write_text(
                """{
                  "map_id": "example",
                  "coordinate_system": {"origin": "top_left", "units": "pixels"},
                  "nodes": [
                    {"id": "start", "name": "Start", "x": 1, "y": 2},
                    {"id": "finish", "name": "Finish", "x": 9, "y": 8}
                  ],
                  "edges": [
                    {"id": "road-1", "from": "start", "to": "finish", "distance": 10,
                     "geometry": [[1, 2], [5, 5], [9, 8]]}
                  ]
                }""",
                encoding="utf-8",
            )

            output_path = visualize_map(map_path)
            html = output_path.read_text(encoding="utf-8")

            self.assertEqual(output_path.name, "example.html")
            self.assertIn("road-1", html)
            self.assertIn("start", html)
            self.assertIn("polyline", html)
            self.assertIn("pointermove", html)
            self.assertIn("Reset view", html)

    def test_falls_back_to_edge_between_nodes_without_geometry(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            map_path = root / "straight.json"
            map_path.write_text(
                """{
                  "nodes": [
                    {"id": "a", "x": 0, "y": 0},
                    {"id": "b", "x": 10, "y": 0}
                  ],
                  "edges": [{"id": "ab", "from": "a", "to": "b"}]
                }""",
                encoding="utf-8",
            )

            html = visualize_map(map_path).read_text(encoding="utf-8")
            self.assertIn('points="0.000,0.000 10.000,0.000"', html)

    def test_rejects_bad_geometry(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            map_path = Path(temporary_directory) / "bad.json"
            map_path.write_text(
                """{
                  "nodes": [{"id":"a","x":0,"y":0},{"id":"b","x":1,"y":1}],
                  "edges": [{"id":"ab","from":"a","to":"b","geometry":[[0,0]]}]
                }""",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "at least two points"):
                visualize_map(map_path)


if __name__ == "__main__":
    unittest.main()
