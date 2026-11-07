import json
import tempfile
import unittest
from pathlib import Path

from PIL import Image, ImageDraw

from pathToMap import trace_path_map


class PathToMapTests(unittest.TestCase):
    def test_creates_named_map_with_graph_geometry(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            image_path = root / "simple-track.png"
            output_dir = root / "maps"

            image = Image.new("RGB", (160, 120), "white")
            draw = ImageDraw.Draw(image)
            draw.line((20, 60, 140, 60), fill="black", width=9)
            draw.line((80, 20, 80, 60), fill="black", width=9)
            image.save(image_path)

            output_path = trace_path_map(image_path, output_dir, max_dimension=160)
            data = json.loads(output_path.read_text(encoding="utf-8"))

            self.assertEqual(output_path.name, "simple-track.json")
            self.assertEqual(data["map_id"], "simple-track")
            self.assertEqual(data["conversion"]["status"], "draft_requires_visual_review")
            self.assertFalse(data["coordinate_system"]["webots_transform_calibrated"])
            self.assertGreaterEqual(len(data["nodes"]), 3)
            self.assertGreaterEqual(len(data["edges"]), 2)
            self.assertTrue(all(edge["geometry"] for edge in data["edges"]))

    def test_can_convert_edge_lengths_to_meters(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            image_path = root / "straight.png"
            image = Image.new("RGB", (120, 80), "white")
            ImageDraw.Draw(image).line((10, 40, 110, 40), fill="black", width=7)
            image.save(image_path)

            output_path = trace_path_map(
                image_path,
                root / "maps",
                max_dimension=120,
                pixels_per_meter=100,
            )
            data = json.loads(output_path.read_text(encoding="utf-8"))

            self.assertEqual(data["coordinate_system"]["units"], "meters")
            self.assertTrue(data["edges"])
            self.assertTrue(all(0 < edge["distance"] < 2 for edge in data["edges"]))

    def test_rejects_empty_path_image(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            image_path = root / "blank.png"
            Image.new("RGB", (80, 80), "white").save(image_path)

            with self.assertRaisesRegex(ValueError, "no dark path pixels"):
                trace_path_map(image_path, root / "maps")


if __name__ == "__main__":
    unittest.main()
