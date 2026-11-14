import csv
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from scripts.telemetry.prepare_telemetry import prepare_dataset


def write_run(path, run_id, start=0.0, count=80, offset=0.0):
    fields = [
        "run_id",
        "time",
        "target_vel_left",
        "target_vel_right",
        "actual_pos_left",
        "actual_pos_right",
        "gs_left",
        "gs_center",
        "gs_right",
    ]
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        for index in range(count):
            time_value = start + index * 0.032
            writer.writerow(
                {
                    "run_id": run_id,
                    "time": time_value,
                    "target_vel_left": 2.0 + offset,
                    "target_vel_right": 3.0 + offset,
                    "actual_pos_left": index * 0.064,
                    "actual_pos_right": index * 0.096,
                    "gs_left": 300.0 + offset,
                    "gs_center": 310.0 + offset,
                    "gs_right": 320.0 + offset,
                }
            )


class PrepareTelemetryTests(unittest.TestCase):
    def test_splits_by_run_and_fits_scaler_on_training_only(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            train_path = root / "train.csv"
            validation_path = root / "validation.csv"
            write_run(train_path, "train", offset=0.0)
            write_run(validation_path, "validation", offset=100.0)

            metadata = prepare_dataset(
                [train_path, validation_path],
                root / "out",
                window_size=10,
                stride=2,
                minimum_duration=0,
            )

            with np.load(root / "out" / "windows.npz") as arrays:
                self.assertEqual(arrays["motor_train"].shape, (35, 10, 4))
                self.assertEqual(arrays["ir_validation"].shape, (35, 10, 3))
            self.assertEqual([run["run_id"] for run in metadata["train_runs"]], ["train"])
            self.assertEqual(
                [run["run_id"] for run in metadata["validation_runs"]],
                ["validation"],
            )
            self.assertEqual(metadata["scaling"]["motor_mean"][:2], [2.0, 3.0])
            self.assertEqual(metadata["scaling"]["ir_mean"], [300.0, 310.0, 320.0])
            self.assertIsNone(metadata["test_split"])

    def test_detects_appended_episode_after_time_reset(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            combined = root / "combined.csv"
            first = root / "first.csv"
            second = root / "second.csv"
            write_run(first, "episode-a", count=40)
            write_run(second, "episode-b", count=40, offset=10.0)
            with combined.open("w", newline="", encoding="utf-8") as output:
                for source in (first, second):
                    with source.open(newline="", encoding="utf-8") as input_file:
                        output.write(input_file.read() if source == first else "\n".join(input_file.read().splitlines()[1:]) + "\n")

            metadata = prepare_dataset(
                [combined],
                root / "out",
                window_size=5,
                minimum_duration=0,
            )

            run_ids = {
                run["run_id"]
                for run in metadata["train_runs"] + metadata["validation_runs"]
            }
            self.assertEqual(run_ids, {"episode-a", "episode-b"})

    def test_skips_short_episode_and_requires_two_runs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            short = root / "short.csv"
            write_run(short, "short", count=5)

            with self.assertRaisesRegex(ValueError, "No usable telemetry runs"):
                prepare_dataset(
                    [short],
                    root / "out",
                    window_size=3,
                    minimum_duration=10,
                )

    def test_exact_duplicate_run_is_not_placed_in_validation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = root / "run-a.csv"
            duplicate = root / "run-b.csv"
            write_run(first, "run-a", count=40)
            write_run(duplicate, "run-b", count=40)

            metadata = prepare_dataset(
                [first, duplicate],
                root / "out",
                window_size=5,
                minimum_duration=0,
            )

            self.assertEqual(len(metadata["train_runs"]), 1)
            self.assertEqual(metadata["validation_runs"], [])
            self.assertIn("identical to run-a", metadata["excluded_episodes"][0]["reason"])
            with np.load(root / "out" / "windows.npz") as arrays:
                self.assertEqual(arrays["motor_validation"].shape, (0, 5, 4))
                self.assertEqual(arrays["ir_validation"].shape, (0, 5, 3))

    def test_accepts_route_controller_sensor_names(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            route = root / "route.csv"
            fields = [
                "time", "target_vel_left", "target_vel_right",
                "actual_pos_left", "actual_pos_right", "gs_left", "gs_center", "gs_right",
            ]
            for name, offset in (("run-a.csv", 0), ("run-b.csv", 10)):
                with (root / name).open("w", newline="", encoding="utf-8") as file:
                    writer = csv.DictWriter(file, fieldnames=fields)
                    writer.writeheader()
                    for index in range(40):
                        writer.writerow(
                            {
                                "time": index * 0.032,
                                "target_vel_left": 2.0 + offset,
                                "target_vel_right": 2.0 + offset,
                                "actual_pos_left": index * 0.064,
                                "actual_pos_right": index * 0.064,
                                "gs_left": 300 + offset,
                                "gs_center": 300 + offset,
                                "gs_right": 300 + offset,
                            }
                        )
            metadata = prepare_dataset(
                [root / "run-a.csv", root / "run-b.csv"],
                root / "out",
                window_size=5,
                minimum_duration=0,
            )
            self.assertEqual(len(metadata["train_runs"]), 1)
            self.assertEqual(len(metadata["validation_runs"]), 1)

    def test_explicit_split_keeps_assignments_and_creates_test_set(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            paths = {}
            # Name the test run so it would sort first; the explicit split must
            # still keep it out of training.
            for name, offset in (("a-test", 50.0), ("b-train", 0.0), ("c-validation", 100.0)):
                paths[name] = root / f"{name}.csv"
                write_run(paths[name], name, offset=offset)

            metadata = prepare_dataset(
                [],
                root / "out",
                window_size=10,
                stride=2,
                minimum_duration=0,
                split={
                    "train": [paths["b-train"]],
                    "validation": [paths["c-validation"]],
                    "test": [paths["a-test"]],
                },
                split_source="splits/example.json",
            )

            self.assertEqual(metadata["split_method"], "explicit")
            self.assertEqual(metadata["split_source"], "splits/example.json")
            self.assertEqual([r["run_id"] for r in metadata["train_runs"]], ["b-train"])
            self.assertEqual([r["run_id"] for r in metadata["validation_runs"]], ["c-validation"])
            self.assertEqual([r["run_id"] for r in metadata["test_runs"]], ["a-test"])
            self.assertEqual(metadata["run_quality"]["a-test"]["assigned_split"], "test")
            self.assertIsNotNone(metadata["test_split"])
            self.assertEqual(metadata["scaling"]["ir_mean"], [300.0, 310.0, 320.0])
            with np.load(root / "out" / "windows.npz") as arrays:
                self.assertEqual(arrays["motor_test"].shape, (35, 10, 4))
                self.assertEqual(arrays["ir_test"].shape, (35, 10, 3))

    def test_explicit_split_rejects_file_in_two_splits(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            run = root / "run.csv"
            write_run(run, "run")
            with self.assertRaisesRegex(ValueError, "assigned to both train and test"):
                prepare_dataset(
                    [],
                    root / "out",
                    window_size=5,
                    minimum_duration=0,
                    split={"train": [run], "test": [run]},
                )

    def test_fraction_split_writes_empty_test_arrays(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_run(root / "a.csv", "a")
            write_run(root / "b.csv", "b", offset=5.0)
            metadata = prepare_dataset(
                [root / "a.csv", root / "b.csv"],
                root / "out",
                window_size=5,
                minimum_duration=0,
            )
            self.assertEqual(metadata["test_runs"], [])
            with np.load(root / "out" / "windows.npz") as arrays:
                self.assertEqual(arrays["motor_test"].shape, (0, 5, 4))


if __name__ == "__main__":
    unittest.main()
