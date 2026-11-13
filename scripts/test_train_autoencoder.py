import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from train_autoencoder import persistence_alarms, train_all


def write_dataset(root, windows=40, window_size=5):
    rng = np.random.default_rng(0)
    arrays = {}
    split_runs = {}
    for split in ("train", "validation", "test"):
        arrays[f"motor_{split}"] = rng.normal(size=(windows, window_size, 4)).astype(np.float32)
        arrays[f"ir_{split}"] = rng.normal(size=(windows, window_size, 3)).astype(np.float32)
        split_runs[split] = [
            {"run_id": f"{split}-a", "window_start": 0, "window_count": windows // 2},
            {"run_id": f"{split}-b", "window_start": windows // 2, "window_count": windows // 2},
        ]
    for split in ("train", "validation", "test"):
        arrays[f"{split}_window_start_seconds"] = np.tile(np.arange(windows // 2) * 0.1, 2)
    np.savez(root / "windows.npz", **arrays)
    metadata = {
        "split_source": "splits/example.json",
        "window_size_samples": window_size,
        "motor_features": ["a", "b", "c", "d"],
        "ir_features": ["l", "c", "r"],
        "scaling": {"motor_mean": [0] * 4, "motor_scale": [1] * 4,
                    "ir_mean": [0] * 3, "ir_scale": [1] * 3},
        "train_runs": split_runs["train"],
        "validation_runs": split_runs["validation"],
        "test_runs": split_runs["test"],
    }
    (root / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")


class PersistenceTests(unittest.TestCase):
    def test_k_of_n_window(self):
        exceed = [1, 0, 1, 0, 0, 0, 1, 1, 1, 0]
        alarms = persistence_alarms(exceed, k=2, n=3)
        self.assertEqual(alarms.tolist(),
                         [False, False, True, False, False, False, False, True, True, True])

    def test_requires_k_inside_trailing_window(self):
        self.assertFalse(persistence_alarms([1, 0, 0, 0, 1], k=2, n=3).any())


class TrainAutoencoderTests(unittest.TestCase):
    def test_trains_both_models_and_writes_outputs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset = root / "dataset"
            dataset.mkdir()
            write_dataset(dataset)

            config = train_all(dataset, root / "model", hidden=8, bottleneck=2,
                               epochs=3, patience=2, plots=False)

            for name, input_size in (("motor", 20), ("ir", 15)):
                info = config["models"][name]
                self.assertTrue((root / "model" / f"{name}.pt").exists())
                self.assertEqual(info["input_size"], input_size)
                self.assertEqual(set(info["results"]), {"dense_autoencoder", "pca_baseline"})
                result = info["results"]["dense_autoencoder"]
                # 0.5 s warm-up at 0.1 s spacing leaves 15 of 20 windows per run.
                self.assertEqual([r["monitored_windows"] for r in result["test_runs"]], [15, 15])
                self.assertEqual(info["threshold"], result["threshold"])
                self.assertEqual(len(result["test_runs"]), 2)
                self.assertLessEqual(result["test_window_exceedance_rate"], 1.0)
            saved = json.loads((root / "model" / "config.json").read_text(encoding="utf-8"))
            self.assertEqual(saved["dataset_split_source"], "splits/example.json")

    def test_conv_architecture_trains_and_reconstructs_window_shape(self):
        import torch

        from train_autoencoder import ConvAutoencoder

        model = ConvAutoencoder(window_size=5, features=4, hidden=8, bottleneck=2)
        self.assertEqual(tuple(model(torch.zeros(3, 20)).shape), (3, 20))
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset = root / "dataset"
            dataset.mkdir()
            write_dataset(dataset)
            config = train_all(dataset, root / "model", arch="conv", hidden=8,
                               bottleneck=2, epochs=2, patience=2, plots=False)
            self.assertIn("conv_autoencoder", config["models"]["ir"]["results"])

    def test_threshold_is_validation_percentile(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset = root / "dataset"
            dataset.mkdir()
            write_dataset(dataset)
            config = train_all(dataset, root / "model", hidden=8, bottleneck=2, epochs=2,
                               patience=2, threshold_percentile=100.0, plots=False)
            result = config["models"]["motor"]["results"]["pca_baseline"]
            self.assertAlmostEqual(result["threshold"], result["scores"]["validation"]["max"])


if __name__ == "__main__":
    unittest.main()
