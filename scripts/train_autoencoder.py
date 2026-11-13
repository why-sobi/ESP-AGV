"""Train motor and ground-IR autoencoders on healthy telemetry windows.

Each model is trained on the dataset's training runs only, early-stopped on
validation loss, and given an anomaly threshold from the validation
reconstruction-error distribution. Healthy test runs then measure the false
alarm rate, per window and with a k-of-n persistence rule. A PCA model with the
same bottleneck size is evaluated identically as a linear baseline.

Windows starting before --warmup-s (the spawn/settling transient) are not
monitored: they are excluded from thresholds, rates and alarms, but still used
for training.

Architectures (--arch): ``dense`` (Linear/ReLU only) and ``conv`` (a 1D
convolutional encoder with the same dense decoder). Both use only layers that
are straightforward to port to plain C.

Outputs (in --out): <model>.pt state dicts, config.json (architecture,
threshold, scaler, metrics) and <model>_scores.png plots.
"""

import argparse
import json
import random
from pathlib import Path

import numpy as np
import torch
from torch import nn

MODELS = ("motor", "ir")
SPLITS = ("train", "validation", "test")


class DenseAutoencoder(nn.Module):
    """Flattened-window autoencoder built only from Linear and ReLU layers."""

    def __init__(self, input_size, hidden, bottleneck):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Linear(input_size, hidden),
            nn.ReLU(),
            nn.Linear(hidden, bottleneck),
        )
        self.decoder = nn.Sequential(
            nn.Linear(bottleneck, hidden),
            nn.ReLU(),
            nn.Linear(hidden, input_size),
        )

    def forward(self, x):
        return self.decoder(self.encoder(x))


class ConvAutoencoder(nn.Module):
    """1D-conv encoder over time, dense decoder; takes flattened windows."""

    def __init__(self, window_size, features, hidden, bottleneck, channels=16):
        super().__init__()
        self.window_size = window_size
        self.features = features
        reduced = (window_size + 1) // 2
        self.encoder = nn.Sequential(
            nn.Conv1d(features, channels, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.Conv1d(channels, channels, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(channels * reduced, bottleneck),
        )
        self.decoder = nn.Sequential(
            nn.Linear(bottleneck, hidden),
            nn.ReLU(),
            nn.Linear(hidden, window_size * features),
        )

    def forward(self, x):
        # Flattened input is time-major [t0 f0, t0 f1, ...]; Conv1d wants
        # [batch, features, time].
        sequence = x.view(-1, self.window_size, self.features).transpose(1, 2)
        return self.decoder(self.encoder(sequence))


def build_model(arch, window_size, features, hidden, bottleneck):
    if arch == "dense":
        return DenseAutoencoder(window_size * features, hidden, bottleneck)
    if arch == "conv":
        return ConvAutoencoder(window_size, features, hidden, bottleneck)
    raise ValueError(f"Unknown architecture: {arch}")


ARCH_LAYERS = {
    "dense": "Linear(in,hidden) ReLU Linear(hidden,bottleneck) "
             "Linear(bottleneck,hidden) ReLU Linear(hidden,in)",
    "conv": "reshape [time,features]->[features,time]; Conv1d(features,16,k3,p1) ReLU "
            "Conv1d(16,16,k3,s2,p1) ReLU Flatten Linear(16*ceil(time/2),bottleneck) "
            "Linear(bottleneck,hidden) ReLU Linear(hidden,in)",
}


def seed_everything(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.use_deterministic_algorithms(True)


def load_dataset(dataset_dir):
    dataset_dir = Path(dataset_dir)
    metadata = json.loads((dataset_dir / "metadata.json").read_text(encoding="utf-8"))
    arrays = {}
    starts = {}
    with np.load(dataset_dir / "windows.npz") as data:
        for split in SPLITS:
            for model in MODELS:
                key = f"{model}_{split}"
                if key not in data:
                    raise ValueError(f"{dataset_dir} has no {key}; rebuild it with --split")
                arrays[key] = data[key].astype(np.float32)
            starts[split] = data[f"{split}_window_start_seconds"]
    if len(arrays["motor_test"]) == 0 or len(arrays["motor_validation"]) == 0:
        raise ValueError("Dataset needs non-empty validation and test splits.")
    return metadata, arrays, starts


def flatten(windows):
    return windows.reshape(len(windows), -1)


def window_scores(reconstruct, flat):
    """Mean squared reconstruction error per window, in scaled units."""
    return np.mean((reconstruct(flat) - flat) ** 2, axis=1)


def persistence_alarms(exceed, k, n):
    """alarm[i] is True when at least k of the last n windows exceed."""
    exceed = np.asarray(exceed, dtype=np.int32)
    counts = np.convolve(exceed, np.ones(n, dtype=np.int32))[: len(exceed)]
    return counts >= k


def run_slices(split_runs):
    return [
        (run["run_id"], slice(run["window_start"], run["window_start"] + run["window_count"]))
        for run in split_runs
    ]


def evaluate(scores, threshold, test_runs, test_start_seconds, k, n, warmup_s=0.0):
    """False-alarm statistics on healthy test runs (warm-up windows unmonitored)."""
    monitored = test_start_seconds >= warmup_s
    exceed = (scores["test"] > threshold) & monitored
    per_run = []
    for run_id, window_slice in run_slices(test_runs):
        run_exceed = exceed[window_slice]
        alarms = persistence_alarms(run_exceed, k, n)
        times = test_start_seconds[window_slice]
        per_run.append(
            {
                "run_id": run_id,
                "windows": int(len(run_exceed)),
                "monitored_windows": int(monitored[window_slice].sum()),
                "windows_over_threshold": int(run_exceed.sum()),
                "persistent_alarms": int(alarms.sum()),
                "first_alarm_window_start_s": (
                    float(times[np.argmax(alarms)]) if alarms.any() else None
                ),
                "exceedance_window_start_s": [float(t) for t in times[run_exceed]][:20],
            }
        )
    return {
        "test_window_exceedance_rate": float(exceed[monitored].mean()),
        "test_persistent_alarm_windows": int(sum(r["persistent_alarms"] for r in per_run)),
        "test_runs_with_persistent_alarm": int(
            sum(r["persistent_alarms"] > 0 for r in per_run)
        ),
        "test_runs": per_run,
    }


def score_summary(values):
    return {
        "mean": float(np.mean(values)),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "max": float(np.max(values)),
    }


def train_model(flat, arch, window_size, features, hidden, bottleneck, epochs, patience,
                batch_size, lr):
    train = torch.from_numpy(flat["train"])
    validation = torch.from_numpy(flat["validation"])
    model = build_model(arch, window_size, features, hidden, bottleneck)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss()
    generator = torch.Generator().manual_seed(0)

    best = (float("inf"), None, 0)
    history = []
    stale = 0
    for epoch in range(1, epochs + 1):
        model.train()
        order = torch.randperm(len(train), generator=generator)
        total = 0.0
        for start in range(0, len(train), batch_size):
            batch = train[order[start : start + batch_size]]
            optimizer.zero_grad()
            loss = loss_fn(model(batch), batch)
            loss.backward()
            optimizer.step()
            total += loss.item() * len(batch)
        model.eval()
        with torch.no_grad():
            validation_loss = loss_fn(model(validation), validation).item()
        history.append({"epoch": epoch, "train": total / len(train), "validation": validation_loss})
        if validation_loss < best[0]:
            best = (validation_loss, {k: v.clone() for k, v in model.state_dict().items()}, epoch)
            stale = 0
        else:
            stale += 1
            if stale >= patience:
                break
    model.load_state_dict(best[1])
    model.eval()

    def reconstruct(x):
        with torch.no_grad():
            return model(torch.from_numpy(x)).numpy()

    return model, reconstruct, {"best_epoch": best[2], "epochs_run": len(history), "history": history}


def fit_pca(train_flat, components):
    mean = train_flat.mean(axis=0)
    _, _, vt = np.linalg.svd(train_flat - mean, full_matrices=False)
    basis = vt[:components]

    def reconstruct(x):
        return (x - mean) @ basis.T @ basis + mean

    return reconstruct


def plot_scores(path, title, scores, monitored, threshold, test_runs, test_start_seconds,
                warmup_s):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, (histogram, timeline) = plt.subplots(1, 2, figsize=(12, 4))
    kept = {split: scores[split][monitored[split]] for split in SPLITS}
    floor = max(min(float(s.min()) for s in kept.values()), 1e-6)
    bins = np.logspace(np.log10(floor), np.log10(max(float(s.max()) for s in kept.values())), 60)
    for split in SPLITS:
        histogram.hist(kept[split], bins=bins, histtype="step", density=True, label=split)
    histogram.axvline(threshold, color="black", linestyle="--", label="threshold")
    histogram.set_xscale("log")
    histogram.set_xlabel("window reconstruction MSE (scaled units)")
    histogram.set_ylabel("density")
    histogram.legend()
    histogram.set_title(f"{title}: score distribution")

    for run_id, window_slice in run_slices(test_runs):
        timeline.plot(test_start_seconds[window_slice], scores["test"][window_slice],
                      linewidth=1, label=run_id.split("#")[0])
    timeline.axhline(threshold, color="black", linestyle="--")
    timeline.axvspan(0, warmup_s, color="grey", alpha=0.2, label="warm-up (unmonitored)")
    timeline.set_yscale("log")
    timeline.set_xlabel("window start (s)")
    timeline.set_ylabel("MSE")
    timeline.set_title(f"{title}: healthy test runs")
    timeline.legend(fontsize=7)
    figure.tight_layout()
    figure.savefig(path, dpi=120)
    plt.close(figure)


def train_all(dataset, out, arch="dense", hidden=32, bottleneck=8, epochs=300, patience=30,
              batch_size=64, lr=1e-3, seed=0, threshold_percentile=99.5,
              persist_k=3, persist_n=5, warmup_s=0.5, plots=True):
    seed_everything(seed)
    metadata, arrays, starts = load_dataset(dataset)
    window_size = int(metadata["window_size_samples"])
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    monitored = {split: starts[split] >= warmup_s for split in SPLITS}
    if not monitored["validation"].any() or not monitored["test"].any():
        raise ValueError(f"No validation/test windows start after warm-up {warmup_s} s.")

    config = {
        "dataset": Path(dataset).as_posix(),
        "dataset_split_source": metadata.get("split_source"),
        "window_size_samples": window_size,
        "architecture": {
            "type": f"{arch}_autoencoder",
            "layers": ARCH_LAYERS[arch],
            "hidden": hidden,
            "bottleneck": bottleneck,
            "input_layout": "window flattened time-major: [t0 f0, t0 f1, ..., t1 f0, ...]",
        },
        "training": {"epochs_max": epochs, "patience": patience, "batch_size": batch_size,
                     "learning_rate": lr, "seed": seed, "optimizer": "Adam", "loss": "MSE"},
        "threshold_rule": {
            "score": "mean squared reconstruction error over the scaled window",
            "threshold": f"{threshold_percentile} percentile of monitored validation scores",
            "persistence": f"alarm when >= {persist_k} of the last {persist_n} windows exceed",
            "warmup_s": warmup_s,
            "warmup": "windows starting before warmup_s are not monitored",
        },
        "models": {},
    }

    for name in MODELS:
        flat = {split: flatten(arrays[f"{name}_{split}"]) for split in SPLITS}
        features = arrays[f"{name}_train"].shape[2]
        torch.manual_seed(seed)
        model, reconstruct, train_info = train_model(
            flat, arch, window_size, features, hidden, bottleneck, epochs, patience,
            batch_size, lr,
        )
        torch.save(model.state_dict(), out / f"{name}.pt")

        results = {}
        for method, recon in ((f"{arch}_autoencoder", reconstruct),
                               ("pca_baseline", fit_pca(flat["train"], bottleneck))):
            scores = {split: window_scores(recon, flat[split]) for split in SPLITS}
            threshold = float(np.percentile(
                scores["validation"][monitored["validation"]], threshold_percentile
            ))
            results[method] = {
                "threshold": threshold,
                "scores": {
                    split: score_summary(scores[split][monitored[split]]) for split in SPLITS
                },
                **evaluate(scores, threshold, metadata["test_runs"], starts["test"],
                           persist_k, persist_n, warmup_s),
            }
            if plots and method != "pca_baseline":
                plot_scores(out / f"{name}_scores.png", f"{name} {arch} AE", scores,
                            monitored, threshold, metadata["test_runs"], starts["test"],
                            warmup_s)

        config["models"][name] = {
            "weights": f"{name}.pt",
            "features": metadata[f"{name}_features"],
            "input_size": int(flat["train"].shape[1]),
            "parameters": int(sum(p.numel() for p in model.parameters())),
            "scaler_mean": metadata["scaling"][f"{name}_mean"],
            "scaler_scale": metadata["scaling"][f"{name}_scale"],
            "threshold": results[f"{arch}_autoencoder"]["threshold"],
            "best_epoch": train_info["best_epoch"],
            "epochs_run": train_info["epochs_run"],
            "final_train_loss": train_info["history"][train_info["best_epoch"] - 1]["train"],
            "final_validation_loss": train_info["history"][train_info["best_epoch"] - 1]["validation"],
            "results": results,
        }

    (out / "config.json").write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", default="data/datasets/sim-baseline-v4")
    parser.add_argument("--out", default="models/dense-v1")
    parser.add_argument("--arch", choices=sorted(ARCH_LAYERS), default="dense")
    parser.add_argument("--hidden", type=int, default=32)
    parser.add_argument("--bottleneck", type=int, default=8)
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--patience", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--threshold-percentile", type=float, default=99.5)
    parser.add_argument("--persist-k", type=int, default=3)
    parser.add_argument("--persist-n", type=int, default=5)
    parser.add_argument("--warmup-s", type=float, default=0.5)
    args = parser.parse_args()

    config = train_all(
        args.dataset, args.out, arch=args.arch, hidden=args.hidden,
        bottleneck=args.bottleneck, epochs=args.epochs, patience=args.patience,
        batch_size=args.batch_size, lr=args.lr, seed=args.seed,
        threshold_percentile=args.threshold_percentile,
        persist_k=args.persist_k, persist_n=args.persist_n, warmup_s=args.warmup_s,
    )
    for name, info in config["models"].items():
        print(f"\n{name}: {info['parameters']} parameters, best epoch "
              f"{info['best_epoch']}/{info['epochs_run']}, "
              f"validation loss {info['final_validation_loss']:.4f}")
        for method, result in info["results"].items():
            print(f"  {method:18s} threshold {result['threshold']:.4f} | "
                  f"test exceedance {100 * result['test_window_exceedance_rate']:.2f}% | "
                  f"persistent alarms {result['test_persistent_alarm_windows']} windows in "
                  f"{result['test_runs_with_persistent_alarm']}/{len(result['test_runs'])} runs")
    print(f"\nSaved to {args.out}")


if __name__ == "__main__":
    main()
