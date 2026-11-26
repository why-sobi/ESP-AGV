"""Optuna architecture search for the ESP's detectors, with measured ESP latency.

Each trial trains one detector (``--model``: the motor or IR conv
autoencoder, or the motor GRU) on the healthy training runs, sets its
threshold with the deployed rule (no persistent alarm on any healthy
validation run; the GRU then x --gru-margin), and scores the *tuning* half of
the fault runs. Two objectives, as a Pareto front:

1. detection: tuning runs of the model's own faults detected (motor and
   encoder faults for the motor models, IR faults for the IR model), minus
   0.05 x the median detection delay in seconds (a tie-break between equally
   many detections);
2. latency: the architecture's median per-frame time on the ESP, looked up in
   the table measured by scripts/models/esp_latency.py (no estimate).

The fault runs are split by route and speed: the tuning half is A 2.4, B 3.4,
C 2.6 and D 3.1 rad/s, the held-out half the other speed of each route. The
held-out half and the healthy test runs are only used by --final, which
retrains chosen architectures with the full budget and reports them next to
the deployed models.

Trials train with a reduced budget (training windows at stride 2, at most
--epochs epochs); the deployed architecture is always trial 0, under the same
budget, as the in-search baseline.

    python -m scripts.models.search_architectures --model motor --trials 40
    python -m scripts.models.search_architectures --final
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import optuna
import torch
from numpy.lib.stride_tricks import sliding_window_view
from torch import nn

from scripts.faults.evaluate_faults import primary_model
from scripts.models.esp_latency import CONV_SPACE, GRU_SPACE, conv_key, gru_key, load_table
from scripts.models.gru_experiment import (
    GRUForecaster,
    alarms,
    gru_errors,
    load_faults,
    load_healthy,
    no_alarm_threshold,
    trailing_mean,
    train_gru,
    window_mask,
)

ROOT = Path(__file__).resolve().parents[2]
MODELS = ("motor", "ir")
TUNING = {("A", "2.4"), ("B", "3.4"), ("C", "2.6"), ("D", "3.1")}
K, N, WARMUP_S = 3, 5, 0.5
DEPLOYED = {
    "conv": {"window": 20, "channels": 16, "kernel": 3, "bottleneck": 8, "hidden": 32,
             "lr": 1e-3},
    "gru": {"hidden": 32, "score_window": 62, "lr": 3e-3},
}
GRU_SCORE_WINDOWS = (31, 47, 62, 93)


class ConvAE(nn.Module):
    """The deployed conv autoencoder with the kernel size as a parameter."""

    def __init__(self, window, features, channels, kernel, bottleneck, hidden):
        super().__init__()
        self.window, self.features = window, features
        reduced = (window + 1) // 2
        self.encoder = nn.Sequential(
            nn.Conv1d(features, channels, kernel, padding=kernel // 2),
            nn.ReLU(),
            nn.Conv1d(channels, channels, kernel, stride=2, padding=kernel // 2),
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(channels * reduced, bottleneck),
        )
        self.decoder = nn.Sequential(
            nn.Linear(bottleneck, hidden),
            nn.ReLU(),
            nn.Linear(hidden, window * features),
        )

    def forward(self, x):
        sequence = x.view(-1, self.window, self.features).transpose(1, 2)
        return self.decoder(self.encoder(sequence))


def is_tuning(run):
    info = run["info"]
    return (info["route"], f"{float(info['wheel_speed_rad_s']):g}") in TUNING


def flat_windows(data, window, stride=1):
    windows = sliding_window_view(data, window, axis=0)[::stride]  # (n, F, W)
    return np.ascontiguousarray(windows.transpose(0, 2, 1)).reshape(len(windows), -1)


def train_conv(healthy, name, arch, epochs, patience, train_stride, seed):
    torch.manual_seed(seed)
    window = arch["window"]
    train = torch.from_numpy(np.concatenate(
        [flat_windows(run[name], window, train_stride) for run in healthy["train"]]))
    validation = torch.from_numpy(np.concatenate(
        [flat_windows(run[name], window, 4) for run in healthy["validation"]]))
    features = healthy["train"][0][name].shape[1]
    model = ConvAE(window, features, arch["channels"], arch["kernel"], arch["bottleneck"],
                   arch["hidden"])
    optimizer = torch.optim.Adam(model.parameters(), lr=arch["lr"])
    generator = torch.Generator().manual_seed(seed)
    best, stale, best_epoch = (math.inf, None), 0, 0
    for epoch in range(1, epochs + 1):
        model.train()
        order = torch.randperm(len(train), generator=generator)
        for start in range(0, len(train), 256):
            batch = train[order[start:start + 256]]
            optimizer.zero_grad()
            loss = ((model(batch) - batch) ** 2).mean()
            loss.backward()
            optimizer.step()
        model.eval()
        with torch.no_grad():
            current = float(((model(validation) - validation) ** 2).mean())
        if current < best[0]:
            best, stale, best_epoch = (current, {k: v.clone() for k, v in
                                                 model.state_dict().items()}), 0, epoch
        else:
            stale += 1
            if stale >= patience:
                break
    model.load_state_dict(best[1])
    model.eval()
    return model, {"best_epoch": best_epoch, "validation_loss": best[0]}


def conv_scores(model, run, name, window):
    """(score, mask) per sample: MSE of the window ending there."""
    data = run[name]
    score = np.full(len(data), np.nan)
    if len(data) >= window:
        flat = torch.from_numpy(flat_windows(data, window).astype(np.float32))
        with torch.no_grad():
            score[window - 1:] = ((model(flat) - flat) ** 2).mean(dim=1).numpy()
    return score, window_mask(run, window, WARMUP_S)


def gru_scores(model, run, score_window):
    score = trailing_mean(gru_errors(model, run, "motor"), score_window).max(axis=1)
    return score, window_mask(run, score_window, WARMUP_S)


def evaluate(score_fn, healthy, faults, family, margin=1.0, splits=("validation",)):
    """Threshold on healthy validation runs, then detection per fault run of
    ``family`` ('motor' or 'ir') and healthy alarms per split."""
    threshold = no_alarm_threshold([score_fn(run) for run in healthy["validation"]], K, N)
    threshold *= margin
    healthy_alarms = {split: sum(bool(alarms(*score_fn(run), threshold, K, N).any())
                                 for run in healthy[split]) for split in splits}
    per_run = []
    for run in faults:
        if primary_model(run["info"]["fault_type"]) != family:
            continue
        series = alarms(*score_fn(run), threshold, K, N)
        times, onset = run["times"], run["onset"]
        post = np.flatnonzero(series & (times >= onset))
        per_run.append({
            "fault": run["info"]["fault_name"], "file": run["file"],
            "detected": bool(len(post)),
            "delay_s": float(times[post[0]] - onset) if len(post) else None,
            "pre_onset": bool((series & (times < onset)).any()),
        })
    delays = [r["delay_s"] for r in per_run if r["detected"]]
    detected = sum(r["detected"] for r in per_run)
    return {
        "threshold": threshold,
        "detected": detected,
        "runs": len(per_run),
        "median_delay_s": float(np.median(delays)) if delays else None,
        "pre_onset_runs": sum(r["pre_onset"] for r in per_run),
        "healthy_alarm_runs": healthy_alarms,
        "score": detected - 0.05 * (float(np.median(delays)) if delays else 0.0)
                 - sum(r["pre_onset"] for r in per_run),
        "per_run": per_run,
    }


def load_data():
    config = json.loads((ROOT / "models/esp-conv-v1/config.json").read_text(encoding="utf-8"))
    mean = {m: np.asarray(config["models"][m]["scaler_mean"]) for m in MODELS}
    scale = {m: np.asarray(config["models"][m]["scaler_scale"]) for m in MODELS}
    healthy = load_healthy(ROOT / "data/datasets/splits/esp-runs-v1.json", mean, scale)
    faults = load_faults(ROOT / "data/esp_runs/faults/manifest.csv", mean, scale)
    return healthy, faults


def build_and_score(kind, name, arch, healthy, faults, args, final=False):
    """Train one architecture and evaluate it; returns (model, result)."""
    family = "ir" if name == "ir" else "motor"
    splits = ("validation", "test") if final else ("validation",)
    if kind == "conv":
        model, info = train_conv(healthy, name, arch, args.final_epochs if final else args.epochs,
                                 args.patience * (3 if final else 1),
                                 1 if final else args.train_stride, args.seed)
        result = evaluate(lambda run: conv_scores(model, run, name, arch["window"]),
                          healthy, faults, family, splits=splits)
    else:
        torch.manual_seed(args.seed)
        model, info = train_gru(healthy, "motor", arch["hidden"], 256, 32, 32,
                                args.final_iterations if final else args.iterations,
                                arch["lr"], 100, 8 if final else 4)
        info = {"best_iteration": info["best_iteration"],
                "validation_loss": info["best_validation_loss"]}
        result = evaluate(lambda run: gru_scores(model, run, arch["score_window"]),
                          healthy, faults, family, margin=args.gru_margin, splits=splits)
    result["training"] = info
    return model, result


def suggest(trial, kind, features):
    if kind == "gru":
        return {"hidden": trial.suggest_categorical("hidden", GRU_SPACE["hidden"]),
                "score_window": trial.suggest_categorical("score_window", GRU_SCORE_WINDOWS),
                "lr": trial.suggest_float("lr", 1e-3, 1e-2, log=True)}
    return {
        "window": trial.suggest_categorical("window", CONV_SPACE["window"]),
        "channels": trial.suggest_categorical("channels", CONV_SPACE["channels"]),
        "kernel": trial.suggest_categorical("kernel", CONV_SPACE["kernel"]),
        "bottleneck": trial.suggest_categorical("bottleneck", CONV_SPACE["bottleneck"]),
        "hidden": trial.suggest_categorical("hidden", CONV_SPACE["hidden"]),
        "lr": trial.suggest_float("lr", 3e-4, 3e-3, log=True),
    }


def latency_key(kind, name, arch):
    if kind == "gru":
        return gru_key(2, arch["hidden"], 2)
    features = 3 if name == "ir" else 4
    return conv_key(features, arch["window"], arch["channels"], arch["kernel"],
                    arch["bottleneck"], arch["hidden"])


def search(args):
    kind = "gru" if args.model == "gru" else "conv"
    name = "motor" if args.model == "gru" else args.model
    healthy, faults = load_data()
    tuning = [run for run in faults if is_tuning(run)]
    table = load_table(args.latency)
    out = args.out / args.model
    out.mkdir(parents=True, exist_ok=True)
    study = optuna.create_study(
        study_name=args.model, storage=f"sqlite:///{(out / 'study.db').as_posix()}",
        load_if_exists=True, directions=["maximize", "minimize"],
        sampler=optuna.samplers.TPESampler(seed=args.seed, multivariate=True))
    if len(study.trials) == 0:
        study.enqueue_trial(DEPLOYED[kind])

    def objective(trial):
        arch = suggest(trial, kind, 3 if name == "ir" else 4)
        latency = table[latency_key(kind, name, arch)]
        _, result = build_and_score(kind, name, arch, healthy, tuning, args)
        for key in ("detected", "runs", "median_delay_s", "pre_onset_runs", "threshold"):
            trial.set_user_attr(key, result[key])
        trial.set_user_attr("latency_us", latency["median_us"])
        trial.set_user_attr("weight_bytes", latency["weight_bytes"])
        trial.set_user_attr("training", result["training"])
        print(f"trial {trial.number}: {arch} -> detected {result['detected']}/{result['runs']}, "
              f"median delay {result['median_delay_s']}, {latency['median_us']} us", flush=True)
        return result["score"], latency["median_us"]

    torch.set_num_threads(args.threads)
    study.optimize(objective, n_trials=args.trials)
    rows = []
    for trial in study.trials:
        if trial.state != optuna.trial.TrialState.COMPLETE:
            continue
        rows.append({"number": trial.number, "params": trial.params,
                     "score": trial.values[0], "latency_us": trial.values[1],
                     **trial.user_attrs,
                     "pareto": trial.number in {t.number for t in study.best_trials}})
    (out / "trials.json").write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
    print(f"Pareto front ({len(study.best_trials)}):")
    for trial in sorted(study.best_trials, key=lambda t: t.values[1]):
        print(f"  #{trial.number}: score {trial.values[0]:.2f}, {trial.values[1]:.0f} us, "
              f"{trial.params}")


def deployed_score_fn(kind, name):
    """Score function of the deployed model (saved weights, as on the ESP)."""
    from scripts.models.train_autoencoder import ConvAutoencoder

    if kind == "gru":
        config = json.loads((ROOT / "models/esp-gru-v1/config.json").read_text(encoding="utf-8"))
        model = GRUForecaster(2, 2, int(config["hidden"]))
        model.load_state_dict(torch.load(ROOT / "models/esp-gru-v1/gru_motor.pt",
                                         weights_only=True))
        model.eval()
        window = int(config["deploy"]["motor_gru"]["score_window_samples"])
        return lambda run: gru_scores(model, run, window)
    config = json.loads((ROOT / "models/esp-conv-v1/config.json").read_text(encoding="utf-8"))
    info, architecture = config["models"][name], config["architecture"]
    window = int(config["window_size_samples"])
    model = ConvAutoencoder(window, len(info["features"]), architecture["hidden"],
                            architecture["bottleneck"])
    model.load_state_dict(torch.load(ROOT / "models/esp-conv-v1" / info["weights"],
                                     weights_only=True))
    model.eval()
    return lambda run: conv_scores(model, run, name, window)


def final(args):
    """Retrain chosen architectures with the full budget; report them and the
    deployed models on the held-out faults and the healthy test runs."""
    chosen = json.loads(Path(args.final).read_text(encoding="utf-8"))
    healthy, faults = load_data()
    held_out = [run for run in faults if not is_tuning(run)]
    table = load_table(args.latency)
    torch.set_num_threads(args.threads)
    report = {"held_out_runs": len(held_out), "models": {}}
    for model_name in args.final_models:
        kind = "gru" if model_name == "gru" else "conv"
        name = "motor" if model_name == "gru" else model_name
        family = "ir" if name == "ir" else "motor"
        entries = []
        deployed = evaluate(deployed_score_fn(kind, name), healthy, held_out, family,
                            margin=args.gru_margin if kind == "gru" else 1.0,
                            splits=("validation", "test"))
        entries.append({"label": "deployed", "arch": DEPLOYED[kind],
                        "latency_us": table[latency_key(kind, name, DEPLOYED[kind])]["median_us"],
                        "weight_bytes": table[latency_key(kind, name,
                                                          DEPLOYED[kind])]["weight_bytes"],
                        **{k: v for k, v in deployed.items() if k != "per_run"}})
        for index, arch in enumerate(chosen.get(model_name, [])):
            model, result = build_and_score(kind, name, arch, healthy, held_out, args, final=True)
            out = args.out / model_name / f"final-{index}"
            out.mkdir(parents=True, exist_ok=True)
            torch.save(model.state_dict(), out / "weights.pt")
            key = latency_key(kind, name, arch)
            entries.append({"label": f"candidate {index}", "arch": arch,
                            "latency_us": table[key]["median_us"],
                            "weight_bytes": table[key]["weight_bytes"],
                            **{k: v for k, v in result.items() if k != "per_run"}})
            (out / "result.json").write_text(json.dumps(entries[-1], indent=2, default=str) + "\n",
                                             encoding="utf-8")
            print(model_name, entries[-1]["label"], arch, "->", result["detected"], "/",
                  result["runs"], result["healthy_alarm_runs"], flush=True)
        report["models"][model_name] = entries
        (args.out / f"final_{model_name}.json").write_text(
            json.dumps(entries, indent=2, default=str) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", choices=("motor", "ir", "gru"))
    parser.add_argument("--trials", type=int, default=40)
    parser.add_argument("--final", type=Path, default=None,
                        help="JSON {model: [architecture, ...]} to retrain and report")
    parser.add_argument("--final-models", nargs="+", default=["motor", "ir", "gru"],
                        help="which models of --final to run (for parallel runs)")
    parser.add_argument("--latency", type=Path, default=ROOT / "reports/esp-latency-v1/latency.csv")
    parser.add_argument("--out", type=Path, default=ROOT / "reports/arch-search-v1")
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--final-epochs", type=int, default=300)
    parser.add_argument("--patience", type=int, default=6)
    parser.add_argument("--train-stride", type=int, default=2)
    parser.add_argument("--iterations", type=int, default=2000)
    parser.add_argument("--final-iterations", type=int, default=4000)
    parser.add_argument("--gru-margin", type=float, default=2.0)
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    if args.final:
        final(args)
    elif args.model:
        search(args)
    else:
        parser.error("give --model to search or --final to report")


if __name__ == "__main__":
    main()
