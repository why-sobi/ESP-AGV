"""Offline experiment: pair the conv autoencoders with long-memory detectors.

Reads whole runs (not the 20-sample windows in windows.npz): the healthy ESP
runs of a split file and the labelled fault runs of a fault manifest. Every
detector produces one motor and one IR score per window end sample, gated
exactly like the conv models (window start before --warmup-s, or any window
sample in a commanded manoeuvre, is unmonitored), with a k-of-n persistence
alarm and a threshold set by the no-validation-alarm rule.

Detectors:
- ``conv``: the deployed conv autoencoders (mean squared error over the window).
- ``conv-max``: the same reconstruction, scored per feature and taking the
  largest feature (a fault on one wheel or sensor is not averaged away).
- ``diff``: per feature, the mean squared sample-to-sample difference over the
  window (scaled units); largest feature. No training.
- ``stuck`` (IR only): a sensor whose value has not moved by --stuck-eps for
  --stuck-samples frames while the robot drives (mean wheel speed above
  --stuck-speed). No training. (Requiring another sensor to move instead
  missed a sensor stuck at the on-the-line value: off the track all three
  sensors read flat floor.)
- ``gru``: stateful GRU forecasters run one step per frame from the start of a
  run. Motor: a plant model that predicts both encoder wheel speeds at t from
  the commanded wheel targets only (no measured speeds as input, so a weak
  wheel cannot be hidden by copying its last value). IR: predicts the three
  sensors at t from the sensors, wheel targets and wheel speeds at t-1.
  Score: per output feature, the mean squared prediction error over the last
  --gru-window samples; largest feature.

Pairs (conv OR detector) are what the ESP would run. Each member keeps its own
no-validation-alarm threshold, so a pair also raises no alarm on healthy
validation runs. The report gives, per detector and pair, false alarms on the
healthy test runs and, per fault, detection by the primary model (motor and
encoder faults: motor; IR faults: IR) and by either model.

Outputs (in --out): summary.md, results.json. GRU weights and config go to
--model-out (--reuse-weights evaluates saved GRU weights without training).
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import torch
from numpy.lib.stride_tricks import sliding_window_view
from torch import nn

from scripts.faults.evaluate_faults import load_detectors, primary_model
from scripts.models.train_autoencoder import persistence_alarms, seed_everything
from scripts.telemetry.prepare_telemetry import (
    _features,
    _read_segments,
    load_split,
    unmonitored_samples,
    window_monitored,
)

ROOT = Path(__file__).resolve().parents[2]
MODELS = ("motor", "ir")
CONV_WINDOW = 20


class GRUForecaster(nn.Module):
    """One-layer GRU with a linear head; one step per frame on the ESP."""

    def __init__(self, inputs, outputs, hidden):
        super().__init__()
        self.gru = nn.GRU(inputs, hidden, batch_first=True)
        self.head = nn.Linear(hidden, outputs)

    def forward(self, x, state=None):
        out, state = self.gru(x, state)
        return self.head(out), state


def load_run(path, rows, run_id, mean, scale):
    """Scaled motor/IR features per derived sample plus gating and fault labels."""
    motor, ir, times, _ = _features(rows, run_id, CONV_WINDOW)
    active = np.asarray([row.get("fault_active") == "1" for row in rows[1:]])
    return {
        "run_id": run_id,
        "file": path.name,
        "raw_ir": ir,
        "raw_motor": motor,
        "motor": ((motor - mean["motor"]) / scale["motor"]).astype(np.float32),
        "ir": ((ir - mean["ir"]) / scale["ir"]).astype(np.float32),
        "times": times,
        "unmonitored": unmonitored_samples(rows),
        "onset": float(times[active.argmax()]) if active.any() else None,
    }


def load_healthy(split_path, mean, scale):
    split = load_split(split_path)
    runs = {}
    for name, paths in split.items():
        runs[name] = []
        for path in paths:
            for run_id, rows in _read_segments(Path(path), 10.0)[0]:
                runs[name].append(load_run(Path(path), rows, run_id, mean, scale))
    return runs


def load_faults(manifest, mean, scale):
    runs = []
    with manifest.open(newline="", encoding="utf-8") as file:
        for info in csv.DictReader(file):
            if info["outcome"] == "serial_error":
                continue
            path = manifest.parent / info["file"]
            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            run = load_run(path, rows, path.stem, mean, scale)
            run["info"] = info
            runs.append(run)
    return runs


# ---------------------------------------------------------------- gating


def window_mask(run, window, warmup_s):
    """Per sample t: True when the window of `window` samples ending at t is monitored."""
    count = len(run["times"])
    mask = np.zeros(count, bool)
    if count < window:
        return mask
    starts_ok = window_monitored(run["unmonitored"], window)
    starts_ok &= run["times"][: count - window + 1] >= warmup_s
    mask[window - 1:] = starts_ok
    return mask


def trailing_mean(values, window):
    """values: (N, F). Per sample t, the mean of rows t-window+1..t (NaN before)."""
    out = np.full(values.shape, np.nan)
    if len(values) >= window:
        out[window - 1:] = sliding_window_view(values, window, axis=0).mean(axis=2)
    return out


# ---------------------------------------------------------------- detectors


def conv_scores(run, reconstruct, name):
    """Per sample: (mean, per-feature max) reconstruction error of the window ending there."""
    data = run[name]
    count, features = data.shape
    mean_score = np.full(count, np.nan)
    max_score = np.full(count, np.nan)
    windows = sliding_window_view(data, CONV_WINDOW, axis=0).transpose(0, 2, 1)
    flat = windows.reshape(len(windows), -1).astype(np.float32)
    error = (reconstruct(flat) - flat) ** 2
    per_feature = error.reshape(len(windows), CONV_WINDOW, features).mean(axis=1)
    mean_score[CONV_WINDOW - 1:] = error.mean(axis=1)
    max_score[CONV_WINDOW - 1:] = per_feature.max(axis=1)
    return mean_score, max_score


def diff_score(run, name, window):
    data = run[name]
    squared = np.vstack([np.zeros((1, data.shape[1])), np.diff(data, axis=0) ** 2])
    return trailing_mean(squared, window).max(axis=1)


def stuck_score(run, samples, eps, speed):
    """1.0 where a sensor is flat for `samples` differences while the robot
    drives (mean wheel speed over the same samples + 1 frames above `speed`,
    rad/s), else 0."""
    ir = run["raw_ir"]
    count = len(ir)
    score = np.zeros(count)
    if count <= samples:
        return score
    change = sliding_window_view(np.abs(np.diff(ir, axis=0)), samples, axis=0).max(axis=2)
    wheel = np.abs(run["raw_motor"][:, 2:]).mean(axis=1)
    driving = sliding_window_view(wheel, samples + 1).mean(axis=1) > speed
    score[samples:] = (change < eps).any(axis=1) & driving
    return score


def gru_io(run, name):
    """(inputs, targets) per sample for the forecaster of model `name`."""
    motor = run["motor"]
    if name == "motor":
        # Plant model: wheel targets at t -> encoder wheel speeds at t.
        return motor[:, :2], motor[:, 2:]
    ir = run["ir"]
    previous = np.vstack([np.zeros((1, 7), np.float32), np.hstack([ir, motor])[:-1]])
    return previous, ir


def gru_errors(model, run, name):
    inputs, targets = gru_io(run, name)
    with torch.no_grad():
        predicted, _ = model(torch.from_numpy(inputs)[None])
    return (predicted[0].numpy() - targets) ** 2


def train_gru(runs, name, hidden, chunk, burn_in, batch_size, iterations, lr, eval_every,
              patience):
    pairs = [gru_io(run, name) for run in runs["train"]]
    validation = runs["validation"]
    inputs, outputs = pairs[0][0].shape[1], pairs[0][1].shape[1]
    model = GRUForecaster(inputs, outputs, hidden)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    rng = np.random.default_rng(0)
    lengths = np.asarray([len(x) for x, _ in pairs])
    weights = (lengths - chunk).clip(min=1) / (lengths - chunk).clip(min=1).sum()

    def validation_loss():
        model.eval()
        losses = [gru_errors(model, run, name).mean() for run in validation]
        model.train()
        return float(np.mean(losses))

    best = (float("inf"), None, 0)
    history = []
    stale = 0
    for step in range(1, iterations + 1):
        batch_x, batch_y = [], []
        for run_index in rng.choice(len(pairs), batch_size, p=weights):
            x, y = pairs[run_index]
            start = int(rng.integers(0, max(1, len(x) - chunk)))
            batch_x.append(x[start:start + chunk])
            batch_y.append(y[start:start + chunk])
        x = torch.from_numpy(np.stack(batch_x))
        y = torch.from_numpy(np.stack(batch_y))
        optimizer.zero_grad()
        predicted, _ = model(x)
        # Chunks start mid-run from a zero state: skip the burn-in steps.
        loss = ((predicted[:, burn_in:] - y[:, burn_in:]) ** 2).mean()
        loss.backward()
        nn.utils.clip_grad_norm_(model.parameters(), 1.0)
        optimizer.step()
        if step % eval_every == 0:
            current = validation_loss()
            history.append({"iteration": step, "train": float(loss), "validation": current})
            print(f"  {name} GRU iter {step}: train {float(loss):.5f} validation {current:.5f}",
                  flush=True)
            if current < best[0]:
                best = (current, {k: v.clone() for k, v in model.state_dict().items()}, step)
                stale = 0
            else:
                stale += 1
                if stale >= patience:
                    break
    model.load_state_dict(best[1])
    model.eval()
    return model, {"best_iteration": best[2], "best_validation_loss": best[0],
                   "history": history}


# ---------------------------------------------------------------- thresholds and alarms


def no_alarm_threshold(score_mask_pairs, k, n):
    """Smallest threshold at which no run raises a k-of-n alarm (scores >= 0)."""
    worst = 0.0
    for scores, mask in score_mask_pairs:
        kept = np.where(mask, np.nan_to_num(scores), 0.0)
        padded = np.concatenate([np.zeros(n - 1), kept])
        spans = np.sort(sliding_window_view(padded, n), axis=1)
        if len(spans):
            worst = max(worst, float(spans[:, -k].max()))
    return float(np.nextafter(worst, np.inf))


def alarms(scores, mask, threshold, k, n):
    exceed = mask & (np.nan_to_num(scores) > threshold)
    return persistence_alarms(exceed, k, n)


def compute_scores(run, detectors, args):
    """{detector: {model: (scores, mask)}} for one run."""
    out = {}
    masks = {w: window_mask(run, w, args.warmup_s)
             for w in (CONV_WINDOW, args.gru_window, args.stuck_samples + 1)}
    out["conv"], out["conv-max"] = {}, {}
    for name in MODELS:
        mean_score, max_score = conv_scores(run, detectors["conv"][name][0], name)
        out["conv"][name] = (mean_score, masks[CONV_WINDOW])
        out["conv-max"][name] = (max_score, masks[CONV_WINDOW])
    out["diff"] = {name: (diff_score(run, name, CONV_WINDOW), masks[CONV_WINDOW])
                   for name in MODELS}
    out["stuck"] = {"ir": (stuck_score(run, args.stuck_samples, args.stuck_eps,
                                       args.stuck_speed), masks[args.stuck_samples + 1])}
    out["gru"] = {}
    for name in MODELS:
        error = trailing_mean(gru_errors(detectors["gru"][name], run, name), args.gru_window)
        out["gru"][name] = (error.max(axis=1), masks[args.gru_window])
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--conv-model", default="models/esp-conv-v1")
    parser.add_argument("--split", default="data/datasets/splits/esp-runs-v1.json")
    parser.add_argument("--manifest", type=Path, default=ROOT / "data/esp_runs/faults/manifest.csv")
    parser.add_argument("--model-out", type=Path, default=ROOT / "models/esp-gru-v1")
    parser.add_argument("--out", type=Path, default=ROOT / "reports/esp-gru-eval-v1")
    parser.add_argument("--reuse-weights", action="store_true")
    parser.add_argument("--hidden", type=int, default=32)
    parser.add_argument("--chunk", type=int, default=256)
    parser.add_argument("--burn-in", type=int, default=32)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--iterations", type=int, default=4000)
    parser.add_argument("--eval-every", type=int, default=100)
    parser.add_argument("--patience", type=int, default=8)
    parser.add_argument("--lr", type=float, default=3e-3)
    parser.add_argument("--gru-window", type=int, default=62,
                        help="samples averaged into the GRU score (62 = ~2 s)")
    parser.add_argument("--gru-margin", type=float, default=2.0,
                        help="factor on the GRU threshold for the 'xM' pair")
    parser.add_argument("--stuck-samples", type=int, default=31, help="~1 s at 32 ms")
    parser.add_argument("--stuck-eps", type=float, default=0.5, help="raw sensor units")
    parser.add_argument("--stuck-speed", type=float, default=1.0,
                        help="mean wheel speed (rad/s) above which the robot counts as driving")
    parser.add_argument("--persist-k", type=int, default=3)
    parser.add_argument("--persist-n", type=int, default=5)
    parser.add_argument("--warmup-s", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    k, n = args.persist_k, args.persist_n
    seed_everything(args.seed)

    conv_dir = ROOT / args.conv_model
    conv_config = json.loads((conv_dir / "config.json").read_text(encoding="utf-8"))
    mean = {m: np.asarray(conv_config["models"][m]["scaler_mean"]) for m in MODELS}
    scale = {m: np.asarray(conv_config["models"][m]["scaler_scale"]) for m in MODELS}
    conv = next(v for key, v in load_detectors([conv_dir])[0].items() if key != "pca_baseline")

    print("Loading runs...", flush=True)
    healthy = load_healthy(ROOT / args.split, mean, scale)
    faults = load_faults(args.manifest.resolve(), mean, scale)
    print({split: len(runs) for split, runs in healthy.items()}, "fault runs:", len(faults))

    args.model_out.mkdir(parents=True, exist_ok=True)
    gru = {}
    gru_info = {}
    for name in MODELS:
        torch.manual_seed(args.seed)
        weights = args.model_out / f"gru_{name}.pt"
        if args.reuse_weights:
            inputs, outputs = (x.shape[1] for x in gru_io(healthy["train"][0], name))
            gru[name] = GRUForecaster(inputs, outputs, args.hidden)
            gru[name].load_state_dict(torch.load(weights, weights_only=True))
            gru[name].eval()
        else:
            gru[name], gru_info[name] = train_gru(
                healthy, name, args.hidden, args.chunk, args.burn_in, args.batch_size,
                args.iterations, args.lr, args.eval_every, args.patience)
            torch.save(gru[name].state_dict(), weights)
    if not args.reuse_weights:
        (args.model_out / "config.json").write_text(json.dumps({
            "architecture": "GRU(1 layer) + Linear head, stateful, reset at route start",
            "hidden": args.hidden,
            "motor": {"inputs": ["target_vel_left(t)", "target_vel_right(t)"],
                      "outputs": ["actual_vel_left(t)", "actual_vel_right(t)"]},
            "ir": {"inputs": ["gs_left/center/right(t-1)", "target_vel_left/right(t-1)",
                              "actual_vel_left/right(t-1)"],
                   "outputs": ["gs_left(t)", "gs_center(t)", "gs_right(t)"]},
            "scaler": {m: {"mean": mean[m].tolist(), "scale": scale[m].tolist()} for m in MODELS},
            "scaler_source": args.conv_model,
            "training": {k_: getattr(args, k_.replace("-", "_")) for k_ in
                         ("chunk", "burn_in", "batch_size", "iterations", "lr", "seed")},
            "results": gru_info,
        }, indent=2) + "\n", encoding="utf-8")

    detectors = {"conv": conv, "gru": gru}
    print("Scoring...", flush=True)
    scored = {split: [compute_scores(run, detectors, args) for run in runs]
              for split, runs in healthy.items()}
    fault_scores = [compute_scores(run, detectors, args) for run in faults]

    # Thresholds: no persistent alarm on any healthy validation run.
    thresholds = {}
    for detector, models in scored["validation"][0].items():
        thresholds[detector] = {}
        for name in models:
            if detector == "stuck":
                thresholds[detector][name] = 0.5
            else:
                thresholds[detector][name] = no_alarm_threshold(
                    [s[detector][name] for s in scored["validation"]], k, n)
    # The GRU with a safety factor on its no-validation-alarm threshold.
    thresholds["gru-margin"] = {name: value * args.gru_margin
                                for name, value in thresholds["gru"].items()}

    # What the firmware deploys (scripts/models/export_c.py reads this block).
    config_path = args.model_out / "config.json"
    model_config = json.loads(config_path.read_text(encoding="utf-8"))
    model_config["deploy"] = {
        "motor_gru": {"threshold_no_validation_alarm": thresholds["gru"]["motor"],
                      "margin": args.gru_margin,
                      "threshold": thresholds["gru-margin"]["motor"],
                      "score_window_samples": args.gru_window,
                      "score": "per wheel, mean squared scaled prediction error over the "
                               "score window; the larger wheel"},
        "ir_stuck": {"samples": args.stuck_samples, "eps": args.stuck_eps,
                     "speed": args.stuck_speed,
                     "rule": "a sensor whose sample-to-sample change (raw units) stays below "
                             "eps for `samples` differences while the mean wheel speed "
                             "(|left|+|right|)/2 over the same samples+1 frames exceeds "
                             "`speed` (rad/s)"},
        "persistence": {"k": k, "n": n}, "warmup_s": args.warmup_s,
    }
    config_path.write_text(json.dumps(model_config, indent=2) + "\n", encoding="utf-8")

    def alarm_series(scores, detector, name):
        score, mask = scores[detector.removesuffix("-margin")][name]
        return alarms(score, mask, thresholds[detector][name], k, n)

    methods = {
        "conv": ("conv",), "conv-max": ("conv-max",), "diff": ("diff",), "stuck": ("stuck",),
        "gru": ("gru",),
        "conv + gru": ("conv", "gru"),
        "conv + diff": ("conv", "diff"),
        "conv + stuck": ("conv", "stuck"),
        "conv + gru + stuck": ("conv", "gru", "stuck"),
        "conv-max + gru + stuck": ("conv-max", "gru", "stuck"),
        # Members "detector:model" use that detector for one model only.
        "conv + motor gru + stuck": ("conv", "gru:motor", "stuck"),
        f"conv + motor gru x{args.gru_margin:g} + stuck": ("conv", "gru-margin:motor", "stuck"),
    }

    def method_alarms(scores, members, name, length):
        result = np.zeros(length, bool)
        for member in members:
            detector, _, only = member.partition(":")
            if only and only != name:
                continue
            if name in scores[detector.removesuffix("-margin")]:
                result |= alarm_series(scores, detector, name)
        return result

    report = {"settings": {k_: (str(v) if isinstance(v, Path) else v)
                           for k_, v in vars(args).items()},
              "thresholds": thresholds, "methods": {}}
    fault_names = list(dict.fromkeys(run["info"]["fault_name"] for run in faults))
    for method, members in methods.items():
        entry = {"members": list(members), "healthy": {}, "faults": {}}
        for split in ("validation", "test"):
            entry["healthy"][split] = {
                name: sum(bool(method_alarms(s, members, name, len(r["times"])).any())
                          for s, r in zip(scored[split], healthy[split]))
                for name in MODELS
            }
            entry["healthy"][split]["any"] = sum(
                any(method_alarms(s, members, name, len(r["times"])).any() for name in MODELS)
                for s, r in zip(scored[split], healthy[split]))
            entry["healthy"][split]["runs"] = len(healthy[split])
        for fault in fault_names:
            rows = []
            for run, scores in zip(faults, fault_scores):
                if run["info"]["fault_name"] != fault:
                    continue
                times, onset = run["times"], run["onset"]
                primary = primary_model(run["info"]["fault_type"])
                series = {name: method_alarms(scores, members, name, len(times))
                          for name in MODELS}
                either = series["motor"] | series["ir"]
                post = np.flatnonzero(series[primary] & (times >= onset))
                rows.append({
                    "file": run["file"],
                    "primary_detected": bool(len(post)),
                    "delay_s": float(times[post[0]] - onset) if len(post) else None,
                    "any_detected": bool((either & (times >= onset)).any()),
                    "pre_onset_alarm": bool((either & (times < onset)).any()),
                    # Any monitored conv window ending after onset: otherwise the
                    # run ends inside a manoeuvre and no detector can see it.
                    "observable": bool(
                        (window_mask(run, CONV_WINDOW, args.warmup_s) & (times >= onset)).any()),
                })
            delays = [r["delay_s"] for r in rows if r["primary_detected"]]
            entry["faults"][fault] = {
                "runs": len(rows),
                "observable": sum(r["observable"] for r in rows),
                "primary_detected": sum(r["primary_detected"] for r in rows),
                "any_detected": sum(r["any_detected"] for r in rows),
                "pre_onset_alarm_runs": sum(r["pre_onset_alarm"] for r in rows),
                "median_delay_s": float(np.median(delays)) if delays else None,
                "per_run": rows,
            }
        report["methods"][method] = entry

    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "results.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    table = markdown(report, fault_names)
    (args.out / "summary.md").write_text(table + "\n", encoding="utf-8")
    print(table)
    print(f"Saved to {args.out}")


def markdown(report, fault_names):
    methods = report["methods"]
    lines = ["## Healthy runs with a persistent alarm (motor / IR / either)", "",
             "| Method | Validation | Test |", "|---|---|---|"]
    for method, entry in methods.items():
        cells = []
        for split in ("validation", "test"):
            h = entry["healthy"][split]
            cells.append(f"{h['motor']} / {h['ir']} / {h['any']} of {h['runs']}")
        lines.append(f"| {method} | {cells[0]} | {cells[1]} |")
    lines += ["", "## Fault detection by the primary model (runs detected; "
              "observable = runs with a monitored window after onset)", ""]
    header = "| Fault | observable | " + " | ".join(methods) + " |"
    lines += [header, "|" + "---|" * (len(methods) + 2)]
    for fault in fault_names:
        first = next(iter(methods.values()))["faults"][fault]
        cells = [f"{m['faults'][fault]['primary_detected']}/{m['faults'][fault]['runs']}"
                 for m in methods.values()]
        lines.append(f"| {fault} | {first['observable']}/{first['runs']} | "
                     + " | ".join(cells) + " |")
    lines += ["", "## Fault detection by either model (what stops the ESP)", "",
              header, "|" + "---|" * (len(methods) + 2)]
    for fault in fault_names:
        first = next(iter(methods.values()))["faults"][fault]
        cells = [f"{m['faults'][fault]['any_detected']}/{m['faults'][fault]['runs']}"
                 for m in methods.values()]
        lines.append(f"| {fault} | {first['observable']}/{first['runs']} | "
                     + " | ".join(cells) + " |")
    lines += ["", "## Median detection delay, primary model (s)", "",
              "| Fault | " + " | ".join(methods) + " |", "|" + "---|" * (len(methods) + 1)]
    for fault in fault_names:
        cells = []
        for m in methods.values():
            d = m["faults"][fault]["median_delay_s"]
            cells.append("-" if d is None else f"{d:.2f}")
        lines.append(f"| {fault} | " + " | ".join(cells) + " |")
    pre = {method: sum(f["pre_onset_alarm_runs"] for f in entry["faults"].values())
           for method, entry in methods.items()}
    lines += ["", "Fault runs with an alarm before onset: "
              + ", ".join(f"{m} {v}" for m, v in pre.items())]
    return "\n".join(lines)


if __name__ == "__main__":
    main()
