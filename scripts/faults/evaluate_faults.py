"""Evaluate trained autoencoders (and the PCA baseline) on labelled fault runs.

For every fault run, windows are built exactly as in prepare_telemetry.py,
scaled with the model's training scaler and scored. An alarm is raised when at
least k of the last n monitored windows exceed the model's threshold (windows
starting before the warm-up are not monitored), matching train_autoencoder.py.

Per run and model this reports:
- pre-onset false alarm: an alarm whose window ends before the fault onset;
- detection: the first alarm whose window ends at or after the onset, and the
  delay from onset to the end of that window.

Motor and encoder faults are expected to be caught by the motor model, IR
faults by the IR model; alarms of the other model are reported as
cross-coupling.
"""

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
import torch

from scripts.telemetry.prepare_telemetry import _features
from scripts.models.train_autoencoder import (
    MODELS,
    build_model,
    fit_pca,
    flatten,
    load_dataset,
    persistence_alarms,
    window_scores,
)

ROOT = Path(__file__).resolve().parents[2]
PRIMARY = {"motor": "motor", "encoder": "motor", "ir": "ir"}


def primary_model(fault_type):
    return PRIMARY[fault_type.split("_")[0]]


def load_detectors(model_dirs):
    """Return {method: {model name: (reconstruct, threshold, mean, scale)}} and window size."""
    detectors = {}
    window_size = None
    for model_dir in model_dirs:
        model_dir = Path(model_dir)
        config = json.loads((model_dir / "config.json").read_text(encoding="utf-8"))
        arch = config["architecture"]["type"].removesuffix("_autoencoder")
        window_size = window_size or config["window_size_samples"]
        if config["window_size_samples"] != window_size:
            raise ValueError("All models must use the same window size.")
        method = f"{model_dir.name} ({arch})"
        detectors[method] = {}
        for name in MODELS:
            info = config["models"][name]
            features = len(info["features"])
            network = build_model(arch, window_size, features,
                                  config["architecture"]["hidden"],
                                  config["architecture"]["bottleneck"])
            network.load_state_dict(torch.load(model_dir / info["weights"], weights_only=True))
            network.eval()

            def reconstruct(x, network=network):
                with torch.no_grad():
                    return network(torch.from_numpy(x)).numpy()

            detectors[method][name] = (
                reconstruct, info["threshold"],
                np.asarray(info["scaler_mean"]), np.asarray(info["scaler_scale"]),
            )

        if "pca_baseline" not in detectors:
            # Same dataset/bottleneck for every model dir; refit PCA once.
            dataset = ROOT / config["dataset"]
            _, arrays, _ = load_dataset(dataset)
            detectors["pca_baseline"] = {}
            for name in MODELS:
                info = config["models"][name]
                detectors["pca_baseline"][name] = (
                    fit_pca(flatten(arrays[f"{name}_train"]), config["architecture"]["bottleneck"]),
                    info["results"]["pca_baseline"]["threshold"],
                    np.asarray(info["scaler_mean"]), np.asarray(info["scaler_scale"]),
                )
    # Put the baseline last for readability.
    detectors["pca_baseline"] = detectors.pop("pca_baseline")
    return detectors, window_size


def run_windows(path, window_size):
    """Return motor/IR windows (unscaled), window start/end times and the onset time."""
    with path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    motor, ir, times, _ = _features(rows, path.stem, window_size)
    active = [row.get("fault_active") == "1" for row in rows[1:]]
    onset = next((t for t, a in zip(times, active) if a), None)
    count = len(times) - window_size + 1
    index = np.arange(count)[:, None] + np.arange(window_size)[None, :]
    return {"motor": motor[index], "ir": ir[index]}, times[:count], times[window_size - 1:], onset


def evaluate_run(windows, starts, ends, onset, detectors, k, n, warmup_s):
    results = {}
    for name, (reconstruct, threshold, mean, scale) in detectors.items():
        flat = flatten(((windows[name] - mean) / scale).astype(np.float32))
        scores = window_scores(reconstruct, flat)
        exceed = (scores > threshold) & (starts >= warmup_s)
        alarms = persistence_alarms(exceed, k, n)
        pre = alarms & (ends < onset) if onset is not None else alarms
        post = np.flatnonzero(alarms & (ends >= onset)) if onset is not None else []
        results[name] = {
            "pre_onset_false_alarm": bool(pre.any()),
            "pre_onset_alarm_windows": int(pre.sum()),
            "detected": bool(len(post)),
            "delay_s": float(ends[post[0]] - onset) if len(post) else None,
            "scores": scores,
        }
    return results


def summarize(records, methods, faults):
    summary = {}
    for method in methods:
        summary[method] = {}
        for fault in faults:
            rows = [r for r in records if r["fault_name"] == fault]
            primary = primary_model(rows[0]["fault_type"])
            other = "ir" if primary == "motor" else "motor"
            delays = [r["results"][method][primary]["delay_s"] for r in rows
                      if r["results"][method][primary]["detected"]]
            summary[method][fault] = {
                "fault_type": rows[0]["fault_type"],
                "target": rows[0].get("target", ""),
                "severity": rows[0].get("severity", ""),
                "runs": len(rows),
                "primary_model": primary,
                "detected": sum(r["results"][method][primary]["detected"] for r in rows),
                "median_delay_s": float(np.median(delays)) if delays else None,
                "max_delay_s": float(np.max(delays)) if delays else None,
                "cross_model_detected": sum(r["results"][method][other]["detected"] for r in rows),
                "pre_onset_false_alarm_runs": sum(
                    any(r["results"][method][m]["pre_onset_false_alarm"] for m in MODELS)
                    for r in rows
                ),
                "stopped_early_runs": sum(r["outcome"] == "stopped_early" for r in rows),
            }
    return summary


def markdown_table(summary):
    lines = []
    for method, faults in summary.items():
        lines += [f"### {method}", "",
                  "| Fault | Primary model | Detected | Median delay (s) | Max delay (s) "
                  "| Other model fired | Runs with pre-onset false alarm | Runs stopped early |",
                  "|---|---|---|---|---|---|---|---|"]
        for fault, s in faults.items():
            median = "-" if s["median_delay_s"] is None else f"{s['median_delay_s']:.2f}"
            longest = "-" if s["max_delay_s"] is None else f"{s['max_delay_s']:.2f}"
            lines.append(
                f"| {fault} | {s['primary_model']} | {s['detected']}/{s['runs']} | {median} "
                f"| {longest} | {s['cross_model_detected']}/{s['runs']} "
                f"| {s['pre_onset_false_alarm_runs']}/{s['runs']} "
                f"| {s['stopped_early_runs']}/{s['runs']} |"
            )
        lines.append("")
    return "\n".join(lines)


def severity_axis(fault_type, severity):
    """Map a severity to an x value where larger always means a worse fault."""
    if fault_type == "motor_gain":
        return 100.0 * (1.0 - severity), "power loss (% of commanded speed)"
    return severity, f"{fault_type} severity"


def plot_sweeps(out_dir, summary):
    """For each fault type/target tested at several severities, plot detection
    rate and median delay against severity for every method."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    groups = defaultdict(set)
    for faults in summary.values():
        for s in faults.values():
            if s["severity"] not in ("", None):
                groups[(s["fault_type"], s["target"])].add(float(s["severity"]))
    written = []
    for (fault_type, target), severities in groups.items():
        if len(severities) < 2:
            continue
        figure, (rate_axis, delay_axis) = plt.subplots(1, 2, figsize=(11, 4))
        for method, faults in summary.items():
            points = sorted(
                (severity_axis(fault_type, float(s["severity"]))[0],
                 s["detected"] / s["runs"], s["median_delay_s"])
                for s in faults.values()
                if s["fault_type"] == fault_type and s["target"] == target
            )
            x = [p[0] for p in points]
            rate_axis.plot(x, [100 * p[1] for p in points], marker="o", label=method)
            delay_axis.plot([p[0] for p in points if p[2] is not None],
                            [p[2] for p in points if p[2] is not None], marker="o", label=method)
        label = severity_axis(fault_type, 1.0)[1]
        rate_axis.set_xlabel(label)
        rate_axis.set_ylabel("runs detected (%)")
        rate_axis.set_ylim(-5, 105)
        rate_axis.set_title(f"{fault_type} on {target}: detection rate")
        rate_axis.legend(fontsize=8)
        delay_axis.set_xlabel(label)
        delay_axis.set_ylabel("median detection delay (s)")
        delay_axis.set_title(f"{fault_type} on {target}: delay (detected runs)")
        figure.tight_layout()
        path = out_dir / f"sweep_{fault_type}_{target}.png"
        figure.savefig(path, dpi=120)
        plt.close(figure)
        written.append(path)
    return written


def plot_faults(path, records, method, detectors, faults):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    columns = 5
    rows_count = (len(faults) + columns - 1) // columns
    figure, axes = plt.subplots(rows_count, columns, figsize=(4 * columns, 3.2 * rows_count),
                                squeeze=False)
    for axis, fault in zip(axes.flat, faults):
        runs = [r for r in records if r["fault_name"] == fault]
        primary = primary_model(runs[0]["fault_type"])
        for record in runs:
            if record["onset"] is None:
                continue
            scores = record["results"][method][primary]["scores"]
            axis.plot(record["ends"] - record["onset"], scores, linewidth=0.8)
        axis.axhline(detectors[method][primary][1], color="black", linestyle="--", linewidth=1)
        axis.axvline(0, color="red", linewidth=1)
        axis.set_yscale("log")
        axis.set_title(f"{fault}\n({primary} model)", fontsize=9)
        axis.set_xlabel("time from onset (s)", fontsize=8)
    for axis in list(axes.flat)[len(faults):]:
        axis.set_visible(False)
    figure.suptitle(f"{method}: primary-model score (dashed = threshold, red = onset)")
    figure.tight_layout()
    figure.savefig(path, dpi=110)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--models", nargs="+", default=["models/dense-v1", "models/conv-v1"])
    parser.add_argument("--manifest", type=Path, default=ROOT / "data/fault_runs/manifest.csv")
    parser.add_argument("--out", type=Path, default=ROOT / "reports/fault-eval-v1")
    parser.add_argument("--persist-k", type=int, default=3)
    parser.add_argument("--persist-n", type=int, default=5)
    parser.add_argument("--warmup-s", type=float, default=0.5)
    args = parser.parse_args()
    args.manifest = args.manifest.resolve()
    args.out = args.out.resolve()

    detectors, window_size = load_detectors([ROOT / m for m in args.models])
    with args.manifest.open(newline="", encoding="utf-8") as file:
        runs = list(csv.DictReader(file))
    records = []
    for run in runs:
        path = args.manifest.parent / run["file"]
        windows, starts, ends, onset = run_windows(path, window_size)
        record = dict(run, onset=onset, ends=ends, results={})
        for method, method_detectors in detectors.items():
            record["results"][method] = evaluate_run(
                windows, starts, ends, onset, method_detectors,
                args.persist_k, args.persist_n, args.warmup_s,
            )
        records.append(record)

    faults = list(dict.fromkeys(r["fault_name"] for r in records))
    summary = summarize(records, list(detectors), faults)
    args.out.mkdir(parents=True, exist_ok=True)
    per_run = [
        {k: v for k, v in r.items() if k not in ("ends", "results")}
        | {"results": {m: {n: {k: v for k, v in res.items() if k != "scores"}
                           for n, res in models.items()}
                       for m, models in r["results"].items()}}
        for r in records
    ]
    (args.out / "results.json").write_text(
        json.dumps({"settings": {"models": args.models, "persist_k": args.persist_k,
                                 "persist_n": args.persist_n, "warmup_s": args.warmup_s,
                                 "manifest": str(args.manifest.relative_to(ROOT))},
                    "summary": summary, "runs": per_run}, indent=2) + "\n",
        encoding="utf-8",
    )
    table = markdown_table(summary)
    (args.out / "summary.md").write_text(table + "\n", encoding="utf-8")
    for method in detectors:
        safe = method.split(" ")[0]
        plot_faults(args.out / f"{safe}_fault_scores.png", records, method, detectors, faults)
    for path in plot_sweeps(args.out, summary):
        print(f"Severity sweep plot: {path}")
    print(table)
    print(f"Saved to {args.out}")


if __name__ == "__main__":
    main()
