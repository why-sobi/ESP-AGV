"""Quantize the deployed detectors with esp-ppq and compare them with float32.

Runs in the separate esp-ppq environment (esp-ppq needs Python 3.11-3.12):

    uv venv --python 3.12 .venv-ppq
    uv pip install --python .venv-ppq\\Scripts\\python.exe "esp-ppq[cpu]"
    .venv-ppq\\Scripts\\python.exe -m scripts.models.quantize_int8

For the motor and IR conv autoencoders (models/esp-conv-v1) and the motor GRU
(models/esp-gru-v1) this:

- quantizes each model for the ESP32-S3 with ESP-DL's rules (w8a8 = INT8, and
  w16a16 = INT16 for reference), calibrated on healthy training windows, and
  exports the .espdl files ESP-DL loads (--out/<quant>/);
- runs the quantized model in esp-ppq's simulator on every healthy and fault
  run, scores it exactly like the float model, and reports the score error,
  and detection and false alarms both with the float thresholds and with
  thresholds re-derived from the quantized validation scores;
- writes test vectors (inputs and simulated outputs) for the on-ESP check in
  scripts/esp/int8_bench.py.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

from esp_ppq import QuantizationSettingFactory
from esp_ppq.api import espdl_quantize_torch
from esp_ppq.executor import TorchExecutor

from scripts.faults.evaluate_faults import primary_model
from scripts.models.gru_experiment import (
    GRUForecaster,
    alarms,
    load_faults,
    load_healthy,
    no_alarm_threshold,
    trailing_mean,
    window_mask,
)
from scripts.models.train_autoencoder import ConvAutoencoder

ROOT = Path(__file__).resolve().parents[2]
MODELS = ("motor", "ir")
K, N, WARMUP_S = 3, 5, 0.5


class GRUStep(torch.nn.Module):
    """One step of the motor GRU with explicit state, as the ESP runs it."""

    def __init__(self, model):
        super().__init__()
        self.cell = torch.nn.GRUCell(model.gru.input_size, model.gru.hidden_size)
        self.cell.weight_ih.data = model.gru.weight_ih_l0.data.clone()
        self.cell.weight_hh.data = model.gru.weight_hh_l0.data.clone()
        self.cell.bias_ih.data = model.gru.bias_ih_l0.data.clone()
        self.cell.bias_hh.data = model.gru.bias_hh_l0.data.clone()
        self.head = model.head

    def forward(self, x, state):
        state = self.cell(x, state)
        return self.head(state), state


def flat_windows(data, window):
    windows = np.lib.stride_tricks.sliding_window_view(data, window, axis=0)
    return np.ascontiguousarray(windows.transpose(0, 2, 1)).reshape(len(windows), -1)


def run_batched(executor, inputs, batch=1024):
    outputs = []
    for start in range(0, len(inputs), batch):
        chunk = torch.from_numpy(inputs[start:start + batch].astype(np.float32))
        outputs.append(executor(chunk)[0].detach().cpu().numpy())
    return np.concatenate(outputs)


def conv_score_fn(reconstruct, name, window):
    cache = {}

    def score(run):
        key = run["file"] + run.get("run_id", "")
        if key not in cache:
            data = run[name]
            values = np.full(len(data), np.nan)
            flat = flat_windows(data, window)
            values[window - 1:] = ((reconstruct(flat) - flat) ** 2).mean(axis=1)
            cache[key] = (values, window_mask(run, window, WARMUP_S))
        return cache[key]

    return score


def gru_score_fn(step, hidden, score_window, runs):
    """Score every run with ``step(x [B,2], state [B,H]) -> (y, state)`` run
    over all runs at once (padded), one call per time step."""
    longest = max(len(run["times"]) for run in runs)
    inputs = np.zeros((longest, len(runs), 2), np.float32)
    for i, run in enumerate(runs):
        inputs[:len(run["times"]), i] = run["motor"][:, :2]
    state = np.zeros((len(runs), hidden), np.float32)
    predicted = np.zeros((longest, len(runs), 2), np.float32)
    for t in range(longest):
        predicted[t], state = step(inputs[t], state)
    cache = {}
    for i, run in enumerate(runs):
        count = len(run["times"])
        errors = (predicted[:count, i] - run["motor"][:, 2:]) ** 2
        score = trailing_mean(errors, score_window).max(axis=1)
        cache[run["file"] + run.get("run_id", "")] = (score,
                                                      window_mask(run, score_window, WARMUP_S))
    return lambda run: cache[run["file"] + run.get("run_id", "")]


def compare(float_fn, quant_fn, float_threshold, healthy, faults, family, margin=1.0):
    """Score error and detection of a quantized model against its float one."""
    differences = []
    for run in healthy["validation"] + healthy["test"] + faults:
        f, mask = float_fn(run)
        q, _ = quant_fn(run)
        keep = mask & np.isfinite(f)
        if keep.any():
            differences.append(np.abs(q[keep] - f[keep]))
    differences = np.concatenate(differences)
    requantized = no_alarm_threshold([quant_fn(r) for r in healthy["validation"]], K, N) * margin
    result = {"score_abs_error": {
        "median": float(np.median(differences)), "p99": float(np.percentile(differences, 99)),
        "max": float(differences.max()), "relative_to_threshold_p99":
            float(np.percentile(differences, 99) / float_threshold)}}
    for label, fn, threshold in (("float", float_fn, float_threshold),
                                 ("quant_float_threshold", quant_fn, float_threshold),
                                 ("quant_own_threshold", quant_fn, requantized)):
        detected, pre_onset, per_fault = 0, 0, {}
        for run in faults:
            if primary_model(run["info"]["fault_type"]) != family:
                continue
            series = alarms(*fn(run), threshold, K, N)
            hit = bool((series & (run["times"] >= run["onset"])).any())
            detected += hit
            pre_onset += bool((series & (run["times"] < run["onset"])).any())
            per_fault[run["info"]["fault_name"]] = per_fault.get(run["info"]["fault_name"], 0) + hit
        result[label] = {
            "threshold": threshold,
            "detected": detected,
            "pre_onset_runs": pre_onset,
            "per_fault": per_fault,
            "healthy_validation_alarm_runs": sum(
                bool(alarms(*fn(r), threshold, K, N).any()) for r in healthy["validation"]),
            "healthy_test_alarm_runs": sum(
                bool(alarms(*fn(r), threshold, K, N).any()) for r in healthy["test"]),
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--conv-model", default="models/esp-conv-v1")
    parser.add_argument("--gru", default="models/esp-gru-v1")
    parser.add_argument("--out", type=Path, default=ROOT / "models/esp-quantized-v1")
    parser.add_argument("--report", type=Path, default=ROOT / "reports/esp-int8-v1")
    parser.add_argument("--quant", nargs="+", default=["w8a8", "w16a16"])
    parser.add_argument("--calibration-windows", type=int, default=1024)
    args = parser.parse_args()
    torch.manual_seed(0)

    conv_dir = ROOT / args.conv_model
    config = json.loads((conv_dir / "config.json").read_text(encoding="utf-8"))
    gru_config = json.loads((ROOT / args.gru / "config.json").read_text(encoding="utf-8"))
    mean = {m: np.asarray(config["models"][m]["scaler_mean"]) for m in MODELS}
    scale = {m: np.asarray(config["models"][m]["scaler_scale"]) for m in MODELS}
    healthy = load_healthy(ROOT / "data/datasets/splits/esp-runs-v1.json", mean, scale)
    faults = load_faults(ROOT / "data/esp_runs/faults/manifest.csv", mean, scale)
    window = int(config["window_size_samples"])
    architecture = config["architecture"]
    rng = np.random.default_rng(0)
    report = {"settings": {"quant": args.quant, "target": "esp32s3",
                           "calibration_windows": args.calibration_windows}, "models": {}}
    vectors = {}

    for name in MODELS:
        info = config["models"][name]
        features = len(info["features"])
        model = ConvAutoencoder(window, features, architecture["hidden"],
                                architecture["bottleneck"])
        model.load_state_dict(torch.load(conv_dir / info["weights"], weights_only=True))
        model.eval()

        def float_reconstruct(flat, model=model):
            with torch.no_grad():
                return model(torch.from_numpy(flat.astype(np.float32))).numpy()

        float_fn = conv_score_fn(float_reconstruct, name, window)
        train = np.concatenate([flat_windows(r[name], window) for r in healthy["train"]])
        calibration = train[rng.choice(len(train), args.calibration_windows, replace=False)]
        test_inputs = flat_windows(healthy["test"][0][name], window)[100:116]
        vectors[name] = {"input": test_inputs.tolist(),
                         "float": float_reconstruct(test_inputs).tolist()}
        report["models"][name] = {}
        for quant in args.quant:
            export = args.out / quant / f"{name}.espdl"
            graph = espdl_quantize_torch(
                model=model, espdl_export_file=str(export),
                calib_dataloader=DataLoader(
                    [torch.from_numpy(row.astype(np.float32)) for row in calibration],
                    batch_size=32),
                calib_steps=args.calibration_windows // 32,
                input_shape=[1, window * features], target="esp32s3", quant_type=quant,
                setting=QuantizationSettingFactory.espdl_setting(), device="cpu",
                error_report=False, export_test_values=False, verbose=0)
            executor = TorchExecutor(graph=graph, device="cpu")
            quant_fn = conv_score_fn(lambda flat, e=executor: run_batched(e, flat), name, window)
            report["models"][name][quant] = compare(
                float_fn, quant_fn, info["threshold"], healthy, faults,
                "ir" if name == "ir" else "motor")
            report["models"][name][quant]["espdl_bytes"] = export.stat().st_size
            vectors[name][quant] = run_batched(executor, test_inputs).tolist()
            print(name, quant, json.dumps({k: v for k, v in report["models"][name][quant].items()
                                           if k != "per_fault"}, default=str)[:600], flush=True)

    # Motor GRU, one step at a time with explicit state.
    deploy = gru_config["deploy"]["motor_gru"]
    hidden = int(gru_config["hidden"])
    gru = GRUForecaster(2, 2, hidden)
    gru.load_state_dict(torch.load(ROOT / args.gru / "gru_motor.pt", weights_only=True))
    gru.eval()
    step_model = GRUStep(gru).eval()
    all_runs = healthy["train"] + healthy["validation"] + healthy["test"] + faults

    def float_step(x, state):
        with torch.no_grad():
            y, s = step_model(torch.from_numpy(x), torch.from_numpy(state))
        return y.numpy(), s.numpy()

    float_fn = gru_score_fn(float_step, hidden, int(deploy["score_window_samples"]), all_runs)
    report["models"]["gru"] = {}
    # Calibration: (input, state) pairs from float runs of training data.
    pairs = []
    for run in healthy["train"][:12]:
        state = np.zeros((1, hidden), np.float32)
        for x in run["motor"][:, :2].astype(np.float32):
            pairs.append((x[None], state))
            _, state = float_step(x[None], state)
    picks = rng.choice(len(pairs), args.calibration_windows, replace=False)
    calibration = [[torch.from_numpy(pairs[i][0][0]), torch.from_numpy(pairs[i][1][0])]
                   for i in picks]
    for quant in args.quant:
        export = args.out / quant / "motor_gru.espdl"
        graph = espdl_quantize_torch(
            model=step_model, espdl_export_file=str(export),
            calib_dataloader=DataLoader(calibration, batch_size=32),
            calib_steps=args.calibration_windows // 32,
            input_shape=[[1, 2], [1, hidden]], target="esp32s3", quant_type=quant,
            setting=QuantizationSettingFactory.espdl_setting(), device="cpu",
            error_report=False, export_test_values=False, verbose=0)
        executor = TorchExecutor(graph=graph, device="cpu")

        def quant_step(x, state, e=executor):
            y, s = e([torch.from_numpy(x), torch.from_numpy(state)])[:2]
            return y.detach().numpy(), s.detach().numpy()

        quant_fn = gru_score_fn(quant_step, hidden, int(deploy["score_window_samples"]), all_runs)
        report["models"]["gru"][quant] = compare(
            float_fn, quant_fn, deploy["threshold"], healthy, faults, "motor",
            margin=float(deploy["margin"]))
        report["models"]["gru"][quant]["espdl_bytes"] = export.stat().st_size
        print("gru", quant, json.dumps({k: v for k, v in report["models"]["gru"][quant].items()
                                        if k != "per_fault"}, default=str)[:600], flush=True)

    args.report.mkdir(parents=True, exist_ok=True)
    (args.report / "results.json").write_text(json.dumps(report, indent=2, default=str) + "\n",
                                              encoding="utf-8")
    (args.out / "test_vectors.json").write_text(json.dumps(vectors) + "\n", encoding="utf-8")
    print(f"Saved {args.report / 'results.json'} and {args.out}")


if __name__ == "__main__":
    main()
