"""Export the ESP's anomaly detectors to a C++ header.

Writes the conv autoencoder pair (motor + IR), the motor GRU plant model and
the stuck-IR rule: weights, the training scaler, the thresholds and the
monitoring rules (window, warm-up, persistence, U-turn settling).
firmware/main/anomaly.cpp runs them with esp-dsp, whose ESP32-S3 SIMD
routines need 16-byte-aligned data and lengths that are multiples of 4, so the
weights are written pre-arranged for those calls:

- Conv1d layers become one matrix multiply each (im2col): the weight is stored
  as [k * in_channels + i][out_channel], rows padded to a multiple of 4.
- The motor and IR models may differ in window, channels, bottleneck and
  hidden size (models/esp-conv-v2); each model's sizes go into its struct and
  the firmware's buffers are sized for the largest.
- The encoder Linear takes the conv output time-major ([t][channel]) instead
  of torch's channel-major Flatten, so its columns are permuted to match.
- Linear and GRU weights are stored row-major with each row padded to a
  multiple of 4 floats (zeros), for one dot product per output.

    python -m scripts.models.export_c --model models/esp-conv-v2 \
        --gru models/esp-gru-v2 --out firmware/main/anomaly_model.h
"""

import argparse
import json
from pathlib import Path

import numpy as np
import torch

from scripts.models.train_autoencoder import MODELS, model_architecture, model_window

CONV_LAYERS = ("conv1_wt", "conv1_b", "conv2_wt", "conv2_b", "enc_w", "enc_b",
               "dec1_w", "dec1_b", "dec2_w", "dec2_b")
GRU_LAYERS = ("w_ih", "w_hh", "b_ih", "b_hh", "head_w", "head_b")
MAX_FEATURES = 4


def round4(value):
    return (value + 3) // 4 * 4


def pad_rows(matrix, width=None):
    """Pad each row of a 2-D array with zeros to `width` (default: a multiple of 4)."""
    matrix = np.asarray(matrix, dtype=np.float32)
    width = width or round4(matrix.shape[1])
    out = np.zeros((matrix.shape[0], width), np.float32)
    out[:, :matrix.shape[1]] = matrix
    return out


def conv_arrays(state, features, window, channels):
    """Conv autoencoder weights arranged for anomaly.cpp."""
    reduced = (window + 1) // 2
    w1 = state["encoder.0.weight"].numpy()  # [out, in, k]
    w2 = state["encoder.2.weight"].numpy()
    conv1 = w1.transpose(2, 1, 0).reshape(3 * features, channels)  # [k*F+i][c]
    conv1_padded = np.zeros((round4(3 * MAX_FEATURES), channels), np.float32)
    conv1_padded[: 3 * features] = conv1
    conv2 = w2.transpose(2, 1, 0).reshape(3 * channels, channels)  # [k*C+i][c]
    enc = state["encoder.5.weight"].numpy()  # [bottleneck, c*reduced+t]
    bottleneck = enc.shape[0]
    enc_time_major = enc.reshape(bottleneck, channels, reduced).transpose(0, 2, 1)
    return {
        "conv1_wt": conv1_padded,
        "conv1_b": state["encoder.0.bias"].numpy(),
        "conv2_wt": conv2,
        "conv2_b": state["encoder.2.bias"].numpy(),
        "enc_w": pad_rows(enc_time_major.reshape(bottleneck, reduced * channels)),
        "enc_b": state["encoder.5.bias"].numpy(),
        "dec1_w": pad_rows(state["decoder.0.weight"].numpy()),
        "dec1_b": state["decoder.0.bias"].numpy(),
        "dec2_w": pad_rows(state["decoder.2.weight"].numpy()),
        "dec2_b": state["decoder.2.bias"].numpy(),
    }


def gru_arrays(state):
    return {
        "w_ih": pad_rows(state["gru.weight_ih_l0"].numpy()),
        "w_hh": pad_rows(state["gru.weight_hh_l0"].numpy()),
        "b_ih": state["gru.bias_ih_l0"].numpy(),
        "b_hh": state["gru.bias_hh_l0"].numpy(),
        "head_w": pad_rows(state["head.weight"].numpy()),
        "head_b": state["head.bias"].numpy(),
    }


def c_float(value):
    """A C++ float literal: '50' and '0' need a decimal point before the F."""
    text = f"{float(value):.9g}"
    if not any(c in text for c in ".e"):
        text += ".0"
    return text + "F"


def c_array(name, values):
    values = [float(v) for v in np.asarray(values).flatten()]
    lines = []
    for start in range(0, len(values), 6):
        lines.append("    " + ", ".join(c_float(v) for v in values[start:start + 6]) + ",")
    return (f"alignas(16) inline constexpr float {name}[{len(values)}] = {{\n"
            + "\n".join(lines) + "\n};\n")


def persistence(rule):
    k, n = (int(x) for x in rule["persistence"].split(">= ")[1].split(" windows")[0]
            .split(" of the last "))
    return k, n


def export(model_dir, gru_dir, out_path):
    model_dir, gru_dir = Path(model_dir), Path(gru_dir)
    config = json.loads((model_dir / "config.json").read_text(encoding="utf-8"))
    gru_config = json.loads((gru_dir / "config.json").read_text(encoding="utf-8"))
    deploy = gru_config["deploy"]
    architecture = config["architecture"]
    if architecture["type"] != "conv_autoencoder":
        raise ValueError("Only the conv autoencoder is supported by firmware/main/anomaly.cpp")
    rule = config["threshold_rule"]
    k, n = persistence(rule)
    if (deploy["persistence"]["k"], deploy["persistence"]["n"]) != (k, n):
        raise ValueError("The conv and GRU configs use different persistence rules.")
    if deploy["warmup_s"] != rule["warmup_s"]:
        raise ValueError("The conv and GRU configs use different warm-ups.")
    windows = {name: model_window(config, name) for name in MODELS}
    sizes = {name: model_architecture(config, name) for name in MODELS}
    for name, arch in sizes.items():
        if arch["kernel"] != 3:
            raise ValueError(f"{name}: firmware/main/anomaly.cpp supports kernel 3 only")
        for key in ("channels", "bottleneck", "hidden"):
            if arch[key] % 4:
                raise ValueError(f"{name}: {key} must be a multiple of 4 for esp-dsp")
    max_window = max(windows.values())
    max_reduced = (max_window + 1) // 2
    unmonitored = rule.get("unmonitored_states", [])
    if set(unmonitored) - {"AT_DEST", "UNLOADING", "U_TURN"}:
        raise ValueError(f"Unsupported unmonitored states: {unmonitored}")
    if gru_config["scaler"]["motor"]["mean"] != config["models"]["motor"]["scaler_mean"]:
        raise ValueError("The GRU must use the conv motor model's scaler.")
    hidden = int(gru_config["hidden"])
    stuck = deploy["ir_stuck"]
    motor_gru = deploy["motor_gru"]

    parts = [
        f"// Generated by scripts/models/export_c.py from {model_dir.as_posix()} and "
        f"{gru_dir.as_posix()}. Do not edit.\n",
        "#pragma once\n\n#include <cstddef>\n\nnamespace anomaly_model {\n\n",
        "// Buffer sizes: the largest of the motor and IR conv models (each model's\n"
        "// own sizes are in its ConvAutoencoder).\n",
        f"constexpr int kMaxWindow = {max_window};  // Samples per window (32 ms each).\n",
        f"constexpr int kMaxWindowPadded = {round4(max_window)};  "
        "// Rows of the conv1 matrix multiply.\n",
        f"constexpr int kMaxReduced = {max_reduced};  // Time steps after the stride-2 conv.\n",
        f"constexpr int kMaxReducedPadded = {round4(max_reduced)};  "
        "// Rows of the conv2 matrix multiply.\n",
        f"constexpr int kMaxChannels = {max(a['channels'] for a in sizes.values())};\n",
        f"constexpr int kMaxBottleneck = {max(a['bottleneck'] for a in sizes.values())};\n",
        f"constexpr int kMaxHidden = {max(a['hidden'] for a in sizes.values())};\n",
        f"constexpr int kMaxFeatures = {MAX_FEATURES};\n",
        f"constexpr int kConv1Patch = {round4(3 * MAX_FEATURES)};  "
        "// im2col row: 3 taps x features, padded.\n",
        f"constexpr int kPersistK = {k};  // Alarm when >= K of the last N windows exceed.\n",
        f"constexpr int kPersistN = {n};\n",
        f"constexpr int kWarmupMs = {round(float(rule['warmup_s']) * 1000)};  "
        "// Windows starting earlier are not monitored.\n",
        "constexpr int kSettleMs = 500;  // Samples this soon after a U-turn are not monitored.\n\n",
        "// Motor GRU plant model: wheel targets -> encoder wheel speeds.\n",
        f"constexpr int kGruHidden = {hidden};\n",
        "constexpr int kGruInputs = 4;   // 2 wheel targets, padded.\n",
        "constexpr int kGruOutputs = 2;  // Wheel speeds.\n",
        f"constexpr int kGruWindow = {motor_gru['score_window_samples']};  "
        "// Errors averaged into one score.\n",
        f"constexpr float kGruThreshold = {c_float(motor_gru['threshold'])};  "
        f"// {motor_gru['margin']:g} x the no-validation-alarm threshold.\n\n",
        "// Stuck IR sensor: flat for kStuckSamples differences while the robot drives.\n",
        f"constexpr int kStuckSamples = {stuck['samples']};\n",
        f"constexpr float kStuckEps = {c_float(stuck['eps'])};     // Raw sensor units.\n",
        f"constexpr float kStuckSpeed = {c_float(stuck['speed'])};  "
        "// Mean wheel speed (|left| + |right|) / 2, rad/s.\n\n",
        "struct ConvAutoencoder {\n  int features;\n  int window;\n  int channels;\n"
        "  int bottleneck;\n  int hidden;\n  float threshold;\n  const float *mean;\n"
        "  const float *scale;\n"
        + "".join(f"  const float *{name};\n" for name in CONV_LAYERS)
        + "};\n\n",
    ]
    for name in MODELS:
        info = config["models"][name]
        state = torch.load(model_dir / info["weights"], weights_only=True)
        prefix = "k" + name.capitalize()
        features = len(info["features"])
        arch, window = sizes[name], windows[name]
        parts.append(f"// {name}: features {', '.join(info['features'])}; window {window}, "
                     f"{arch['channels']} channels, bottleneck {arch['bottleneck']}, "
                     f"hidden {arch['hidden']}\n")
        parts.append(c_array(f"{prefix}Mean", info["scaler_mean"]))
        parts.append(c_array(f"{prefix}Scale", info["scaler_scale"]))
        for c_name, values in conv_arrays(state, features, window, arch["channels"]).items():
            parts.append(c_array(f"{prefix}_{c_name}", values))
        members = ", ".join(f"{prefix}_{c_name}" for c_name in CONV_LAYERS)
        parts.append(
            f"inline constexpr ConvAutoencoder {prefix}{{{features}, {window}, "
            f"{arch['channels']}, {arch['bottleneck']}, {arch['hidden']}, "
            f"{c_float(info['threshold'])}, {prefix}Mean, {prefix}Scale, {members}}};\n\n"
        )
    state = torch.load(gru_dir / "gru_motor.pt", weights_only=True)
    parts.append("// Motor GRU (torch gate order r, z, n); scaled with kMotorMean/kMotorScale.\n")
    for c_name, values in gru_arrays(state).items():
        parts.append(c_array(f"kGru_{c_name}", values))
    parts.append("\n}  // namespace anomaly_model\n")
    Path(out_path).write_text("".join(parts), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", default="models/esp-conv-v2")
    parser.add_argument("--gru", default="models/esp-gru-v2")
    parser.add_argument("--out", default="firmware/main/anomaly_model.h")
    args = parser.parse_args()
    export(args.model, args.gru, args.out)
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
