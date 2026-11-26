"""Measure the quantized detectors on the ESP with ESP-DL, next to float32.

Needs the ESP-DL benchmark app flashed (firmware/esp-dl-bench, which replaces
the AGV firmware until that is flashed again) and the models and test vectors
from scripts/models/quantize_int8.py. For each model and quantization it
reports the ESP-DL time per inference (quantizing the input, Model::run,
dequantizing the output; median of --repeats), its parameter and total heap
memory, and how far the ESP's outputs are from esp-ppq's simulation and from
float32 on the test vectors. The float32 esp-dsp times come from the deployed
firmware's benchmark table (reports/esp-latency-v1).

    python -m scripts.esp.int8_bench --port COM3
"""

import argparse
import json
from pathlib import Path

import numpy as np
import serial

from scripts.models.esp_latency import conv_key, gru_key, load_table

ROOT = Path(__file__).resolve().parents[2]
QUANTS = ("w8a8", "w16a16")
FLOAT_KEYS = {"motor": conv_key(4, 20, 16, 3, 8, 32), "ir": conv_key(3, 20, 16, 3, 8, 32),
              "motor_gru": gru_key(2, 32, 2)}


def ask(link, command, prefix):
    link.write((command + "\n").encode("ascii"))
    while True:
        line = link.readline().decode("ascii", errors="replace").strip()
        if not line:
            raise TimeoutError(f"No reply to {command[:40]!r}")
        if line.startswith(prefix):
            fields = line.split(",")
            if fields[1] != "1":
                raise ValueError(f"ESP failed {command[:40]!r}: {line}")
            return fields[2:]


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--repeats", type=int, default=101)
    parser.add_argument("--vectors", type=Path,
                        default=ROOT / "models/esp-quantized-v1/test_vectors.json")
    parser.add_argument("--latency", type=Path, default=ROOT / "reports/esp-latency-v1/latency.csv")
    parser.add_argument("--out", type=Path, default=ROOT / "reports/esp-int8-v1/esp_bench.json")
    args = parser.parse_args()
    vectors = json.loads(args.vectors.read_text(encoding="utf-8"))
    float_table = load_table(args.latency)

    link = serial.Serial(port=None, baudrate=115200, timeout=10.0, dsrdtr=False, rtscts=False)
    link.port = args.port
    link.dtr = False
    link.rts = False
    link.open()
    link.reset_input_buffer()
    rows = []
    try:
        for model in ("motor", "ir", "motor_gru"):
            reference = float_table[FLOAT_KEYS[model]]
            rows.append({"model": model, "quant": "float32 (esp-dsp)",
                         "median_us": reference["median_us"], "max_us": reference["max_us"],
                         "param_bytes": reference["weight_bytes"]})
            for quant in QUANTS:
                median, longest, params, heap = (
                    int(v) for v in ask(link, f"T,{model}_{quant},{args.repeats}", "t,"))
                row = {"model": model, "quant": quant, "median_us": median, "max_us": longest,
                       "param_bytes": params, "heap_bytes": heap}
                if model in vectors and quant in vectors[model]:
                    esp, simulated, exact = [], [], []
                    for i, values in enumerate(vectors[model]["input"]):
                        reply = ask(link, f"R,{model}_{quant}," + ",".join(f"{v:.6g}" for v in values),
                                    "r,")
                        esp.append([float(v) for v in reply])
                        simulated.append(vectors[model][quant][i])
                        exact.append(vectors[model]["float"][i])
                    esp, simulated, exact = (np.asarray(a) for a in (esp, simulated, exact))
                    row["max_abs_vs_simulation"] = float(np.abs(esp - simulated).max())
                    row["max_abs_vs_float"] = float(np.abs(esp - exact).max())
                rows.append(row)
                print(row, flush=True)
    finally:
        link.close()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
    print(f"Saved {args.out}")


if __name__ == "__main__":
    main()
