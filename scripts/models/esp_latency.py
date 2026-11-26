"""Measure detector architectures' latency and weight memory on the ESP.

Sends the firmware's 'B' benchmark command (firmware/main/bench.cpp) for every
architecture in the search space of scripts/models/search_architectures.py:
the same esp-dsp forward passes as the deployed detectors, with runtime layer
sizes and random weights, so no reflashing is needed. Each line is the median
and maximum of --repeats runs, in microseconds, and the padded float32 weight
size in bytes.

    python -m scripts.models.esp_latency --port COM3 --out reports/esp-latency-v1
"""

import argparse
import csv
import itertools
import json
from pathlib import Path

import serial

ROOT = Path(__file__).resolve().parents[2]

# Search space shared with search_architectures.py.
CONV_SPACE = {
    "features": (3, 4),
    "window": (10, 16, 20, 24, 32),
    "channels": (8, 16, 24, 32),
    "kernel": (3, 5),
    "bottleneck": (4, 8, 12, 16),
    "hidden": (16, 32, 64),
}
GRU_SPACE = {"inputs": (2,), "hidden": (8, 12, 16, 24, 32, 48, 64), "outputs": (2,)}


def conv_key(features, window, channels, kernel, bottleneck, hidden):
    return f"conv-F{features}-W{window}-C{channels}-K{kernel}-Z{bottleneck}-H{hidden}"


def gru_key(inputs, hidden, outputs):
    return f"gru-I{inputs}-H{hidden}-O{outputs}"


def load_table(path):
    """{key: {"median_us", "max_us", "weight_bytes"}} from a latency CSV."""
    with Path(path).open(newline="", encoding="utf-8") as file:
        return {row["key"]: {k: int(row[k]) for k in ("median_us", "max_us", "weight_bytes")}
                for row in csv.DictReader(file)}


def measure(link, command):
    link.write((command + "\n").encode("ascii"))
    while True:
        line = link.readline().decode("ascii", errors="replace").strip()
        if not line:
            raise TimeoutError(f"No reply to {command!r}")
        if line.startswith("b,"):
            ok, median, longest, weight = (int(v) for v in line.split(",")[1:])
            if not ok:
                raise ValueError(f"ESP rejected {command!r}")
            return median, longest, weight


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--repeats", type=int, default=31)
    parser.add_argument("--out", type=Path, default=ROOT / "reports/esp-latency-v1")
    args = parser.parse_args()

    link = serial.Serial(port=None, baudrate=115200, timeout=5.0, dsrdtr=False, rtscts=False)
    link.port = args.port
    link.dtr = False
    link.rts = False
    link.open()
    link.reset_input_buffer()
    rows = []
    try:
        for values in itertools.product(*GRU_SPACE.values()):
            spec = dict(zip(GRU_SPACE, values))
            median, longest, weight = measure(
                link, f"B,G,{spec['inputs']},{spec['hidden']},{spec['outputs']},{args.repeats}")
            rows.append({"key": gru_key(**spec), "model": "gru", **spec,
                         "median_us": median, "max_us": longest, "weight_bytes": weight})
        for values in itertools.product(*CONV_SPACE.values()):
            spec = dict(zip(CONV_SPACE, values))
            median, longest, weight = measure(
                link, "B,C," + ",".join(str(v) for v in values) + f",{args.repeats}")
            rows.append({"key": conv_key(**spec), "model": "conv", **spec,
                         "median_us": median, "max_us": longest, "weight_bytes": weight})
            if len(rows) % 100 == 0:
                print(f"{len(rows)} architectures measured", flush=True)
    finally:
        link.close()

    args.out.mkdir(parents=True, exist_ok=True)
    fields = ["key", "model", *dict.fromkeys(list(CONV_SPACE) + list(GRU_SPACE)),
              "median_us", "max_us", "weight_bytes"]
    with (args.out / "latency.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields, restval="")
        writer.writeheader()
        writer.writerows(rows)
    (args.out / "settings.json").write_text(json.dumps(
        {"repeats": args.repeats, "conv_space": CONV_SPACE, "gru_space": GRU_SPACE,
         "firmware": "esp-dsp float32, weights in internal RAM, -O2, 160 MHz"},
        indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {len(rows)} rows to {args.out / 'latency.csv'}")


if __name__ == "__main__":
    main()
