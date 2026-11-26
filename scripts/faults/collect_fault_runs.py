"""Record labelled fault-injection runs headlessly in Webots.

Reads a fault matrix (routes/speeds x faults, shared onset time), runs each
combination with the mission's ``fault`` block set, and appends one manifest
row per run.

Optional matrix keys: ``mission`` (overrides for every run, e.g.
``control_mode: esp32`` and ``return_to_start``), ``spawn_jitter`` (start-pose
jitter, seeded by each run's ``seed``) and, per run, ``onset_s`` and ``seed``.
In esp32 mode a run's speed is sent to the ESP as its route speed.

Fault runs are expected to end early sometimes (line-loss or path
safety stop); that is recorded as the outcome, not treated as an error.
"""

import argparse
import csv
import json
import sys
import time
from pathlib import Path

from scripts.telemetry.collect_route_runs import (
    DEFAULT_MISSION,
    DEFAULT_WORLD,
    ROOT,
    ROUTES,
    error_log_size,
    find_webots,
    run_mission,
    serial_error_since,
)

DEFAULT_MATRIX = ROOT / "data" / "fault_runs" / "fault-matrix-v1.json"
MANIFEST_FIELDS = (
    "file", "route", "start_node", "destination_node", "wheel_speed_rad_s",
    "fault_name", "fault_type", "target", "severity", "onset_s",
    "outcome", "duration_s", "faulty_samples", "control_mode", "seed",
)


def summarize_fault_run(path):
    with path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        return 0.0, 0
    duration = float(rows[-1]["time"]) - float(rows[0]["time"])
    faulty = sum(1 for row in rows if row.get("fault_active") == "1")
    return duration, faulty


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix", type=Path, default=DEFAULT_MATRIX)
    parser.add_argument("--faults", nargs="+", help="Only these fault names")
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--mission", type=Path, default=DEFAULT_MISSION, help="Base mission")
    parser.add_argument("--out", type=Path, help="Defaults to the matrix file's directory")
    parser.add_argument("--webots", help="Path to the Webots executable")
    parser.add_argument("--timeout", type=float, default=180.0, help="Wall seconds per run")
    parser.add_argument("--serial-retries", type=int, default=3,
                        help="Repeat a run this many times if the ESP serial link drops")
    parser.add_argument("--resume", action="store_true",
                        help="Skip fault/run combinations already in the manifest")
    args = parser.parse_args()

    matrix = json.loads(args.matrix.read_text(encoding="utf-8-sig"))
    faults = [f for f in matrix["faults"] if not args.faults or f["name"] in args.faults]
    if args.faults and len(faults) != len(set(args.faults)):
        parser.error("unknown fault name in --faults")
    webots = find_webots(args.webots)
    base_mission = json.loads(args.mission.read_text(encoding="utf-8-sig"))
    # Fault runs only report alarms unless the matrix's mission block sets
    # esp_fault_stop_s: a fault stop ends the run at the first long alarm.
    base_mission.pop("esp_fault_stop_s", None)
    base_mission.update(matrix.get("mission", {}))
    out_dir = (args.out or args.matrix.parent).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest = out_dir / "manifest.csv"
    fields = MANIFEST_FIELDS
    if manifest.exists():
        # Older manifests lack the later columns; keep appending in their layout.
        with manifest.open(newline="", encoding="utf-8") as existing:
            fields = tuple(next(csv.reader(existing), MANIFEST_FIELDS))
    write_header = not manifest.exists()
    done = set()
    if args.resume and manifest.exists():
        with manifest.open(newline="", encoding="utf-8") as existing:
            done = {
                (row["fault_name"], row["route"], float(row["wheel_speed_rad_s"]))
                for row in csv.DictReader(existing) if row["outcome"] != "serial_error"
            }

    missing = 0
    with manifest.open("a", newline="", encoding="utf-8") as manifest_file:
        writer = csv.DictWriter(manifest_file, fieldnames=fields, extrasaction="ignore")
        if write_header:
            writer.writeheader()
        for fault_index, fault in enumerate(faults):
            for run_index, run in enumerate(matrix["runs"]):
                start, destination = ROUTES[run["route"]]
                spec = {k: v for k, v in fault.items() if k != "name"}
                spec.setdefault("onset_s", run.get("onset_s", matrix["onset_s"]))
                spec.setdefault("seed", fault_index * 100 + run_index)
                mission = dict(
                    base_mission,
                    start_node=start,
                    destination_node=destination,
                    wheel_speed_rad_s=run["speed"],
                    run_label=f"{run['route']}-{run['speed']:g}-{fault['name']}",
                    fault=spec,
                )
                if mission.get("control_mode") == "esp32":
                    mission["esp_speed_rad_s"] = run["speed"]
                if "seed" in run:
                    mission["spawn_jitter"] = dict(matrix.get("spawn_jitter", {}),
                                                   seed=run["seed"])
                if (fault["name"], run["route"], float(run["speed"])) in done:
                    continue
                for attempt in range(args.serial_retries + 1):
                    log_offset = error_log_size(out_dir)
                    path, completed, output, elapsed = run_mission(
                        webots, args.world, mission, out_dir, args.timeout
                    )
                    if not serial_error_since(out_dir, log_offset):
                        break
                    print(f"{fault['name']} {run['route']}@{run['speed']:g}: serial link "
                          f"dropped; retrying",
                          flush=True)
                    if path is not None:
                        writer.writerow({"file": path.name, "route": run["route"],
                                         "fault_name": fault["name"],
                                         "wheel_speed_rad_s": run["speed"],
                                         "outcome": "serial_error"})
                        manifest_file.flush()
                    time.sleep(5.0)  # Let the USB serial device settle.
                if path is None:
                    missing += 1
                    print(f"{fault['name']} {run['route']}@{run['speed']:g}: NO CSV | "
                          + " | ".join(output.strip().splitlines()[-3:]), flush=True)
                    continue
                duration, faulty = summarize_fault_run(path)
                outcome = "completed" if completed else "stopped_early"
                writer.writerow({
                    "file": path.name, "route": run["route"], "start_node": start,
                    "destination_node": destination, "wheel_speed_rad_s": run["speed"],
                    "fault_name": fault["name"], "fault_type": spec["type"],
                    "target": spec.get("target", ""), "severity": spec.get("severity", ""),
                    "onset_s": spec["onset_s"], "outcome": outcome,
                    "duration_s": round(duration, 3), "faulty_samples": faulty,
                    "control_mode": mission.get("control_mode", "supervisor"),
                    "seed": run.get("seed", ""),
                })
                manifest_file.flush()
                print(f"{fault['name']:26s} {run['route']}@{run['speed']:g}: {outcome:13s} "
                      f"{duration:5.1f} s, {faulty} faulty samples ({elapsed:.0f} s wall)",
                      flush=True)
    print(f"Done: {missing} run(s) without a CSV.")
    sys.exit(1 if missing else 0)


if __name__ == "__main__":
    main()
