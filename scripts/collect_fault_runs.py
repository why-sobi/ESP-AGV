"""Record labelled fault-injection runs headlessly in Webots.

Reads a fault matrix (routes/speeds x faults, shared onset time), runs each
combination with the mission's ``fault`` block set, and appends one manifest
row per run. Fault runs are expected to end early sometimes (line-loss or path
safety stop); that is recorded as the outcome, not treated as an error.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

from collect_route_runs import DEFAULT_MISSION, DEFAULT_WORLD, ROOT, ROUTES, find_webots, run_mission

DEFAULT_MATRIX = ROOT / "data" / "fault_runs" / "fault-matrix-v1.json"
MANIFEST_FIELDS = (
    "file", "route", "start_node", "destination_node", "wheel_speed_rad_s",
    "fault_name", "fault_type", "target", "severity", "onset_s",
    "outcome", "duration_s", "faulty_samples",
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
    args = parser.parse_args()

    matrix = json.loads(args.matrix.read_text(encoding="utf-8-sig"))
    faults = [f for f in matrix["faults"] if not args.faults or f["name"] in args.faults]
    if args.faults and len(faults) != len(set(args.faults)):
        parser.error("unknown fault name in --faults")
    webots = find_webots(args.webots)
    base_mission = json.loads(args.mission.read_text(encoding="utf-8-sig"))
    out_dir = (args.out or args.matrix.parent).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest = out_dir / "manifest.csv"
    write_header = not manifest.exists()

    missing = 0
    with manifest.open("a", newline="", encoding="utf-8") as manifest_file:
        writer = csv.DictWriter(manifest_file, fieldnames=MANIFEST_FIELDS)
        if write_header:
            writer.writeheader()
        for fault_index, fault in enumerate(faults):
            for run_index, run in enumerate(matrix["runs"]):
                start, destination = ROUTES[run["route"]]
                spec = {k: v for k, v in fault.items() if k != "name"}
                spec.setdefault("onset_s", matrix["onset_s"])
                spec.setdefault("seed", fault_index * 100 + run_index)
                mission = dict(
                    base_mission,
                    start_node=start,
                    destination_node=destination,
                    wheel_speed_rad_s=run["speed"],
                    run_label=f"{run['route']}-{run['speed']:g}-{fault['name']}",
                    fault=spec,
                )
                path, completed, output, elapsed = run_mission(
                    webots, args.world, mission, out_dir, args.timeout
                )
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
                })
                manifest_file.flush()
                print(f"{fault['name']:26s} {run['route']}@{run['speed']:g}: {outcome:13s} "
                      f"{duration:5.1f} s, {faulty} faulty samples ({elapsed:.0f} s wall)",
                      flush=True)
    print(f"Done: {missing} run(s) without a CSV.")
    sys.exit(1 if missing else 0)


if __name__ == "__main__":
    main()
