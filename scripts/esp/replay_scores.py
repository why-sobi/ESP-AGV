"""Replay recorded ESP runs through the ESP and compare its anomaly scores with Python.

Each CSV recorded in ``esp32`` mode holds every sensor frame the ESP received.
This script sends the same route commands and frames again (skipping the
host-side UNLOADING rows, which the ESP never saw), then checks that:

- the ESP reproduces the logged wheel targets (same firmware control logic);
- its motor/IR window scores match the Python reference (scripts/models and
  scripts/faults/evaluate_faults.py) for the same windows;
- its motor GRU scores match scripts/models/gru_experiment.py;
- its alarms match the Python 3-of-5 rule: the GRU alarm, the stuck-sensor
  alarm, and the motor (conv or GRU) and IR (conv or stuck) alarms.

The ESP restarts its windows (and the GRU state) on the return route, so conv
windows that would span the unload/U-turn are compared only where the ESP
reports a score. The GRU and stuck-rule reference restarts per leg like the
ESP.

    python -m scripts.esp.replay_scores --port COM3 data/esp_runs/healthy/route-...csv
"""

import argparse
import csv
import json
import re
import sys
from pathlib import Path

import numpy as np
import serial
import torch

from scripts.esp.route_actions import format_route_command, route_action_plan
from scripts.faults.evaluate_faults import load_detectors, run_windows
from scripts.maps.route_planner import TrackMap, shortest_route
from scripts.models.gru_experiment import (
    GRUForecaster,
    alarms as gated_alarms,
    gru_errors,
    load_run,
    stuck_score,
    trailing_mean,
    window_mask,
)
from scripts.models.train_autoencoder import MODELS, flatten, persistence_alarms, window_scores

ROOT = Path(__file__).resolve().parents[2]
MAP_PATH = ROOT / "maps" / "paths" / "base track.json"
MISSION_PATH = ROOT / "maps" / "routes" / "mission.json"
TARGET_TOLERANCE = 0.0015  # rad/s; targets are sent with 3 decimals.
# Scores are sent with 5 decimals; thresholds are about 0.2-0.5.
SCORE_TOLERANCE = 1e-4
# GRU scores are sent with 6 decimals; its threshold is about 0.025-0.03. Far
# above it (weak motors score up to ~1) float32 rounding grows with the score,
# so the tolerance is relative there.
GRU_SCORE_TOLERANCE = 2e-5
GRU_RELATIVE_TOLERANCE = 1e-4


def route_command(track_map, mission, start, destination, speed, u_turn):
    route = shortest_route(TrackMap.from_dict(track_map), start, destination)
    plan = route_action_plan(track_map, route,
                             distance_scale=float(mission.get("esp_odometry_scale", 0.92)),
                             edge_scales=mission.get("esp_odometry_scale_by_edge"),
                             sharp_turn_degrees=mission.get("esp_sharp_turn_deg"))
    if u_turn:
        plan = [("U", 0.0)] + plan
    plan = [("V", speed)] + plan
    return "R," + format_route_command(plan)


def read_line(link):
    line = link.readline()
    if not line:
        raise TimeoutError("ESP did not answer")
    return line.decode("ascii", errors="replace").strip()


def replay(link, path, track_map, mission):
    """Return per-row ESP fields (None for skipped rows) and the mismatched targets."""
    speed = float(re.search(r"-[ABCD]-([\d.]+)-", path.name).group(1))
    with path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    results = [None] * len(rows)
    target_mismatches = 0
    leg = None
    sequence = 0
    for index, row in enumerate(rows):
        if row["esp_state"] == "UNLOADING":
            continue
        row_leg = (row["route_start"], row["route_destination"])
        if row_leg != leg:
            command = route_command(track_map, mission, *row_leg, speed, u_turn=leg is not None)
            link.write((command + "\n").encode("ascii"))
            answer = read_line(link)
            # The ESP reports LINK_LOST unprompted when the previous run's
            # frames stopped (e.g. while Python scored it); skip that.
            while answer.startswith("M,") and ",LINK_LOST," in answer:
                answer = read_line(link)
            if answer != "A,ROUTE_OK":
                raise RuntimeError(f"{path.name}: route {command!r} rejected: {answer!r}")
            leg = row_leg
        sequence += 1
        # epuck_route always sends the Webots basic time step (32 ms).
        link.write(
            (f"S,{sequence},32,{float(row['gs_left']):.3f},{float(row['gs_center']):.3f},"
             f"{float(row['gs_right']):.3f},{float(row['actual_pos_left']):.6f},"
             f"{float(row['actual_pos_right']):.6f}\n").encode("ascii")
        )
        fields = read_line(link).split(",")
        if fields[0] != "M" or int(fields[1]) != sequence or len(fields) != 14:
            raise RuntimeError(f"{path.name} row {index}: unexpected reply {fields}")
        # One print step (0.001) of slack: firmware built with another
        # optimisation level may round a target the other way.
        if (abs(float(fields[2]) - float(row["target_vel_left"])) > TARGET_TOLERANCE
                or abs(float(fields[3]) - float(row["target_vel_right"])) > TARGET_TOLERANCE
                or fields[4] != row["esp_state"]):
            target_mismatches += 1
        results[index] = {
            "motor": float(fields[7]), "ir": float(fields[8]),
            "motor_alarm": int(fields[9]), "ir_alarm": int(fields[10]),
            "gru": float(fields[11]), "gru_alarm": int(fields[12]),
            "stuck_alarm": int(fields[13]),
        }
    return rows, results, target_mismatches


def python_reference(path, detectors, window_sizes, k, n, warmup_s):
    """Python scores/alarms per window of each model; window j ends at CSV row
    j + window_sizes[model]."""
    reference = {}
    for name in MODELS:
        windows, starts, _, _, monitored = run_windows(path, window_sizes[name])
        reconstruct, threshold, mean, scale = detectors[name]
        scores = window_scores(reconstruct, flatten(((windows[name] - mean) / scale)
                                                    .astype(np.float32)))
        exceed = (scores > threshold) & (starts >= warmup_s) & monitored
        reference[name] = (scores, persistence_alarms(exceed, k, n), threshold)
    return reference


def gru_stuck_reference(path, gru, deploy, mean, scale, k, n):
    """Per CSV row: motor GRU score, GRU alarm and stuck alarm, computed per leg
    like the ESP (state reset on each route command, UNLOADING rows skipped,
    time counted from the leg's first frame). NaN/False where not defined."""
    with path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    count = len(rows)
    score = np.full(count, np.nan)
    gru_alarm = np.zeros(count, bool)
    stuck_alarm = np.zeros(count, bool)
    legs = []
    for index, row in enumerate(rows):
        if row["esp_state"] == "UNLOADING":
            continue
        leg = (row["route_start"], row["route_destination"])
        if not legs or legs[-1][0] != leg:
            legs.append((leg, []))
        legs[-1][1].append(index)
    warmup = float(deploy["warmup_s"])
    motor_gru = deploy["motor_gru"]
    stuck = deploy["ir_stuck"]
    for _, indices in legs:
        leg_rows = [rows[i] for i in indices]
        run = load_run(path, leg_rows, path.stem, mean, scale)
        # The ESP counts time from the leg's first frame (32 ms at that frame).
        run["times"] = run["times"] - float(leg_rows[0]["time"]) + 0.032
        samples = np.asarray(indices[1:])
        window = int(motor_gru["score_window_samples"])
        leg_score = trailing_mean(gru_errors(gru, run, "motor"), window).max(axis=1)
        score[samples] = leg_score
        gru_alarm[samples] = gated_alarms(leg_score, window_mask(run, window, warmup),
                                          motor_gru["threshold"], k, n)
        span = int(stuck["samples"]) + 1
        flags = stuck_score(run, int(stuck["samples"]), stuck["eps"], stuck["speed"])
        stuck_alarm[samples] = gated_alarms(flags, window_mask(run, span, warmup), 0.5, k, n)
    return score, gru_alarm, stuck_alarm


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+", type=Path, help="ESP-mode run CSVs")
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--model", default="models/esp-conv-v2")
    parser.add_argument("--gru", default="models/esp-gru-v2")
    parser.add_argument("--out", type=Path, help="Optional JSON summary")
    parser.add_argument("--per-edge-odometry", action="store_true",
                        help="Use the mission's esp_odometry_scale_by_edge and corner (l/r) "
                             "turns, for runs recorded after 2026-11-27 15:00; older runs "
                             "used the global scale and plain turns")
    args = parser.parse_args()

    config = json.loads((ROOT / args.model / "config.json").read_text(encoding="utf-8"))
    rule = config["threshold_rule"]
    k, n = 3, 5
    if rule["persistence"] != f"alarm when >= {k} of the last {n} windows exceed":
        raise ValueError(f"Unexpected persistence rule: {rule['persistence']}")
    detectors, window_sizes = load_detectors([ROOT / args.model], per_model_windows=True)
    detectors = next(value for key, value in detectors.items() if key != "pca_baseline")
    gru_config = json.loads((ROOT / args.gru / "config.json").read_text(encoding="utf-8"))
    deploy = gru_config["deploy"]
    gru = GRUForecaster(2, 2, int(gru_config["hidden"]))
    gru.load_state_dict(torch.load(ROOT / args.gru / "gru_motor.pt", weights_only=True))
    gru.eval()
    mean = {name: np.asarray(config["models"][name]["scaler_mean"]) for name in MODELS}
    scale = {name: np.asarray(config["models"][name]["scaler_scale"]) for name in MODELS}
    track_map = json.loads(MAP_PATH.read_text(encoding="utf-8"))
    mission = json.loads(MISSION_PATH.read_text(encoding="utf-8-sig"))
    if args.per_edge_odometry:
        mission.setdefault("esp_sharp_turn_deg", 85.0)  # As epuck_route sends.
    else:
        mission.pop("esp_odometry_scale_by_edge", None)

    link = serial.Serial(port=None, baudrate=115200, timeout=1.0, dsrdtr=False, rtscts=False)
    link.port = args.port
    link.dtr = False
    link.rts = False
    link.open()
    link.reset_input_buffer()
    summary = []
    failed = False
    try:
        for path in args.runs:
            rows, esp, target_mismatches = replay(link, path, track_map, mission)
            reference = python_reference(path, detectors, window_sizes, k, n,
                                         float(rule["warmup_s"]))
            gru_score, gru_alarm, stuck_alarm = gru_stuck_reference(
                path, gru, deploy, mean, scale, k, n)
            # The ESP's motor/IR alarms include the GRU / stuck rule.
            extra = {"motor": gru_alarm, "ir": stuck_alarm}
            run = {"file": path.name, "target_mismatches": target_mismatches}
            for name in MODELS:
                scores, alarms, threshold = reference[name]
                window_size = window_sizes[name]
                pairs = [(esp[r][name], scores[r - window_size], esp[r][f"{name}_alarm"],
                          alarms[r - window_size] or extra[name][r])
                         for r in range(window_size, len(rows))
                         if esp[r] is not None and esp[r][name] >= 0.0]
                esp_scores = np.array([p[0] for p in pairs])
                py_scores = np.array([p[1] for p in pairs])
                difference = np.abs(esp_scores - py_scores)
                alarm_mismatch = sum(int(p[2]) != int(p[3]) for p in pairs)
                run[name] = {
                    "windows": len(pairs),
                    "max_abs_diff": float(difference.max()),
                    "alarm_mismatches": int(alarm_mismatch),
                    "esp_alarm_windows": int(sum(p[2] for p in pairs)),
                    "python_alarm_windows": int(sum(bool(p[3]) for p in pairs)),
                }
                failed |= alarm_mismatch > 0 or difference.max() > SCORE_TOLERANCE
            compared = [r for r in range(len(rows)) if esp[r] is not None]
            gru_rows = [r for r in compared if esp[r]["gru"] >= 0.0]
            gru_difference = np.abs(np.array([esp[r]["gru"] - gru_score[r] for r in gru_rows]))
            gru_tolerance = np.maximum(GRU_SCORE_TOLERANCE,
                                       GRU_RELATIVE_TOLERANCE * np.abs(gru_score[gru_rows]))
            run["gru"] = {
                "windows": len(gru_rows),
                "max_abs_diff": float(gru_difference.max()) if gru_rows else 0.0,
                "score_at_max_diff": (float(gru_score[gru_rows[int(gru_difference.argmax())]])
                                      if gru_rows else 0.0),
                "alarm_mismatches": int(sum(esp[r]["gru_alarm"] != gru_alarm[r]
                                            for r in compared)),
                "esp_alarm_windows": sum(esp[r]["gru_alarm"] for r in compared),
                "python_alarm_windows": int(gru_alarm[compared].sum()),
            }
            run["stuck"] = {
                "alarm_mismatches": int(sum(esp[r]["stuck_alarm"] != stuck_alarm[r]
                                            for r in compared)),
                "esp_alarm_windows": sum(esp[r]["stuck_alarm"] for r in compared),
                "python_alarm_windows": int(stuck_alarm[compared].sum()),
            }
            # NaN: the ESP scored a window Python has no score for.
            failed |= (run["gru"]["alarm_mismatches"] > 0
                       or run["stuck"]["alarm_mismatches"] > 0
                       or not np.all(gru_difference <= gru_tolerance))
            failed |= target_mismatches > 0
            summary.append(run)
            print(f"{path.name}: targets mismatched {target_mismatches}; " + "; ".join(
                f"{name} {run[name]['windows']} windows, max diff {run[name]['max_abs_diff']:.2e}"
                f", alarms ESP/Python "
                f"{run[name]['esp_alarm_windows']}/{run[name]['python_alarm_windows']}, "
                f"mismatched {run[name]['alarm_mismatches']}" for name in MODELS)
                + f"; gru {run['gru']['windows']} windows, max diff "
                f"{run['gru']['max_abs_diff']:.2e}, alarms ESP/Python "
                f"{run['gru']['esp_alarm_windows']}/{run['gru']['python_alarm_windows']}, "
                f"mismatched {run['gru']['alarm_mismatches']}; stuck alarms ESP/Python "
                f"{run['stuck']['esp_alarm_windows']}/{run['stuck']['python_alarm_windows']}, "
                f"mismatched {run['stuck']['alarm_mismatches']}", flush=True)
    finally:
        link.close()
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print("AGREEMENT FAILED" if failed else "ESP scores agree with Python.")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
