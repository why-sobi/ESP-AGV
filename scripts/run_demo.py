"""Open Webots on a track and drive one route, clean or with a fault, for a demo.

By default this opens the Webots window (real time; ``--fast`` runs as fast
as the ESP allows) and leaves it open at the end; pressing Reset in Webots
replays the same run. ``--headless`` runs without a window, like the
collectors, and prints how the run ended. Either way it writes a temporary
mission and points the controller at it, so ``maps/routes/mission.json`` is
never edited. The run CSV goes to ``--out``.

Faults are presets from the ESP fault matrix
(``data/esp_runs/faults/fault-matrix-esp-v1.json``); ``--target`` and
``--severity`` override them. A fault stop (and the phone notification) needs
an alarm lasting 2 s before the route ends. weak-motor and ir-dead get there
within a second; ir-noise (std 200) can flicker below that on some routes
(``--severity 300`` stops reliably); ir-stuck usually stops; stall and
intermittent faults often end in a line-loss stop first; ir-drift is often
not detected.
"""

import argparse
import csv
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

from scripts.telemetry.collect_route_runs import ROOT, TRACKS, find_webots, run_mission

DEFAULT_OUT = ROOT / "data" / "route_runs" / "demo"
FAULTS = {
    "weak-motor": {"type": "motor_gain", "target": "right", "severity": 0.7},
    "stall": {"type": "motor_stall", "target": "left"},
    "intermittent": {"type": "motor_intermittent", "target": "right",
                     "period_s": 1.0, "burst_s": 0.15},
    "encoder-stuck": {"type": "encoder_stuck", "target": "left"},
    "encoder-noise": {"type": "encoder_noise", "target": "right", "severity": 0.02},
    "ir-stuck": {"type": "ir_stuck", "target": "gs_center"},
    "ir-dead": {"type": "ir_dead", "target": "gs_left", "severity": 0.0},
    "ir-noise": {"type": "ir_noise", "target": "gs_right", "severity": 200.0},
    "ir-drift": {"type": "ir_drift", "target": "gs_center", "severity": 100.0},
}


def report(path, fault, elapsed, completed, output):
    """Print how a run ended: final state, first alarms after the onset."""
    with path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    last = rows[-1]
    print(f"{path.name}: {float(last['time']):.1f} s simulated in {elapsed:.0f} s, "
          f"ended {last.get('esp_state') or 'supervisor run'}")
    onset = float(fault["onset_s"]) if fault else 0.0
    for column, name in (("esp_motor_alarm", "motor alarm"), ("esp_ir_alarm", "IR alarm")):
        times = [float(row["time"]) for row in rows if row.get(column) == "1"]
        if times:
            after = f"{times[0] - onset:.2f} s after the onset" if fault else "no fault injected"
            print(f"  first {name} at {times[0]:.2f} s ({after})")
    if last.get("esp_state") == "FAULT_STOP":
        print("  fault stop: the ESP sent a phone notification")
    elif not completed:
        # The controller's reason, if Webots forwarded it before quitting.
        reasons = [line.strip() for line in output.splitlines()
                   if "RUN FAILED" in line or "ESP stopped" in line]
        print("  the controller counted the run as failed"
              + (": " + " / ".join(reasons) if reasons else ""))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--track", choices=sorted(TRACKS), default="base")
    parser.add_argument("--start", default="node-001", help="Start node (default node-001)")
    parser.add_argument("--destination", default="node-004",
                        help="Destination node (default node-004)")
    parser.add_argument("--speed", type=float, default=3.0, help="Wheel speed, rad/s")
    parser.add_argument("--fault", choices=sorted(FAULTS), help="Inject this fault (default: none)")
    parser.add_argument("--target", help="Override the fault's wheel (left/right) or sensor "
                                         "(gs_left/gs_center/gs_right)")
    parser.add_argument("--severity", type=float, help="Override the fault's severity")
    parser.add_argument("--onset", type=float, default=6.0, help="Fault onset, s (default 6)")
    parser.add_argument("--seed", type=int,
                        help="Start-pose jitter seed (3 mm, 4 deg); default: no jitter")
    parser.add_argument("--one-way", action="store_true", help="No return leg")
    parser.add_argument("--no-fault-stop", action="store_true",
                        help="Only report alarms; do not pull over, stop and notify")
    parser.add_argument("--supervisor", action="store_true",
                        help="Drive with the simulator's controller instead of the ESP")
    parser.add_argument("--fast", action="store_true",
                        help="Window, but as fast as the ESP allows instead of real time")
    parser.add_argument("--headless", action="store_true",
                        help="No window; print how the run ended")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--webots", help="Path to the Webots executable")
    args = parser.parse_args()

    mission_path, world = TRACKS[args.track]
    mission = json.loads(mission_path.read_text(encoding="utf-8-sig"))
    map_path = (ROOT / mission["map_path"] if mission.get("map_path")
                else ROOT / "maps" / "paths" / "base track.json")
    nodes = {node["id"] for node in json.loads(map_path.read_text(encoding="utf-8"))["nodes"]}
    for node in (args.start, args.destination):
        if node not in nodes:
            parser.error(f"{node} is not a node of {map_path.name}: {', '.join(sorted(nodes))}")
    esp = not args.supervisor
    if not esp and not args.one_way:
        parser.error("the return leg needs the ESP: add --one-way with --supervisor")

    label = args.fault or "clean"
    mission.update(
        start_node=args.start,
        destination_node=args.destination,
        wheel_speed_rad_s=args.speed,
        control_mode="esp32" if esp else "supervisor",
        return_to_start=esp and not args.one_way,
        spawn_at_start_node=True,
        quit_webots_on_complete=False,
        run_label=f"demo-{args.track}-{args.start}-{args.destination}-{label}",
    )
    mission.pop("fault", None)
    mission.pop("spawn_jitter", None)
    if esp:
        mission["esp_speed_rad_s"] = args.speed
        if args.no_fault_stop:
            mission.pop("esp_fault_stop_s", None)
        else:
            mission["esp_fault_stop_s"] = 2.0
    if args.seed is not None:
        mission["spawn_jitter"] = {"lateral_m": 0.003, "heading_deg": 4.0, "seed": args.seed}
    if args.fault:
        fault = dict(FAULTS[args.fault], onset_s=args.onset)
        if args.target:
            fault["target"] = args.target
        if args.severity is not None:
            fault["severity"] = args.severity
        mission["fault"] = fault

    print(f"{args.track}: {args.start} -> {args.destination} at {args.speed:g} rad/s, "
          f"{'ESP' if esp else 'Supervisor'}, "
          f"{'one way' if not mission['return_to_start'] else 'round trip'}, "
          f"fault: {mission.get('fault', 'none')}")
    args.out.mkdir(parents=True, exist_ok=True)
    if args.headless:
        path, completed, output, elapsed = run_mission(
            find_webots(args.webots), world, mission, args.out.resolve(), timeout=300
        )
        if path is None:
            sys.exit("No run CSV; Webots output tail:\n" + "\n".join(output.splitlines()[-5:]))
        report(path, mission.get("fault"), elapsed, completed, output)
        return
    print("Close the Webots window to finish; Reset (Ctrl+Shift+T) replays the run.")
    with tempfile.TemporaryDirectory() as temporary:
        temp_mission = Path(temporary) / "mission.json"
        temp_mission.write_text(json.dumps(mission, indent=2), encoding="utf-8")
        env = dict(
            os.environ,
            AGV_MISSION_PATH=str(temp_mission),
            AGV_ROUTE_PATH=str(Path(temporary) / "active_route.json"),
            AGV_LOG_DIR=str(args.out.resolve()),
        )
        result = subprocess.run([str(find_webots(args.webots)),
                                 "--mode=fast" if args.fast else "--mode=realtime", str(world)],
                                env=env)
    print(f"Run CSVs in {args.out}")
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
