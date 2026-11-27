"""Record epuck_route runs headlessly in Webots over a route x speed matrix.

Each run gets its own temporary mission file (copied from the base mission
with start/destination/speed replaced), is launched as a separate headless
Webots process, and is checked and appended to the run manifest.

With ``--control-mode esp32`` the ESP on the mission's serial port drives the
robot and the speed is sent as its route speed (``esp_speed_rad_s``);
``--round-trip`` adds the return leg. ``--seeds`` repeats every route/speed
with a seeded start-pose jitter, since Webots runs are otherwise identical.
"""

import argparse
import csv
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_WORLD = ROOT / "simulation" / "worlds" / "Self Diagnosis AGV.wbt"
DEFAULT_MISSION = ROOT / "maps" / "routes" / "mission.json"
DEFAULT_OUT = ROOT / "data" / "route_runs"
MANIFEST_FIELDS = (
    "file",
    "start_node",
    "destination_node",
    "wheel_speed_rad_s",
    "status",
    "notes",
    "control_mode",
    "round_trip",
    "seed",
)

# Longest line-loss run accepted in a healthy ESP run: the firmware's 750 ms
# line-search limit (kTurnSearchLimitMs) at the 32 ms time step.
ESP_MAX_LOSS_SAMPLES = 23

# Two-edge routes on base track whose runs exceed the 10 s dataset minimum.
ROUTES = {
    "A": ("node-003", "node-001"),
    "B": ("node-001", "node-003"),
    "C": ("node-002", "node-004"),
    "D": ("node-004", "node-002"),
}


def route_spec(mission, route):
    """(start, destination, avoid_edges) of a route letter: from the mission's
    ``routes`` table when it has one (another track, e.g. track2), else the base
    track's ROUTES."""
    table = mission.get("routes")
    if table:
        spec = table[route]
        return spec["start"], spec["destination"], spec.get("avoid_edges")
    start, destination = ROUTES[route]
    return start, destination, None


def find_webots(explicit=None):
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    for base in (
        os.environ.get("WEBOTS_HOME"),
        Path(os.environ.get("LOCALAPPDATA", "")) / "Programs" / "Webots",
        Path("C:/Program Files/Webots"),
    ):
        if base:
            candidates.append(Path(base) / "msys64" / "mingw64" / "bin" / "webots.exe")
            candidates.append(Path(base) / "webots")
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("Webots not found; pass --webots or set WEBOTS_HOME.")


def summarize_run(path, max_irregular=0, max_loss_streak=0):
    """Return (healthy, notes) for a route CSV using the manual-review checks.

    ``max_loss_streak`` is the longest tolerated run of consecutive line-loss
    samples after 0.5 s. The Supervisor controller tolerates none; the ESP's
    bounded line search rides through single weak frames on corners.
    """
    with path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    if len(rows) < 2:
        return False, "fewer than two samples"
    times = [float(row["time"]) for row in rows]
    intervals = [b - a for a, b in zip(times, times[1:])]
    median = sorted(intervals)[len(intervals) // 2]
    irregular = sum(1 for dt in intervals if abs(dt - median) > max(0.001, 0.1 * median))
    # A U-turn and a corner turn (spinning onto a crossing line) sweep the
    # sensors off the line on purpose.
    lost = [
        row["line_error"] == ""
        and row.get("esp_state") not in ("U_TURN", "UNLOADING", "TURN_LEFT", "TURN_RIGHT")
        for row in rows
    ]
    # The ESP runs in blind for the last few cm to a destination (past the end
    # of a line or across a T's crossbar; firmware kArrivalSlackM): a loss that
    # ends in AT_DEST is arriving, not losing the line.
    arriving = [False] * len(rows)
    for i in range(len(rows) - 1, -1, -1):
        arriving[i] = rows[i].get("esp_state") == "AT_DEST" or (
            lost[i] and i + 1 < len(rows) and arriving[i + 1])
    late = [is_lost and not arrive and time > 0.5
            for is_lost, arrive, time in zip(lost, arriving, times)]
    longest = streak = 0
    for is_late in late:
        streak = streak + 1 if is_late else 0
        longest = max(longest, streak)
    max_error = max(float(row["centerline_error_m"]) for row in rows)
    duration = times[-1] - times[0]
    notes = (
        f"{duration:.1f} s; {sum(lost)} line-loss samples"
        f" ({sum(late)} after 0.5 s, longest run {longest}); {irregular} irregular intervals;"
        f" max centerline error {max_error:.3f} m"
    )
    healthy = longest <= max_loss_streak and irregular <= max_irregular and max_error < 0.05
    return healthy, notes


def serial_error_since(out_dir, offset):
    """True if controller-errors.log gained a serial-port failure after ``offset`` bytes.

    The ESP's USB serial link occasionally drops for a moment; such a run says
    nothing about the robot and should be repeated, not counted as a failure.
    """
    log = Path(out_dir) / "controller-errors.log"
    if not log.exists():
        return False
    with log.open("rb") as file:
        file.seek(offset)
        return b"SerialException" in file.read()


def error_log_size(out_dir):
    log = Path(out_dir) / "controller-errors.log"
    return log.stat().st_size if log.exists() else 0


def run_mission(webots, world, mission, out_dir, timeout):
    """Run one headless Webots session with ``mission``.

    Returns (new CSV path or None, completed, console output, wall seconds).
    ``completed`` is True only when the controller quit with status 0.
    """
    mission = dict(mission, spawn_at_start_node=True, quit_webots_on_complete=True)
    with tempfile.TemporaryDirectory() as temporary:
        mission_path = Path(temporary) / "mission.json"
        mission_path.write_text(json.dumps(mission, indent=2), encoding="utf-8")
        env = dict(
            os.environ,
            AGV_MISSION_PATH=str(mission_path),
            AGV_ROUTE_PATH=str(Path(temporary) / "active_route.json"),
            AGV_LOG_DIR=str(out_dir),
        )
        before = set(out_dir.glob("route-*.csv"))
        command = [
            str(webots),
            "--batch",
            "--mode=fast",
            "--no-rendering",
            "--minimize",
            "--stdout",
            "--stderr",
            str(world),
        ]
        started = time.monotonic()
        # Webots stops forwarding controller output once the controller calls
        # simulationQuit, so the final console lines are unreliable. The exit
        # code is the controller's quit status: 0 only when the route completed.
        try:
            result = subprocess.run(
                command, env=env, capture_output=True, text=True, timeout=timeout
            )
            output = result.stdout + result.stderr
            reached = result.returncode == 0
        except subprocess.TimeoutExpired as error:
            output = f"timed out after {timeout} s\n{error.stdout or ''}{error.stderr or ''}"
            reached = False
        elapsed = time.monotonic() - started

    new_files = sorted(set(out_dir.glob("route-*.csv")) - before)
    return (new_files[0] if len(new_files) == 1 else None), reached, output, elapsed


def run_one(webots, world, base_mission, route, speed, out_dir, timeout, seed=None,
            jitter=None):
    start, destination, avoid = route_spec(base_mission, route)
    mission = dict(
        base_mission,
        start_node=start,
        destination_node=destination,
        wheel_speed_rad_s=speed,
        run_label=f"{route}-{speed:g}" + ("" if seed is None else f"-s{seed}"),
    )
    if avoid:
        mission["avoid_edges"] = avoid
    if mission.get("control_mode") == "esp32":
        mission["esp_speed_rad_s"] = speed
    if seed is not None:
        mission["spawn_jitter"] = dict(jitter or {}, seed=seed)
    mission.pop("fault", None)
    path, reached, output, elapsed = run_mission(webots, world, mission, out_dir, timeout)
    if path is None:
        return None, start, destination, speed, False, (
            "expected one new CSV; output tail: "
            + " | ".join(output.strip().splitlines()[-3:])
        ), elapsed
    # The ESP searches for a lost line for up to 750 ms (23 samples) before it
    # stops with LINE_LOST, so a completed ESP run never exceeds that. Its sensors
    # briefly leave the line on corners and when running onto a terminal node.
    healthy, notes = summarize_run(
        path, max_loss_streak=ESP_MAX_LOSS_SAMPLES if mission.get("control_mode") == "esp32" else 0
    )
    if not reached:
        healthy = False
        notes = "controller did not report completion (non-zero exit or timeout); " + notes
    return path, start, destination, speed, healthy, notes, elapsed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--routes", default="ABCD", help="Route letters from ROUTES, e.g. ABCD")
    parser.add_argument("--speeds", nargs="+", type=float, default=[2.0, 2.5, 3.0, 3.5])
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--mission", type=Path, default=DEFAULT_MISSION, help="Base mission")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--manifest", type=Path, help="Defaults to <out>/manifest.csv")
    parser.add_argument("--webots", help="Path to the Webots executable")
    parser.add_argument("--timeout", type=float, default=180.0, help="Wall seconds per run")
    parser.add_argument("--control-mode", choices=("supervisor", "esp32"),
                        help="Override the mission's control_mode")
    parser.add_argument("--round-trip", action="store_true",
                        help="Return to the start after unloading (esp32 only)")
    parser.add_argument("--seeds", nargs="+", type=int,
                        help="Repeat each route/speed once per start-pose jitter seed")
    parser.add_argument("--jitter-lateral-m", type=float, default=0.003)
    parser.add_argument("--jitter-heading-deg", type=float, default=4.0)
    parser.add_argument("--serial-retries", type=int, default=3,
                        help="Repeat a run this many times if the ESP serial link drops")
    parser.add_argument("--resume", action="store_true",
                        help="Skip route/speed/seed runs already healthy in the manifest")
    parser.add_argument("--avoid-edges", nargs="+", default=None,
                        help="Plan routes without these edges (mission avoid_edges), e.g. "
                             "edge-005 to drive edge-004")
    args = parser.parse_args()

    webots = find_webots(args.webots)
    base_mission = json.loads(args.mission.read_text(encoding="utf-8-sig"))
    unknown = set(args.routes) - set(base_mission.get("routes") or ROUTES)
    if unknown:
        parser.error(f"unknown routes: {''.join(sorted(unknown))}")
    # Dataset runs only report alarms: a fault stop would end them early.
    base_mission.pop("esp_fault_stop_s", None)
    if args.control_mode:
        base_mission["control_mode"] = args.control_mode
    base_mission["return_to_start"] = args.round_trip
    if args.avoid_edges:
        base_mission["avoid_edges"] = args.avoid_edges
    if args.round_trip and base_mission.get("control_mode") != "esp32":
        parser.error("--round-trip needs control mode esp32")
    jitter = {"lateral_m": args.jitter_lateral_m, "heading_deg": args.jitter_heading_deg}
    out_dir = args.out.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest = args.manifest or out_dir / "manifest.csv"
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
                (row["start_node"], float(row["wheel_speed_rad_s"]), row.get("seed", ""))
                for row in csv.DictReader(existing) if row["status"] == "healthy"
            }

    failures = 0
    with manifest.open("a", newline="", encoding="utf-8") as manifest_file:
        writer = csv.DictWriter(manifest_file, fieldnames=fields, extrasaction="ignore")
        if write_header:
            writer.writeheader()
        for route in args.routes:
            for speed in args.speeds:
                for seed in args.seeds or [None]:
                    key = (route_spec(base_mission, route)[0], speed,
                           "" if seed is None else str(seed))
                    if key in done:
                        continue
                    for attempt in range(args.serial_retries + 1):
                        log_offset = error_log_size(out_dir)
                        path, start, destination, speed, healthy, notes, elapsed = run_one(
                            webots, args.world, base_mission, route, speed, out_dir,
                            args.timeout, seed, jitter,
                        )
                        link_dropped = not healthy and serial_error_since(out_dir, log_offset)
                        status = ("serial_error" if link_dropped
                                  else "healthy" if healthy else "failed")
                        name = path.name if path else ""
                        print(f"{route} {start}->{destination} @ {speed:g} seed {seed}: "
                              f"{status} ({elapsed:.0f} s wall) {name} | {notes}", flush=True)
                        if path:
                            writer.writerow({
                                "file": name, "start_node": start,
                                "destination_node": destination, "wheel_speed_rad_s": speed,
                                "status": status, "notes": notes,
                                "control_mode": base_mission.get("control_mode", "supervisor"),
                                "round_trip": int(args.round_trip),
                                "seed": "" if seed is None else seed,
                            })
                            manifest_file.flush()
                        if not link_dropped:
                            break
                        time.sleep(5.0)  # Let the USB serial device settle.
                    failures += not healthy
    print(f"Done: {failures} failed run(s).")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
