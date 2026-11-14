"""Record epuck_route runs headlessly in Webots over a route x speed matrix.

Each run gets its own temporary mission file (copied from the base mission
with start/destination/speed replaced), is launched as a separate headless
Webots process, and is checked and appended to the run manifest.
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

ROOT = Path(__file__).resolve().parent.parent
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
)

# Two-edge routes on base track whose runs exceed the 10 s dataset minimum.
ROUTES = {
    "A": ("node-003", "node-001"),
    "B": ("node-001", "node-003"),
    "C": ("node-002", "node-004"),
    "D": ("node-004", "node-002"),
}


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


def summarize_run(path, max_irregular=0):
    """Return (healthy, notes) for a route CSV using the manual-review checks."""
    with path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    if len(rows) < 2:
        return False, "fewer than two samples"
    times = [float(row["time"]) for row in rows]
    intervals = [b - a for a, b in zip(times, times[1:])]
    median = sorted(intervals)[len(intervals) // 2]
    irregular = sum(1 for dt in intervals if abs(dt - median) > max(0.001, 0.1 * median))
    line_loss = [float(row["time"]) for row in rows if row["line_error"] == ""]
    late_loss = [t for t in line_loss if t > 0.5]
    max_error = max(float(row["centerline_error_m"]) for row in rows)
    duration = times[-1] - times[0]
    notes = (
        f"{duration:.1f} s; {len(line_loss)} line-loss samples"
        f" ({len(late_loss)} after 0.5 s); {irregular} irregular intervals;"
        f" max centerline error {max_error:.3f} m"
    )
    healthy = not late_loss and irregular <= max_irregular and max_error < 0.05
    return healthy, notes


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


def run_one(webots, world, base_mission, route, speed, out_dir, timeout):
    start, destination = ROUTES[route]
    mission = dict(
        base_mission,
        start_node=start,
        destination_node=destination,
        wheel_speed_rad_s=speed,
        run_label=f"{route}-{speed:g}",
    )
    mission.pop("fault", None)
    path, reached, output, elapsed = run_mission(webots, world, mission, out_dir, timeout)
    if path is None:
        return None, start, destination, speed, False, (
            "expected one new CSV; output tail: "
            + " | ".join(output.strip().splitlines()[-3:])
        ), elapsed
    healthy, notes = summarize_run(path)
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
    args = parser.parse_args()

    unknown = set(args.routes) - set(ROUTES)
    if unknown:
        parser.error(f"unknown routes: {''.join(sorted(unknown))}")
    webots = find_webots(args.webots)
    base_mission = json.loads(args.mission.read_text(encoding="utf-8-sig"))
    out_dir = args.out.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest = args.manifest or out_dir / "manifest.csv"
    write_header = not manifest.exists()

    failures = 0
    with manifest.open("a", newline="", encoding="utf-8") as manifest_file:
        writer = csv.writer(manifest_file)
        if write_header:
            writer.writerow(MANIFEST_FIELDS)
        for route in args.routes:
            for speed in args.speeds:
                path, start, destination, speed, healthy, notes, elapsed = run_one(
                    webots, args.world, base_mission, route, speed, out_dir, args.timeout
                )
                status = "healthy" if healthy else "failed"
                failures += not healthy
                name = path.name if path else ""
                print(f"{route} {start}->{destination} @ {speed:g}: {status} "
                      f"({elapsed:.0f} s wall) {name} | {notes}", flush=True)
                if path:
                    writer.writerow([name, start, destination, speed, status, notes])
                    manifest_file.flush()
    print(f"Done: {failures} failed run(s).")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
