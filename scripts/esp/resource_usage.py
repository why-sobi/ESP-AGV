"""Summarize the ESP's runtime resource use: heap, CPU load per core and task,
and stack peaks.

Reads the ``*.resources.jsonl`` files that ``epuck_route`` writes when
``AGV_ESP_RESOURCE_LOG_S`` is set (one "U" query per interval while driving),
and optionally takes a final "U" snapshot from the port, whose minimum-ever
heap includes the last run's notification (TLS) after its fault stop.
"""

import argparse
import json
import time
from pathlib import Path

import serial

# Task stack sizes in bytes: firmware/main (serial_control, notify) and
# firmware/sdkconfig (ESP-IDF tasks). The WiFi task's size is fixed inside
# the WiFi library.
STACK_BYTES = {
    "serial_control": 12288,
    "notify": 8192,
    "IDLE0": 1536,
    "IDLE1": 1536,
    "tcpip": 3072,
    "Tmr Svc": 2048,
    "esp_timer": 3584,
    "sys_evt": 2304,
    "ipc0": 1280,
    "ipc1": 1280,
}
FRAME_US = 32000  # Webots time step: one sensor frame per 32 ms.
HEAP_FIELDS = (
    "heap_free", "heap_min_free", "heap_largest_block", "internal_free",
    "internal_min_free", "psram_total", "psram_free", "heap_total", "tasks", "window_us",
)


def query(port_name):
    port = serial.Serial(
        port=None, baudrate=115200, timeout=0.5, write_timeout=1.0, dsrdtr=False, rtscts=False
    )
    port.port = port_name
    port.dtr = False
    port.rts = False
    port.open()
    try:
        time.sleep(0.5)
        port.reset_input_buffer()
        port.write(b"U\n")
        fields = port.readline().decode("ascii").strip().split(",")
        if fields[0] != "U" or len(fields) != len(HEAP_FIELDS) + 1:
            raise RuntimeError(f"Malformed ESP resource response: {fields!r}")
        usage = dict(zip(HEAP_FIELDS, (int(value) for value in fields[1:])))
        usage["task_list"] = []
        for _ in range(usage["tasks"]):
            name, core, permille, stack = port.readline().decode("ascii").strip().split(",")[1:]
            usage["task_list"].append({
                "name": name, "core": int(core), "cpu_permille": int(permille),
                "stack_free_min": int(stack),
            })
        return usage
    finally:
        port.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=Path, required=True, help="directory of *.resources.jsonl")
    parser.add_argument("--out", type=Path, required=True, help="report directory")
    parser.add_argument("--port", help="also take a final snapshot from this port (e.g. COM3)")
    args = parser.parse_args()

    samples = []
    for path in sorted(args.runs.glob("*.resources.jsonl")):
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                sample = json.loads(line)
                sample["run"] = path.name.removesuffix(".resources.jsonl")
                samples.append(sample)
    if not samples:
        raise SystemExit(f"No *.resources.jsonl in {args.runs}")
    final = query(args.port) if args.port else None
    snapshots = samples + ([final] if final else [])

    # Webots runs in fast mode, so frames come faster than one per 32 ms of
    # wall time. The CPU time per frame does not depend on that; scale it to
    # real time (one frame per FRAME_US).
    per_frame_us = {}
    for previous, sample in zip(samples, samples[1:]):
        # The ESP gets no frames while the host unloads at the destination.
        if previous["run"] != sample["run"] or "UNLOADING" in (
            previous["esp_state"], sample["esp_state"]
        ):
            continue
        frames = round((sample["time"] - previous["time"]) * 1e6 / FRAME_US)
        if frames <= 0:
            continue
        for task in sample["task_list"]:
            per_frame_us.setdefault(task["name"], []).append(
                task["cpu_permille"] / 1000 * sample["window_us"] / frames
            )

    # Driving windows only: the first query of a run starts its CPU window.
    tasks = {}
    for sample in samples:
        for task in sample["task_list"]:
            entry = tasks.setdefault(task["name"], {
                "core": task["core"], "cpu_permille": [], "stack_free_min": task["stack_free_min"],
            })
            entry["cpu_permille"].append(task["cpu_permille"])
    for snapshot in snapshots:
        for task in snapshot["task_list"]:
            if task["name"] in tasks:
                tasks[task["name"]]["stack_free_min"] = min(
                    tasks[task["name"]]["stack_free_min"], task["stack_free_min"]
                )

    summary = {
        "runs": sorted({sample["run"] for sample in samples}),
        "queries": len(samples),
        "heap_total": samples[0]["heap_total"],
        "psram_total": samples[0]["psram_total"],
        "heap_free_min_while_driving": min(sample["heap_free"] for sample in samples),
        "heap_free_max_while_driving": max(sample["heap_free"] for sample in samples),
        "heap_min_free_ever": min(snapshot["heap_min_free"] for snapshot in snapshots),
        "heap_largest_block_min": min(snapshot["heap_largest_block"] for snapshot in snapshots),
        "final_snapshot": final,
        "cores": {},
        "tasks": {},
    }
    for core in (0, 1):
        idle = tasks.get(f"IDLE{core}")
        if idle:
            loads = [1000 - value for value in idle["cpu_permille"]]
            summary["cores"][core] = {
                "load_mean_pct": round(sum(loads) / len(loads) / 10, 2),
                "load_max_pct": round(max(loads) / 10, 2),
            }
    for name, entry in sorted(tasks.items(), key=lambda item: -sum(item[1]["cpu_permille"])):
        values = entry["cpu_permille"]
        stack = STACK_BYTES.get(name)
        summary["tasks"][name] = {
            "core": entry["core"],
            "cpu_mean_pct": round(sum(values) / len(values) / 10, 2),
            "cpu_max_pct": round(max(values) / 10, 2),
            "stack_bytes": stack,
            "stack_free_min": entry["stack_free_min"],
            "stack_peak_used": stack - entry["stack_free_min"] if stack else None,
        }
        frame_times = per_frame_us.get(name)
        if frame_times and name == "serial_control":
            mean_us = sum(frame_times) / len(frame_times)
            summary["tasks"][name].update(
                cpu_us_per_frame_mean=round(mean_us, 1),
                cpu_us_per_frame_max=round(max(frame_times), 1),
                realtime_load_pct=round(mean_us / FRAME_US * 100, 2),
            )

    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in summary.items() if key != "final_snapshot"},
                     indent=2))


if __name__ == "__main__":
    main()
