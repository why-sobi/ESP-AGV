"""Validate Webots telemetry and prepare run-separated autoencoder windows."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np


MOTOR_FEATURES = (
    "target_vel_left",
    "target_vel_right",
    "actual_vel_left",
    "actual_vel_right",
)
IR_FEATURES = ("gs_left", "gs_center", "gs_right")
SPLIT_NAMES = ("train", "validation", "test")
REPO_ROOT = Path(__file__).resolve().parent.parent


def load_split(path):
    """Read an explicit run split; relative run paths resolve from the repo root."""
    path = Path(path)
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    split = {}
    for name in SPLIT_NAMES:
        entries = data.get(name, [])
        if not isinstance(entries, list) or not all(isinstance(item, str) for item in entries):
            raise ValueError(f"{path}: '{name}' must be a list of file paths")
        split[name] = [
            Path(item) if Path(item).is_absolute() else REPO_ROOT / item
            for item in entries
        ]
    return split


def _column(row, *names):
    for name in names:
        value = row.get(name)
        if value not in (None, ""):
            return float(value)
    raise ValueError(f"Missing telemetry column; expected one of {names}")


def _read_segments(path, minimum_duration):
    required = ("time", "target_vel_left", "target_vel_right", "actual_pos_left", "actual_pos_right")
    segments = []
    with path.open(newline="", encoding="utf-8-sig") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None:
            raise ValueError(f"{path} has no CSV header")
        missing = [name for name in required if name not in reader.fieldnames]
        if missing:
            raise ValueError(f"{path} is missing required columns: {', '.join(missing)}")

        current = []
        prior_time = None
        segment_number = 0
        for line_number, row in enumerate(reader, start=2):
            try:
                time_value = _column(row, "time")
            except (TypeError, ValueError) as error:
                raise ValueError(f"{path}:{line_number}: invalid time value") from error
            if prior_time is not None and time_value <= prior_time:
                if current:
                    segments.append((segment_number, current))
                    segment_number += 1
                current = []
            current.append(row)
            prior_time = time_value
        if current:
            segments.append((segment_number, current))

    accepted = []
    excluded = []
    for index, rows in segments:
        if len(rows) < 2:
            excluded.append({"episode": index, "rows": len(rows), "reason": "fewer than two samples"})
            continue
        start = float(rows[0]["time"])
        end = float(rows[-1]["time"])
        duration = end - start
        if duration < minimum_duration:
            excluded.append(
                {
                    "episode": index,
                    "rows": len(rows),
                    "duration_seconds": duration,
                    "reason": "shorter than minimum duration",
                }
            )
            continue

        run_id = rows[0].get("run_id") or f"{path.stem}#episode-{index}"
        accepted.append((run_id, rows))
    return accepted, excluded


def _features(rows, run_id, window_size):
    times = np.asarray([_column(row, "time") for row in rows], dtype=np.float64)
    target_left = np.asarray([_column(row, "target_vel_left") for row in rows])
    target_right = np.asarray([_column(row, "target_vel_right") for row in rows])
    position_left = np.asarray([_column(row, "actual_pos_left") for row in rows])
    position_right = np.asarray([_column(row, "actual_pos_right") for row in rows])
    sensor_left = np.asarray([_column(row, "gs_left", "gs0") for row in rows])
    sensor_center = np.asarray([_column(row, "gs_center", "gs1") for row in rows])
    sensor_right = np.asarray([_column(row, "gs_right", "gs2") for row in rows])

    delta_time = np.diff(times)
    if np.any(delta_time <= 0):
        raise ValueError(f"{run_id} has non-increasing timestamps inside an episode")
    actual_left = np.diff(position_left) / delta_time
    actual_right = np.diff(position_right) / delta_time

    motor = np.column_stack(
        (target_left[1:], target_right[1:], actual_left, actual_right)
    )
    ir = np.column_stack((sensor_left[1:], sensor_center[1:], sensor_right[1:]))
    sample_times = times[1:]
    if not np.isfinite(motor).all() or not np.isfinite(ir).all():
        raise ValueError(f"{run_id} contains NaN or infinite telemetry")
    if len(motor) < window_size:
        raise ValueError(
            f"{run_id} has {len(motor)} derived samples, fewer than window size {window_size}"
        )

    intervals = delta_time
    median_interval = float(np.median(intervals))
    tolerance = max(0.001, median_interval * 0.1)
    irregular_intervals = int(np.count_nonzero(np.abs(intervals - median_interval) > tolerance))
    all_sensors_bright = int(
        np.count_nonzero((sensor_left[1:] > 400) & (sensor_center[1:] > 400) & (sensor_right[1:] > 400))
    )
    summary = {
        "rows": len(rows),
        "duration_seconds": float(times[-1] - times[0]),
        "median_interval_seconds": median_interval,
        "irregular_intervals": irregular_intervals,
        "all_sensors_bright_samples": all_sensors_bright,
        "all_sensors_bright_fraction": all_sensors_bright / len(ir),
        "controller": rows[0].get("controller"),
        "route_start": rows[0].get("route_start"),
        "route_destination": rows[0].get("route_destination"),
        "edge_ids": sorted(
            {row.get("edge_id") for row in rows if row.get("edge_id")}
        ),
        "mean_target_wheel_speed_rad_s": float(
            np.mean((target_left[1:] + target_right[1:]) * 0.5)
        ),
    }
    return motor, ir, sample_times, summary


def _fit_scaler(matrices):
    joined = np.concatenate(matrices, axis=0)
    mean = joined.mean(axis=0)
    scale = joined.std(axis=0)
    scale[scale == 0] = 1.0
    return mean, scale


def _make_windows(records, indices, window_size, stride, motor_mean, motor_scale, ir_mean, ir_scale):
    motor_windows = []
    ir_windows = []
    run_indices = []
    start_times = []
    split_runs = []

    for run_index in indices:
        run_id, motor, ir, times, _ = records[run_index]
        first_window = len(motor_windows)
        for start in range(0, len(motor) - window_size + 1, stride):
            end = start + window_size
            motor_windows.append((motor[start:end] - motor_mean) / motor_scale)
            ir_windows.append((ir[start:end] - ir_mean) / ir_scale)
            run_indices.append(run_index)
            start_times.append(times[start])
        split_runs.append(
            {
                "run_id": run_id,
                "window_start": first_window,
                "window_count": len(motor_windows) - first_window,
            }
        )

    feature_count_motor = len(MOTOR_FEATURES)
    feature_count_ir = len(IR_FEATURES)
    motor_array = np.asarray(motor_windows, dtype=np.float32).reshape(
        -1, window_size, feature_count_motor
    )
    ir_array = np.asarray(ir_windows, dtype=np.float32).reshape(
        -1, window_size, feature_count_ir
    )
    return (
        motor_array,
        ir_array,
        np.asarray(run_indices, dtype=np.int32),
        np.asarray(start_times, dtype=np.float64),
        split_runs,
    )


def prepare_dataset(
    input_paths,
    output_dir,
    window_size=20,
    stride=1,
    validation_fraction=0.25,
    minimum_duration=10.0,
    split=None,
    split_source=None,
):
    """Prepare windows from telemetry runs.

    With ``split`` (a mapping of ``train``/``validation``/``test`` to file
    lists), ``input_paths`` is ignored and runs keep their assigned split.
    Otherwise the last ``validation_fraction`` of runs, by run id, becomes
    validation and there is no test split.
    """
    if window_size < 2 or stride < 1:
        raise ValueError("window_size must be at least 2 and stride must be positive")
    if not 0.0 < validation_fraction < 1.0:
        raise ValueError("validation_fraction must be between 0 and 1")
    if minimum_duration < 0:
        raise ValueError("minimum_duration cannot be negative")

    file_split = {}
    if split is not None:
        unknown = set(split) - set(SPLIT_NAMES)
        if unknown:
            raise ValueError(f"Unknown split names: {', '.join(sorted(unknown))}")
        input_paths = []
        for name in SPLIT_NAMES:
            for path in split.get(name, []):
                key = Path(path).resolve()
                if key in file_split:
                    raise ValueError(
                        f"{path} is assigned to both {file_split[key]} and {name}"
                    )
                file_split[key] = name
                input_paths.append(path)

    records = []
    run_split = {}
    excluded = []
    for path in input_paths:
        path = Path(path)
        segments, skipped = _read_segments(path, minimum_duration)
        excluded.extend({"file": str(path), **item} for item in skipped)
        for run_id, rows in segments:
            motor, ir, times, summary = _features(rows, run_id, window_size)
            if split is not None:
                summary["assigned_split"] = file_split[path.resolve()]
                run_split[run_id] = summary["assigned_split"]
            records.append((run_id, motor, ir, times, summary))

    records.sort(key=lambda record: record[0])
    unique_records = []
    seen_signatures = {}
    for record in records:
        signature = hashlib.sha256(
            record[1].tobytes() + record[2].tobytes()
        ).hexdigest()
        previous_run = seen_signatures.get(signature)
        if previous_run is not None:
            excluded.append(
                {
                    "run_id": record[0],
                    "reason": f"telemetry is identical to {previous_run}",
                }
            )
            continue
        seen_signatures[signature] = record[0]
        unique_records.append(record)
    records = unique_records

    if not records:
        raise ValueError(
            "No usable telemetry runs remain after duration and duplicate checks."
        )

    if split is not None:
        indices = {
            name: [i for i, record in enumerate(records) if run_split[record[0]] == name]
            for name in SPLIT_NAMES
        }
        if not indices["train"]:
            raise ValueError("The explicit split has no usable training runs.")
        train_indices = indices["train"]
        validation_indices = indices["validation"]
        test_indices = indices["test"]
    else:
        validation_count = 0
        if len(records) >= 2:
            validation_count = max(1, math.ceil(len(records) * validation_fraction))
            validation_count = min(validation_count, len(records) - 1)
        train_indices = list(range(0, len(records) - validation_count))
        validation_indices = list(range(len(records) - validation_count, len(records)))
        test_indices = []

    motor_mean, motor_scale = _fit_scaler([records[i][1] for i in train_indices])
    ir_mean, ir_scale = _fit_scaler([records[i][2] for i in train_indices])
    train = _make_windows(
        records, train_indices, window_size, stride,
        motor_mean, motor_scale, ir_mean, ir_scale,
    )
    validation = _make_windows(
        records, validation_indices, window_size, stride,
        motor_mean, motor_scale, ir_mean, ir_scale,
    )
    test = _make_windows(
        records, test_indices, window_size, stride,
        motor_mean, motor_scale, ir_mean, ir_scale,
    )

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_dir / "windows.npz",
        motor_train=train[0],
        ir_train=train[1],
        motor_validation=validation[0],
        ir_validation=validation[1],
        motor_test=test[0],
        ir_test=test[1],
        motor_train_run_index=train[2],
        motor_validation_run_index=validation[2],
        motor_test_run_index=test[2],
        train_window_start_seconds=train[3],
        validation_window_start_seconds=validation[3],
        test_window_start_seconds=test[3],
    )

    metadata = {
        "data_origin": "Webots simulation telemetry; not physical hardware telemetry",
        "window_size_samples": window_size,
        "stride_samples": stride,
        "motor_features": list(MOTOR_FEATURES),
        "ir_features": list(IR_FEATURES),
        "scaling": {
            "fit_on": "training runs only",
            "motor_mean": motor_mean.tolist(),
            "motor_scale": motor_scale.tolist(),
            "ir_mean": ir_mean.tolist(),
            "ir_scale": ir_scale.tolist(),
        },
        "split_method": "explicit" if split is not None else "validation_fraction",
        "split_source": None if split_source is None else str(split_source),
        "train_runs": train[4],
        "validation_runs": validation[4],
        "test_runs": test[4],
        "run_quality": {
            record[0]: record[4]
            for record in records
        },
        "excluded_episodes": excluded,
        "test_split": "held-out healthy runs; see test_runs" if test_indices else None,
        "test_split_note": (
            "Test runs are healthy only; fault-injection runs are not included yet."
            if test_indices
            else "No test split: provide one with an explicit --split file."
        ),
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--runs", nargs="+", help="Input telemetry CSV files")
    source.add_argument(
        "--split",
        help="JSON file assigning run CSVs to train/validation/test "
        "(relative paths resolve from the repository root)",
    )
    parser.add_argument("--out", default="data/datasets/sim-baseline-v1")
    parser.add_argument("--window", type=int, default=20)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--validation-fraction", type=float, default=0.25)
    parser.add_argument("--minimum-duration", type=float, default=10.0)
    args = parser.parse_args()

    metadata = prepare_dataset(
        args.runs or [],
        args.out,
        window_size=args.window,
        stride=args.stride,
        validation_fraction=args.validation_fraction,
        minimum_duration=args.minimum_duration,
        split=load_split(args.split) if args.split else None,
        split_source=Path(args.split).as_posix() if args.split else None,
    )
    print(f"Prepared windows: {Path(args.out) / 'windows.npz'}")
    print(f"Metadata and run quality: {Path(args.out) / 'metadata.json'}")
    print(f"Training runs: {[run['run_id'] for run in metadata['train_runs']]}")
    print(f"Validation runs: {[run['run_id'] for run in metadata['validation_runs']]}")
    print(f"Test runs: {[run['run_id'] for run in metadata['test_runs']]}")
    print(f"Excluded short or duplicate episodes: {len(metadata['excluded_episodes'])}")
    if not metadata["validation_runs"]:
        print("Validation split unavailable: only one unique run remains.")


if __name__ == "__main__":
    main()
