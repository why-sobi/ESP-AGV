# Project Status

## Agreed project goal

Keep the AGV in the Webots simulation environment. Webots provides simulated
sensors and odometry; the ESP32-S3 will run the control and anomaly-detection
computation; Webots applies the returned wheel commands to its simulated robot.
The eventual loop is:

```text
Webots sensors/encoders -> Python bridge over USB serial (COM3) -> ESP32-S3
ESP32-S3 wheel commands/anomaly scores -> Python bridge -> Webots motors/logging
```

The Webots simulation remains the environment throughout this plan.

## Verified

- The graph planner, path-image converter, map visualizer, and Webots route
  controller are implemented. The user visually checked the configured
  simulation route.
- The configured two-edge route has completed headless Webots runs at 2.5 and
  3.0 rad/s. These runs are saved in `data/route_runs`.
- The ESP32-S3 N16R8 project builds and has been flashed over COM3 with an
  ESP32-S3/16 MB-flash configuration.
- `scripts/esp_serial_smoke_test.py` successfully sent route and simulated
  sensor messages over COM3 and verified PD wheel outputs, junction-action
  handling, destination stop, and invalid-route rejection.
- `epuck_data` now writes a separate timestamped CSV per session under
  `data/sensor_runs` instead of appending to a shared filename.
- `scripts/prepare_telemetry.py` derives wheel velocities, creates 20-step
  motor/IR windows, splits by run before windowing, fits scalers on training
  data only, and rejects exact duplicate runs.
- `epuck_route` changes:
  - It spawns facing a point one lookahead along the first edge. The first
    skeleton pixel step at node-001 had pointed the sensors off the line.
  - It stops logging and prints `RUN FAILED` after `max_line_loss_s` of line
    loss.
  - It no longer skips a log row when it advances to the next edge (fixes a
    64 ms gap at each junction).
  - It accepts the `AGV_MISSION_PATH`, `AGV_ROUTE_PATH` and `AGV_LOG_DIR`
    overrides.
- `scripts/collect_route_runs.py` records runs headlessly (about 2 s each),
  checks them, and updates `data/route_runs/manifest.csv`.
- 64 healthy runs exist with the fixed controller: 4 two-edge routes (A
  node-003 to node-001, B node-001 to node-003, C node-002 to node-004, D
  node-004 to node-002) x 16 speeds from 2.0 to 3.5 rad/s in 0.1 steps. All
  reached the destination with no irregular sampling, no line loss after
  0.5 s, and max centerline error at most 0.025 m. The 16 earlier manual runs
  are marked `superseded`.
- `prepare_telemetry.py --split` takes an explicit train/validation/test split
  file. `data/datasets/sim-baseline-v4` (split `route-runs-v3.json`) has 36
  train, 14 validation and 14 test runs (15474/5933/5929 windows). Every held-out
  speed has training runs at +/-0.1 rad/s on the same route.
- `scripts/train_autoencoder.py` trains dense or conv motor/IR autoencoders,
  sets thresholds from validation, and evaluates false alarms on the healthy
  test runs against a PCA baseline. `models/dense-v1` and `models/conv-v1` are
  trained:
  - Motor conv: 0/14 test runs alarm (0.31% of windows over threshold).
  - IR dense: 2/14 runs alarm (0.59% of windows).
  - IR PCA baseline: 5/14 runs alarm (0.68% of windows).
- Fault injection: `epuck_route` supports 9 fault types via a mission `fault`
  block and labels rows with `fault_active`. Healthy runs are byte-identical to
  before. 80 fault runs exist (10 faults x 8 held-out test route/speeds) in
  `data/fault_runs`.
- Fault detection (`reports/fault-eval-v1`, conv-v1):
  - All motor and encoder faults are detected in 8/8 runs, with median delay
    0.10-0.45 s.
  - IR dead and IR drift are detected in 8/8 runs.
  - IR stuck-centre (1/8) and IR noise std 50 (2/8) are mostly missed.
  - No pre-onset false alarms.
  - The PCA baseline misses most IR faults.
- The Python script test suite passes (28 tests at last verification).

## Not yet implemented

- The Python Webots-to-ESP bridge. Webots still runs its own controller; the
  smoke test is a separate host script and does not drive Webots.
- A graph-route-to-relative-junction-action converter for the ESP.
- ESP model inference (weights export to C) and Python-vs-ESP score agreement.
- End-to-end testing where the ESP computes the commands that Webots applies.

## Current data/model status

`sim-baseline-v4` is the healthy dataset. Known limitations:

- Webots is deterministic, so each route and speed pair is one run. The first
  attempt held out whole speed/route combinations at 0.5 rad/s spacing, and the
  IR model then raised false alarms on corners it had not seen at that speed
  (2.6% of windows). The 0.1 rad/s grid fixed that.
- Shortest-path routing never uses the parallel edges edge-002 and edge-004.
- The motor PCA baseline is nearly as good as the autoencoders. The IR
  autoencoders clearly beat it.
- A stuck sensor frozen at a plausible value (the centre sensor on the line) is
  hard to detect from a 0.64 s window. Mild IR noise (about 0.2 of the scaler
  std) is mostly missed. Severity sweeps have not been run yet.
- Each fault severity was tested at one level and one onset time (5 s).

Train only on healthy data; the held-out healthy test runs are for
false-positive measurement.

The anomaly-detection target remains simulated telemetry. ESP deployment means
running the trained inference computation on the ESP32-S3 while Webots remains
the source of sensor data and the destination for wheel commands.

## References

- `docs/USAGE.md`: commands, project files, data collection, preparation, ESP
  checks, and what is or is not wired together.
- `docs/TODO.md`: ordered next steps and completion criteria.
- `docs/DECISIONS.md`: navigation, simulation-first architecture, and lessons.
