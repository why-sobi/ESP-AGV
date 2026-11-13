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
- 16 distinct healthy `epuck_route` runs exist: 4 two-edge routes (node-003 to
  node-001, node-001 to node-003, node-002 to node-004, node-004 to node-002)
  at 2.0, 2.5, 3.0 and 3.5 rad/s. All reached the destination, with max
  centerline error at most 0.024 m and line-loss samples only in the first
  0.5 s. `data/route_runs/manifest.csv` records route, speed, and quality for
  each file.
- `epuck_route` now spawns facing a point one lookahead along the first edge
  (the first skeleton pixel step at node-001 pointed the sensors off the line).
  It also stops logging and prints `RUN FAILED` after `max_line_loss_s` of line
  loss.
- `prepare_telemetry.py --split` takes an explicit train/validation/test split
  file. `data/datasets/sim-baseline-v2` (split
  `data/datasets/splits/route-runs-v1.json`) has 3483 train windows from 8 runs
  and 1732 validation and 1741 test windows from 4 runs each. Validation and
  test each cover every route and every speed. `sim-baseline-v1` is the earlier
  two-run pilot.
- The Python script test suite passes (21 tests at last verification).

## Not yet implemented

- The Python Webots-to-ESP bridge. Webots still runs its own controller; the
  smoke test is a separate host script and does not drive Webots.
- A graph-route-to-relative-junction-action converter for the ESP.
- Anomaly-detection model definitions, training/evaluation scripts, fault
  injection harness, threshold selection, and ESP model inference.
- End-to-end testing where the ESP computes the commands that Webots applies.

## Current data/model status

`sim-baseline-v2` is the healthy dataset for the first model experiments. Known
limitations: Webots is deterministic, so each route and speed pair is one run.
Shortest-path routing never uses the parallel edges edge-002 and edge-004.
Single-edge routes are under the 10 s minimum at these speeds. Each run has one
64 ms logging gap where the controller skips a row at the intermediate junction.
There are no controlled-fault runs yet. Train only on healthy data; the held-out
healthy test runs are for false-positive measurement.

The anomaly-detection target remains simulated telemetry. ESP deployment means
running the trained inference computation on the ESP32-S3 while Webots remains
the source of sensor data and the destination for wheel commands.

## References

- `docs/USAGE.md`: commands, project files, data collection, preparation, ESP
  checks, and what is or is not wired together.
- `docs/TODO.md`: ordered next steps and completion criteria.
- `docs/DECISIONS.md`: navigation, simulation-first architecture, and lessons.
