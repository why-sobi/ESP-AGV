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
- `scripts/esp/esp_serial_smoke_test.py` sends routes and simulated sensor and
  encoder frames over COM3. It verifies PD wheel outputs, odometry,
  distance-gated junction handling, the destination and missed-junction stops,
  and malformed-route rejection.
- `epuck_data` now writes a separate timestamped CSV per session under
  `data/sensor_runs` instead of appending to a shared filename.
- `scripts/telemetry/prepare_telemetry.py` derives wheel velocities, creates 20-step
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
- `scripts/telemetry/collect_route_runs.py` records runs headlessly (about 2 s each),
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
- `scripts/models/train_autoencoder.py` trains dense or conv motor/IR autoencoders,
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
- Severity sweep (`reports/severity-sweep-v1`, 104 runs):
  - Right-motor power loss of 20% or more is detected in 7-8/8 runs; 15% in
    4/8; 10% or less essentially not. The autoencoders beat PCA between 15% and
    30% loss.
  - Right IR noise is reliable only at std 200 (8/8); std 100 is detected in
    4/8.
- The Python script test suite passes (36 tests at last verification).
- **ESP closed-loop control (2026-11-14).** With `control_mode: esp32`,
  `epuck_route` acts as the Webots-to-ESP bridge over COM3. All four two-edge
  routes (A-D) complete headlessly with the ESP computing every wheel command:
  - The ESP takes each turn 2.5-3.6 cm before the junction node.
  - It stops 1.0-1.4 cm from the destination node.
  - Maximum path error is 2.4-2.5 cm, the same as the Supervisor controller.
  - The updated smoke test passes on the flashed firmware.
  - Supervisor-mode runs are unchanged: a new route A 2.5 rad/s run matches
    the committed healthy run byte-for-byte in the original 15 columns.
  - **Round trips** (`return_to_start: true`) also complete on all four
    routes. After a 1 s unload, the ESP U-turns in place (encoder-measured,
    at least 160 degrees then back on the line) and follows the return route.
    It stops 0.2-1.3 cm from the start node, with return-leg path error at
    most 2.4 cm.
- How ESP routing works:
  - `scripts/esp/route_actions.py` converts the planned route into junction
    actions with expected odometry distances, e.g. `R@0.265,D@0.779`.
  - The ESP measures distance from the wheel encoders and only takes a
    junction pattern within +/-0.10 m of its distance.
  - It stops at the destination distance.
  - The Supervisor checks that the stop is within 0.05 m of the node.
- Firmware fixes:
  - **Line loss:** a bounded line search replaces the immediate `LINE_LOST`
    stop that failed the first ESP run at 0.16 s.
  - **Serial delay:** the UART read no longer waits 50 ms for a full buffer,
    so the round trip fell from 50 ms to about 8 ms.
  - **Link timeout:** it starts at the first sensor frame of a route, and the
    ESP resumes after `LINK_LOST` once frames return. Webots pauses in
    wall-clock time at startup, which used to latch `LINK_LOST`.

## Not yet implemented

- ESP model inference (weights export to C), anomaly scores in the serial
  protocol, and Python-vs-ESP score agreement.
- A check of the Supervisor-trained models on ESP-controlled runs, and a
  healthy dataset recorded under ESP control if they raise false alarms.
- ESP-mode runs through the parallel edges (edge-002, edge-004) and with
  injected faults. Round trips are ESP-only; the Supervisor controller has
  no U-turn. The ESP ignores the mission's
  `wheel_speed_rad_s` (its PD loop runs at 2.5-3.5 rad/s), so ESP runs do not
  yet cover a speed grid.

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
  hard to detect from a 0.64 s window.
- IR noise on a single sensor is diluted by the window-mean score. It is
  reliably detected only at about 0.9 of the scaler std.
- Faults use one onset time (5 s). Only motor gain and IR noise have been swept
  over severity.

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
