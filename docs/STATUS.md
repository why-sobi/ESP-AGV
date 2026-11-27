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

- **ESP dataset and models (2026-11-26).**
  - **Route speed:** the route command takes an optional `V@<rad/s>`, which
    `epuck_route` sends from the mission's `esp_speed_rad_s`. The smoke test
    covers it.
  - **Start-pose jitter:** seeded `spawn_jitter` makes repeated runs differ.
  - **Healthy data:** 108 ESP round trips, 4 routes x 9 speeds (2.0-4.0
    rad/s) x 3 seeds, all complete.
  - **Dataset:** `esp-baseline-v1`, split `esp-runs-v1.json` (72/18/18
    runs).
  - **Old models fail on ESP data:** the Supervisor-trained `conv-v1`
    alarmed on all 18 healthy ESP test runs.
  - **New models:** `models/esp-conv-v1` (motor 5208 and IR 4500
    parameters). Windows overlapping `AT_DEST`/`UNLOADING`/`U_TURN` are not
    monitored, and the thresholds are the lowest with no alarm on any healthy
    validation run.
  - **Healthy test false alarms:** motor 2/18 runs, IR 0/18. The first 0.5 s
    after a U-turn is unmonitored too.
  - **Old models on the same runs:** `conv-v1` and `dense-v1` alarm on 18/18
    healthy runs, with 13-25% of windows over threshold
    (`reports/esp-model-comparison-v1`).
  - **Fault runs (`reports/esp-fault-eval-v1`, 120 runs):** no pre-onset
    alarms. Stall, intermittent, 50% power loss, stuck/noisy encoder, IR dead
    and IR noise std 200 are detected 8/8. IR noise also trips the motor
    model, because the PD loop reacts to it.
  - **Weak spots:** 20% power loss (3/8), IR noise of std 100 or less (0-4/8
    by the IR model), IR stuck (1/8) and IR drift (4-6/8).
  - **USB serial drops:** `SerialException` runs are now retried. Moving the
    laptop charger caused one drop.

- **Anomaly detection on the ESP (2026-11-26).**
  - **Export:** `scripts/models/export_c.py` writes
    `firmware/main/anomaly_model.h`. `firmware/main/anomaly.cpp` runs both
    float32 models on every frame.
  - **Protocol:** the motor frame carries the motor/IR scores and the 3-of-5
    alarms, and the controller logs them.
  - **Cost:** about 2.3 + 2.0 ms per frame at `-O2`.
  - **Replay agreement** (`scripts/esp/replay_scores.py`, 33 runs, 65044
    windows): scores within 4.5e-5 and identical alarm decisions.
  - **Live** (`data/esp_runs/live-v1`, 16 round trips): no alarms on the 4
    healthy runs, and all 12 fault runs were detected on the ESP by the right
    model (0.1-5.2 s), with no pre-onset alarm.
  - **Supervisor mode** still reproduces the committed healthy run
    byte-for-byte in the first 15 columns.

- **Fault stop and notifications (2026-11-26).**
  - Route option `F@<s>`: once an alarm has lasted that long the ESP pulls
    over (90 degrees, 8 cm) and stops (`FAULT_STOP`), giving up after 6 s.
  - The ESP posts a plain-language notification to ntfy.sh over WiFi (core 0,
    never blocking control), naming the problem and the place (`E@` labels).
    WiFi settings live in a 12 KB `wificfg` partition.
  - Live (`fault-stop-v1`, `-v2`, 24 runs): healthy 4/4 `AT_DEST`; IR dead
    and IR noise 200 `FAULT_STOP` 4/4; motor faults stop with `FAULT_STOP`
    or `LINE_LOST`; every stop notified exactly once.

- **Motor GRU and stuck-sensor rule (2026-11-26/27).** See `REPORT.md`
  sections 9-12 and `reports/esp-gru-eval-v1`.
  - Offline (120 fault runs): detection by the fault's own detector 82 ->
    94/120, by either detector 98 -> 110/120; healthy validation 0/18, test
    2/18 (unchanged, same runs); no pre-onset alarm. Weak motor 20%/30%
    3/8, 6/8 -> 8/8, 8/8; IR stuck 1/8 -> 6/8 (6 observable).
  - The motor GRU (32 hidden) predicts both wheel speeds from the wheel
    targets; threshold 2x the no-validation-alarm value (0.0246).
  - The stuck rule flags a sensor flat for 1 s while the robot drives (mean
    wheel speed > 1 rad/s). The first version (another sensor moving) missed
    a sensor stuck at the on-the-line value live.
  - All four detectors run on esp-dsp (ESP32-S3 SIMD) with weights copied to
    internal RAM: 0.31 + 0.36 + 0.38 ms per frame (was 2.3 + 2.0 ms for two).
  - Replay (`reports/esp-replay-v2`, 10 runs): conv within 5e-5, GRU within
    1.6e-5, alarms identical. Live (`fault-stop-v3`, `-v4`): healthy 8/8
    clean; weak motor 8/8 `FAULT_STOP`; stuck IR 3/4 `FAULT_STOP` (the 4th
    starts in a U-turn and stops with `LINE_LOST`).

- **Pull-over side from the map (2026-11-27).** The Supervisor checks each
  route step for other lines and walls on both sides and sends `P@` (one
  side per step). On the small loop half the legs would have pulled over onto
  the other loop line; live (`pull-over-v1`) the ESP pulled over left, 4-5 cm
  outside the loop.

- **Architecture evidence (2026-11-27).** See `REPORT.md` sections 14-16.
  - 967 architectures timed on the ESP (`reports/esp-latency-v1`).
  - INT8/INT16 with ESP-DL (`reports/esp-int8-v1`): ~2.5x slower than the
    float32 esp-dsp code at these model sizes; INT8 breaks the motor conv
    model; float32 kept.
  - Optuna search (`reports/arch-search-v1`): smaller motor conv (100 us,
    28/28 held-out vs 25/28) and GRU (127 us, 28/28 vs 27/28) beat the
    deployed ones; the smaller IR conv detects more (22/32 vs 17/32) but adds
    2/18 healthy test alarms. Motor conv and GRU deployed 2026-11-27
    (`esp-conv-v2`, `esp-gru-v2`): 0.81 ms per frame (was 1.18 ms), replay
    alarms identical, live fault-stop-v5 as before.

- The Python test suite passes (47 tests); the ESP smoke test passes (48 checks).
- Second and third tracks (track2, track3) drive and detect with the same
  firmware and models: healthy 8/8 each, faults 20/20 and 16/16.

## Not yet implemented

- Detection of IR drift (4-6/8) and IR noise of std 100 or less (0-3/8 by
  the IR model).
- The smaller IR conv from the search (a trade-off; user's choice).
- A real-unplug test of the COM3 reconnect (tested with a fake port).
- Healthy training runs over edge-002 (the motor conv model raises sub-0.3 s
  alarms at its turns; parallel edges themselves are driven with
  `avoid_edges`).
- Faults that start in a U-turn and lose the line before any monitored window
  are only caught as `LINE_LOST`.

## Current data/model status

`sim-baseline-v4` is the healthy dataset. Known limitations:

- Webots is deterministic, so each route and speed pair is one run. The first
  attempt held out whole speed/route combinations at 0.5 rad/s spacing, and the
  IR model then raised false alarms on corners it had not seen at that speed
  (2.6% of windows). The 0.1 rad/s grid fixed that.
- Shortest-path routing never uses the parallel edges edge-002 and edge-004;
  `avoid_edges` drives them, but no dataset includes them.
- The motor PCA baseline is nearly as good as the autoencoders. The IR
  autoencoders clearly beat it.
- A stuck sensor frozen at a plausible value (the centre sensor on the line) is
  hard to detect from a 0.64 s window; the stuck-sensor rule now covers it.
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

- `REPORT.md`: the whole project, every result and number in one place.
- `docs/USAGE.md`: commands, project files, data collection, preparation, ESP
  checks, and what is or is not wired together.
- `docs/TODO.md`: ordered next steps and completion criteria.
- `docs/DECISIONS.md`: navigation, simulation-first architecture, and lessons.
