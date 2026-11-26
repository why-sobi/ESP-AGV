# Project TODO

## Currently Doing

The ESP32-S3 runs the PD line-following loop and both anomaly models for
Webots over COM3. Round trips A-D complete, and the on-ESP scores agree with
Python. Next: decide how the robot responds to an alarm, and improve the weak
detections. Keep Webots as the simulation environment; the ESP32-S3
is the external compute target, not a move to a physical AGV.

## 1. Collect and qualify healthy runs (done)

- Done: 64 healthy runs (4 routes x 16 speeds), recorded headlessly with
  `scripts/telemetry/collect_route_runs.py` after the junction log-gap fix.
- Optional: exercise edge-002/edge-004 with a forced-edge route option, and add
  seeded start-pose variation if more than one run per route/speed is needed.

## 2. Prepare and validate the dataset (done)

- Done: explicit split support (`--split`) and `data/datasets/sim-baseline-v4`.
- Decide the final window length/features based on model experiments.

## 3. Create and evaluate anomaly models

- Done: dense and conv motor/IR autoencoders, PCA baseline, validation
  thresholds (99.5th percentile), 3-of-5 persistence, 0.5 s warm-up, and
  healthy-test false-alarm evaluation (`scripts/models/train_autoencoder.py`,
  `models/dense-v1`, `models/conv-v1`).
- Done: fault injection (9 types), 80 labelled fault runs, and detection
  evaluation (`scripts/faults/evaluate_faults.py`, `reports/fault-eval-v1`).
- Done: severity sweep for right-motor power loss (5-50%) and right IR noise
  (std 25-200) in `reports/severity-sweep-v1`.
- Optional: per-sensor max scoring or a sample-difference feature for
  single-sensor noise, a longer window for stuck-sensor faults, and sweeps for
  other fault types and onset times.
- Choose the deployed architecture. conv-v1 currently has no pre-onset false
  alarms and detects as well as dense-v1.

## 4. Deploy inference to the ESP32-S3

- Done: `conv-v1` alarmed on every healthy ESP run, so a healthy ESP dataset
  (`esp-baseline-v1`, 108 round trips) was recorded and `models/esp-conv-v1`
  trained, with manoeuvre gating and the no-validation-alarm threshold. Fault
  results are in `reports/esp-fault-eval-v1`.
- Done: the first 0.5 s after a U-turn is unmonitored (healthy test false
  alarms are now motor 2/18, IR 0/18).
- Improve the weak IR detections (noise std 100 or less, stuck, drift). Options:
  per-sensor max scoring, a sample-difference feature, or more healthy seeds.
- Decide how to report a fault when only the motor model fires on an IR
  fault (the PD loop couples them).

- Done: plain float32 C++ forward pass (`firmware/main/anomaly.cpp`, weights
  from `scripts/models/export_c.py`). It costs about 4.5 ms per frame at -O2,
  and the image is 0x336b0 bytes.
- Done: the scaler, state gating, warm-up, threshold and 3-of-5 rule on the
  ESP. Replay agreement holds within 4.5e-5, with identical alarms
  (`scripts/esp/replay_scores.py`).
- Done: scores and alarms in the motor frame; the controller logs them.
- Optional: INT8 quantisation as a comparison (float32 already fits easily).
- Decide what the robot does on an alarm (it currently keeps driving).

## 5. Connect Webots, Python, and the ESP

- Done: `epuck_route` in `esp32` mode is the bridge. It sends the route once
  as junction actions with odometry distances, streams the three IR readings
  and both encoder positions, and applies the returned wheel commands.
- Done: graph routes are converted to distance-tagged junction actions
  (`scripts/esp/route_actions.py`). The ESP gates junctions by odometry.
- Done: sequence checking, timeouts, safe stop on line loss, missed junction
  and destination, and recovery after a host stall (`LINK_LOST`).
- Done: routes A-D complete with the ESP in the loop (see `docs/STATUS.md`).
- Done: anomaly scores and alarm decisions in the motor frame; live runs
  in `data/esp_runs/live-v1` detect all 12 faults with no false alarm.
- Done: return leg. With `return_to_start`, the Supervisor plans destination
  to start and sends it with a leading U-turn; round trips A-D complete.
- Done: ESP-mode fault scenarios (120 runs). A fault that changes the driven path can push
  a junction out of its +/-0.10 m window and stop the run with
  `JUNCTION_MISSED`; decide whether that is the desired safe behavior.
- Done: the ESP takes a route speed (`V@`, mission `esp_speed_rad_s`), so ESP runs can cover
  the same speed grid as the healthy dataset.
- Reconnect handling if COM3 disappears mid-run (currently the run fails).

## Navigation follow-up

- Review generated graph semantics and stable node/edge names.
- Exercise more routes, especially the circular section and multi-branch
  junctions.
- Re-test map conversion, visualization, hand-editing, and Webots image/scale
  alignment on a second track.

## Completion criteria

- Dataset manifests identify independent runs and support healthy-only train,
  healthy validation, healthy test, and labeled fault test sets.
- Python model metrics and threshold results are reproducible from saved
  configuration and data manifests.
- ESP inference agrees with Python within defined numerical tolerances and
  satisfies memory/timing limits.
- Webots is driven by ESP-produced wheel commands through the Python serial
  bridge, and anomaly detections/telemetry are logged with route context.
- Communication loss, malformed frames, line loss, and destination completion
  result in documented, tested safe behavior.
