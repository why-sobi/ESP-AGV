# Project TODO

## Currently Doing

The ESP32-S3 now runs the PD line-following loop for Webots over COM3 (routes
A-D complete). Next: check the anomaly models on ESP-controlled runs, then put
inference on the ESP. Keep Webots as the simulation environment; the ESP32-S3
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

- First, score ESP-controlled healthy runs with `conv-v1`. The models were
  trained on Supervisor-controlled runs, and the ESP's PD loop commands
  different wheel targets. If they raise false alarms, record a healthy
  dataset under ESP control and retrain before exporting.

- Select a model representation/runtime supported by ESP-IDF and verify
  operator coverage, RAM/flash use, and inference latency.
- Port preprocessing and the exact training scaler parameters; test ESP scores
  against Python outputs on the same recorded windows.
- Define how the ESP reports motor/IR anomaly scores and threshold decisions.

## 5. Connect Webots, Python, and the ESP

- Done: `epuck_route` in `esp32` mode is the bridge. It sends the route once
  as junction actions with odometry distances, streams the three IR readings
  and both encoder positions, and applies the returned wheel commands.
- Done: graph routes are converted to distance-tagged junction actions
  (`scripts/esp/route_actions.py`). The ESP gates junctions by odometry.
- Done: sequence checking, timeouts, safe stop on line loss, missed junction
  and destination, and recovery after a host stall (`LINK_LOST`).
- Done: routes A-D complete with the ESP in the loop (see `docs/STATUS.md`).
- Add anomaly scores and alarm decisions to the motor frame once inference
  runs on the ESP.
- Done: return leg. With `return_to_start`, the Supervisor plans destination
  to start and sends it with a leading U-turn; round trips A-D complete.
- Run ESP-mode fault scenarios. A fault that changes the driven path can push
  a junction out of its +/-0.10 m window and stop the run with
  `JUNCTION_MISSED`; decide whether that is the desired safe behavior.
- Decide whether the ESP should take the mission speed, so ESP runs can cover
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
