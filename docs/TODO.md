# Project TODO

## Currently Doing

Build a repeatable, diverse Webots telemetry dataset and turn it into a
validated anomaly-detection model. Keep Webots as the simulation environment;
the ESP32-S3 is the external compute target, not a move to a physical AGV.

## 1. Collect and qualify healthy runs (done)

- Done: 64 healthy runs (4 routes x 16 speeds), recorded headlessly with
  `scripts/collect_route_runs.py` after the junction log-gap fix.
- Optional: exercise edge-002/edge-004 with a forced-edge route option, and add
  seeded start-pose variation if more than one run per route/speed is needed.

## 2. Prepare and validate the dataset (done)

- Done: explicit split support (`--split`) and `data/datasets/sim-baseline-v4`.
- Decide the final window length/features based on model experiments.

## 3. Create and evaluate anomaly models

- Done: dense and conv motor/IR autoencoders, PCA baseline, validation
  thresholds (99.5th percentile), 3-of-5 persistence, 0.5 s warm-up, and
  healthy-test false-alarm evaluation (`scripts/train_autoencoder.py`,
  `models/dense-v1`, `models/conv-v1`).
- Done: fault injection (9 types), 80 labelled fault runs, and detection
  evaluation (`scripts/evaluate_faults.py`, `reports/fault-eval-v1`).
- Done: severity sweep for right-motor power loss (5-50%) and right IR noise
  (std 25-200) in `reports/severity-sweep-v1`.
- Optional: per-sensor max scoring or a sample-difference feature for
  single-sensor noise, a longer window for stuck-sensor faults, and sweeps for
  other fault types and onset times.
- Choose the deployed architecture. conv-v1 currently has no pre-onset false
  alarms and detects as well as dense-v1.

## 4. Deploy inference to the ESP32-S3

- Select a model representation/runtime supported by ESP-IDF and verify
  operator coverage, RAM/flash use, and inference latency.
- Port preprocessing and the exact training scaler parameters; test ESP scores
  against Python outputs on the same recorded windows.
- Define how the ESP reports motor/IR anomaly scores and threshold decisions.

## 5. Connect Webots, Python, and the ESP

- Implement the Python bridge: send route actions once, then stream sensor,
  timing, and required motor/odometry features; receive wheel commands and
  anomaly results.
- Extend the serial protocol to carry the motor telemetry needed by the motor
  autoencoder; the current protocol sends only three IR readings and returns
  wheel targets.
- Keep one process responsible for COM3. Add framing, sequence checking,
  timeouts, safe-stop behavior, and reconnect/error handling.
- Convert graph routes to the ESP's relative junction-action sequence and
  verify the return leg is planned from destination back to dock.
- Run Webots with the ESP in the control loop. Check timing, command
  application, route state, anomaly scores, and safe stop on communication loss.

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
