# Project TODO

## Currently Doing

Build a repeatable, diverse Webots telemetry dataset and turn it into a
validated anomaly-detection model. Keep Webots as the simulation environment;
the ESP32-S3 is the external compute target, not a move to a physical AGV.

## 1. Collect and qualify healthy runs (done for the first dataset)

- Done: 16 healthy runs (4 routes x 4 speeds), qualified and logged in
  `data/route_runs/manifest.csv`.
- Follow-up: stop `epuck_route` from skipping a log row when it advances to the
  next edge (one 64 ms gap per run). Optionally exercise edge-002/edge-004 by
  adding a forced-edge route option, and add seeded start-pose variation if
  more than one run per route/speed is needed.

## 2. Prepare and validate the dataset (done for the first dataset)

- Done: explicit split support (`--split`) and
  `data/datasets/sim-baseline-v2` with healthy train/validation/test runs.
- Decide the final window length/features based on model experiments.

## 3. Create and evaluate anomaly models

- Implement the motor and ground-IR autoencoders and their training/evaluation
  scripts.
- Train only on healthy training runs; fit scaling only on those runs.
- Compare temporal model choices and measure validation reconstruction error.
- Set per-model thresholds using held-out healthy validation data.
- Add controlled Webots motor and sensor faults. Measure detection delay,
  false-positive rate on healthy test runs, and subsystem cross-coupling.

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
