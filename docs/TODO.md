# Project TODO

## Currently Doing

The ESP32-S3 drives Webots round trips, runs four anomaly detectors (motor and
IR conv autoencoders, motor GRU, stuck-sensor rule) on esp-dsp, and on a
persistent alarm pulls over to the map-chosen side, stops and notifies the
phone. Everything is summarized in `REPORT.md`. Open work, in priority order:

1. Detect IR drift (4-6/8) and IR noise of std 100 or less (0-3/8 by the IR
   model): a level check or a cross-sensor model.
2. Done 2026-11-27: motor conv and GRU search winners deployed
   (`esp-conv-v2`, `esp-gru-v2`; replay, offline and live fault-stop-v5).
   Open: the smaller IR conv is a trade-off (22/32 vs 17/32, but 2/18
   healthy test alarms); the user decides.
3. Done 2026-11-27: COM3 reconnect in `epuck_route` (fake-port test; a real
   unplug test is still to do).
4. Done 2026-11-27: ESP runs over the parallel edges (`avoid_edges`,
   per-edge odometry scales; `data/esp_runs/parallel-edges-v2`). Open:
   healthy training runs over edge-002 (sub-0.3 s motor conv alarms there).
5. Done 2026-11-27: track2 (second, unseen track) 8/8 healthy and 20/20
   faults after the navigation fixes (REPORT.md section 11).
6. Done 2026-11-27: track3 (45° bends, Y junction, hairpin, tight curve) 8/8
   healthy, 16/16 faults. Open: hairpin margin at 3.5 rad/s (0.74 of 0.75 s
   line search).

Keep Webots as the simulation environment; the ESP32-S3 is the external
compute target, not a move to a physical AGV.

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
- Done: architectures chosen (conv autoencoders, later paired with a motor
  GRU) and backed by an architecture search on measured ESP latency.
- Optional: sweeps for other fault types and onset times.

## 4. Deploy inference to the ESP32-S3

- Done: `conv-v1` alarmed on every healthy ESP run, so a healthy ESP dataset
  (`esp-baseline-v1`, 108 round trips) was recorded and `models/esp-conv-v1`
  trained, with manoeuvre gating and the no-validation-alarm threshold. Fault
  results are in `reports/esp-fault-eval-v1`.
- Done: the first 0.5 s after a U-turn is unmonitored (healthy test false
  alarms are now motor 2/18, IR 0/18).
- Done: weak motor faults and IR stuck (motor GRU and stuck-sensor rule,
  `reports/esp-gru-eval-v1`): detection 82 -> 94/120 with no extra false
  alarm. Per-sensor max scoring and a sample-difference score were tried and
  did not help.
- Open: IR noise of std 100 or less and IR drift.
- Decide how to report a fault when only the motor model fires on an IR
  fault (the PD loop couples them).

- Done: plain float32 C++ forward pass (`firmware/main/anomaly.cpp`, weights
  from `scripts/models/export_c.py`). It costs about 4.5 ms per frame at -O2,
  and the image is 0x336b0 bytes.
- Done: the scaler, state gating, warm-up, threshold and 3-of-5 rule on the
  ESP. Replay agreement holds within 4.5e-5, with identical alarms
  (`scripts/esp/replay_scores.py`).
- Done: scores and alarms in the motor frame; the controller logs them.
- Done: all detectors on esp-dsp with weights in RAM, ~1.05 ms per frame
  (`firmware/README.md`); replay within 5e-5 with identical alarms
  (`reports/esp-replay-v2`).
- Done: INT8/INT16 comparison with ESP-DL (`reports/esp-int8-v1`): slower at
  these sizes and INT8 breaks the motor conv model; float32 kept.
- Done: on-ESP latency table (`reports/esp-latency-v1`) and Optuna search
  (`reports/arch-search-v1`); motor conv and GRU winners deployed.
- Done: on a persistent alarm the robot pulls over (side from the map), stops
  and sends a phone notification.

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
- Done: pull-over side per route step from the map (`P@`).
- Open: reconnect handling if COM3 disappears mid-run (currently the run
  fails; the ESP stops on `LINK_LOST`).

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

Status: all met except reconnecting after a COM3 drop (the ESP stops safely,
but the host run fails instead of resuming).
