# ESP-AGV: Project Report

A simulated AGV (Webots e-puck) follows a line track between named junctions.
An ESP32-S3 on USB serial is the robot's onboard computer: it runs the line
following, the junction and destination logic, four anomaly detectors, and the
response to a fault (pull over, stop, phone notification). Webots stays the
environment throughout; there is no physical robot. Faults are injected in
Webots, so the ESP sees them only through its inputs, as it would on hardware.

This report covers how the project got to its current state, what each
change was for, and every result measured along the way. Commands are in
`docs/USAGE.md`; design decisions and their reasons in `docs/DECISIONS.md`.

## Contents

1. [System overview](#1-system-overview)
2. [Timeline](#2-timeline)
3. [Navigation and control](#3-navigation-and-control)
4. [Data](#4-data)
5. [Anomaly detection: first models (Supervisor control)](#5-anomaly-detection-first-models-supervisor-control)
6. [Anomaly detection under ESP control](#6-anomaly-detection-under-esp-control)
7. [Inference on the ESP](#7-inference-on-the-esp)
8. [Fault response: stop and notify](#8-fault-response-stop-and-notify)
9. [Long-memory detectors: the GRU experiment](#9-long-memory-detectors-the-gru-experiment)
10. [esp-dsp deployment](#10-esp-dsp-deployment)
11. [Live fault-stop checks](#11-live-fault-stop-checks)
12. [Stuck-sensor rule fix](#12-stuck-sensor-rule-fix)
13. [Map-based pull-over side](#13-map-based-pull-over-side)
14. [On-device latency table](#14-on-device-latency-table)
15. [INT8 / INT16 benchmark](#15-int8--int16-benchmark)
16. [Architecture search (Optuna)](#16-architecture-search-optuna)
17. [Current performance summary](#17-current-performance-summary)
18. [Known limitations and open work](#18-known-limitations-and-open-work)
19. [Where everything is](#19-where-everything-is)

---

## 1. System overview

```text
Webots (e-puck, track, fault injection)
  │  3 ground-IR readings + 2 wheel-encoder positions, every 32 ms
  ▼
epuck_route controller (Python bridge, USB serial COM3, 115200 baud)
  │  one route command per leg: junction actions with odometry distances,
  │  speed (V@), fault-stop hold (F@), place labels (E@), pull-over sides (P@)
  ▼
ESP32-S3 (ESP-IDF 6.0.1, -O2, 160 MHz)
  ├─ PD line follower, odometry-gated junctions, destination, U-turn
  ├─ anomaly detectors (esp-dsp, float32): motor conv AE, motor GRU,
  │                                        IR conv AE, stuck-sensor rule
  ├─ fault stop: pull over to the map-chosen side, stop
  └─ WiFi notification (ntfy.sh) on a separate core
  │  wheel targets, state, scores, alarms
  ▼
Webots applies the wheel targets; every frame is logged to CSV
```

- **Route planning** (`scripts/maps/route_planner.py`): Dijkstra on a named
  graph (4 junctions, 6 edges) extracted from the track image.
- **Control** (`firmware/main/main.cpp`): the ESP steers from the IR readings
  and identifies junctions by wheel odometry (±0.10 m window around the
  expected distance), because the sensor pattern alone is not unique on this
  track.
- **Detection** (`firmware/main/anomaly.cpp`): four detectors, each scoring a
  window of recent samples, alarming when 3 of the last 5 monitored windows
  exceed its threshold. The motor alarm is either motor detector's, the IR
  alarm either IR detector's.
- **Response**: once an alarm has lasted 2 s (`F@2`), the ESP turns ~90° to the
  side chosen from the map, drives 8 cm off the line, stops (`FAULT_STOP`) and
  sends a plain-language phone notification naming the problem and the place.

## 2. Timeline

| Stage | What was added | Key result |
|---|---|---|
| Map and planner | Image-to-graph converter, visualizer, Dijkstra planner, Webots route controller | Two-edge route completes, max centerline error 2.28 cm |
| Healthy data v1-v4 | Headless collector, run validation, run-level splits | 64 healthy runs (4 routes × 16 speeds), `sim-baseline-v4` |
| First models | Dense and conv motor/IR autoencoders, PCA baseline, fault injection (9 types), severity sweep | All motor faults 8/8 on Supervisor data; IR stuck 1/8, light IR noise 2/8 |
| ESP control | Serial bridge, distance-tagged routes, ESP PD loop, round trips with U-turn | Routes A-D complete; stops 1.0-1.4 cm from the node |
| ESP models | 108 healthy ESP round trips, `esp-conv-v1`, manoeuvre gating, no-validation-alarm thresholds | Healthy test false alarms motor 2/18, IR 0/18 |
| Inference on ESP | float32 C++ forward pass, replay agreement, live runs | Scores within 4.5e-5 of Python; live 12/12 faults detected |
| Fault response | Fault stop + pull-over, WiFi notifications (ntfy), WiFi config partition | 24 live runs, every stop notified exactly once |
| GRU experiment | Motor GRU plant model, stuck-IR rule and baselines, offline comparison | Detection 82 → 94 of 120 (own model), no extra false alarms |
| esp-dsp deployment | All detectors on Espressif's SIMD library, weights in RAM | 4.4 ms → 1.05 ms per frame for three models instead of two |
| Fixes from live runs | Stuck rule needs driving (not another sensor moving); pull-over side from the map | Stuck IR stops 2/4 → 3/4 live (4th unobservable); no pull-over onto the loop's other line |
| Evidence for architectures | Latency of 967 architectures measured on the ESP; INT8/INT16 benchmark; Optuna search | Float32 esp-dsp 2.5× faster than ESP-DL INT8; smaller models found that are 2-3× faster and detect as well or better |
| Search winners deployed | Per-model conv sizes in export and firmware; `esp-conv-v2` (motor) + `esp-gru-v2`; COM3 reconnect | 1.18 → 0.81 ms per frame; same 94/110 of 120 offline; live 15/16 as expected (fault-stop-v5) |
| Parallel edges, other tracks | `avoid_edges`, per-edge odometry scales; track2 and track3 generated by `make_track.py`; navigation fixes (stale junction pattern, blind arrival, `l`/`r` corners, `<`/`>` bear turns, spins unmonitored) | Base 16/16, track2 8/8 + faults 20/20, track3 8/8 + faults 16/16 |

## 3. Navigation and control

### Map

- `maps/paths/base track.json`: 4 junctions (node-001 … node-004) and 6 edges
  (edge-001 … edge-006), including two parallel pairs (edge-002/edge-003 and
  the small loop edge-004/edge-005).
- The 2400×1800 px image is stretched onto a 1 m × 1 m arena (0.417 mm/px
  horizontally, 0.556 mm/px vertically), so horizontal lines are about a third
  wider than vertical ones. On wide stretches all three sensors see dark,
  which is why junctions cannot be identified from the sensor pattern alone
  (the first ESP attempt reported the destination 0.24 m early).

### Routes used throughout

| Route | One way | Speeds (healthy ESP data) |
|---|---|---|
| A | node-003 → node-001 | 2.0-4.0 rad/s, 0.25 steps |
| B | node-001 → node-003 | same |
| C | node-002 → node-004 | same |
| D | node-004 → node-002 | same |

Round trips run the return leg after a 1 s unload, starting with a U-turn.

### ESP control results

- **Junctions:** taken 2.5-3.6 cm before the node; the all-dark pattern is
  only accepted within ±0.10 m of the expected odometry distance. Wheel
  odometry reads less than the map length (the robot cuts corners): ~0.973
  on the long edges and ~0.836 on the curved loop. A single 0.92 fits every
  standard route (one long edge plus one loop edge) but stops 8-10 cm short
  on two long edges, so distances now use these per-edge scales (section 11).
- **Destination:** stops 1.0-1.4 cm from the node (one way), 0.2-1.3 cm on
  return legs; maximum path error 2.4-2.5 cm (same as the Supervisor
  controller).
- **U-turn:** spins left in place, encoder-measured, hands over to the PD loop
  after ≥160° with the centre sensor on the line; gives up after 270°.
- **Link:** serial round trip ~8 ms after removing a 50 ms UART wait; link
  timeout 250 ms, treated as a pause (Webots stops in wall-clock time).
- **Line loss:** bounded 750 ms line search before `LINE_LOST`.

## 4. Data

| Dataset | Contents | Use |
|---|---|---|
| `data/route_runs` (`sim-baseline-v4`) | 64 healthy Supervisor runs, 4 routes × 16 speeds (2.0-3.5 rad/s) | First models |
| `data/fault_runs` | 80 fault runs (10 faults × 8) + 104 severity-sweep runs | First fault evaluation |
| `data/esp_runs/healthy` (`esp-baseline-v1`) | 108 healthy ESP round trips: 4 routes × 9 speeds × 3 start-pose seeds | ESP models; split 72 train / 18 validation / 18 test (`esp-runs-v1.json`) |
| `data/esp_runs/faults` | 120 labelled ESP fault runs: 15 faults × 8 (4 routes × 2 speeds); 2 serial-error runs repeated | All ESP detector evaluation |
| `data/esp_runs/live-v1` | 16 live round trips (4 healthy + 12 faults) | First live check |
| `data/esp_runs/fault-stop-v1`, `-v2` | 20 + 8 live fault-stop runs | Fault stop and notifications |
| `data/esp_runs/fault-stop-v3`, `-v4`, `pull-over-v1` | 16 + 8 + 4 live runs | Deployed GRU/stuck rule, fixes |

- Webots is deterministic, so repeated runs need seeded start-pose jitter
  (lateral 3 mm, heading 4°).
- Splits are by run, never by window. Scalers are fit on training runs only.
- Fault types: motor gain (weak), motor stall, intermittent motor, encoder
  stuck, encoder noise, IR stuck, IR dead, IR noise, IR drift (ramp of 50 or
  100 units/s).

## 5. Anomaly detection: first models (Supervisor control)

`scripts/models/train_autoencoder.py`, `reports/fault-eval-v1`,
`reports/severity-sweep-v1`.

- **Features:** motor = wheel targets + encoder-derived wheel speeds (4); IR =
  three ground sensors (3); 20-sample windows (0.64 s), scaled.
- **Models:** dense and 1D-conv autoencoders (bottleneck 8) and a PCA
  baseline; score = window reconstruction MSE; alarm = 3 of last 5 windows.
- **Healthy test false alarms:** motor conv 0/14 runs (0.31% of windows over
  threshold); IR dense 2/14 (0.59%); IR PCA 5/14 (0.68%).
- **Faults (conv-v1, 80 runs):** every motor and encoder fault 8/8 (median
  delay 0.10-0.45 s); IR dead and drift 8/8; IR stuck centre 1/8; IR noise
  std 50 2/8; no pre-onset alarms; PCA misses most IR faults.
- **Severity sweep (104 runs):** right-motor power loss ≥20% detected 7-8/8,
  15% 4/8, ≤10% essentially never; autoencoders beat PCA between 15% and 30%.
  Right IR noise reliable only at std 200 (8/8); std 100 4/8.

## 6. Anomaly detection under ESP control

`models/esp-conv-v1`, `reports/esp-model-comparison-v1`,
`reports/esp-fault-eval-v1`.

- **Why retrain:** the motor features contain the controller's wheel targets,
  and the ESP's PD loop commands differently; the round trip also adds the
  stop, unload and U-turn. The Supervisor-trained `conv-v1`/`dense-v1` alarm
  on 18/18 healthy ESP runs, with 13-25% of windows over threshold.
- **Model:** conv autoencoder (Conv1d(F,16,k3) → Conv1d(16,16,k3,s2) →
  Linear(160,8) → Linear(8,32) → Linear(32,20F)); motor 5,208 and IR 4,500
  parameters.
- **Gating:** windows starting in the first 0.5 s, overlapping `AT_DEST`,
  `UNLOADING`, `U_TURN`, `FAULT_PULLOVER`, `FAULT_STOP`, or within 0.5 s after
  a U-turn are not monitored.
- **Threshold rule:** the lowest value at which no healthy validation run
  raises a persistent alarm. The 99.5th-percentile rule had given 7/18 motor
  and 5/18 IR healthy test alarms and 15/120 IR pre-onset alarms; the run-level
  rule gives 2/18, 0/18 and 0/120. Thresholds: motor 0.1894, IR 0.4476.
- **Fault results (120 runs, own model):**

| Fault | Detected | Median delay |
|---|---|---|
| Motor weak 20% (right) | 3/8 | 4.13 s |
| Motor weak 30% (right) | 6/8 | 1.78 s |
| Motor weak 50% (left) | 8/8 | 2.22 s |
| Motor stall (left) | 8/8 | 0.38 s |
| Motor intermittent (right) | 8/8 | 0.67 s |
| Encoder stuck (left) | 8/8 | 0.56 s |
| Encoder noise (right) | 8/8 | 1.06 s |
| IR stuck (centre) | 1/8 | 1.09 s |
| IR dead (left) | 8/8 | 0.53 s |
| IR noise std 50 (right) | 0/8 | - |
| IR noise std 100 (right) | 3/8 | 8.00 s |
| IR noise std 200 (right) | 8/8 | 2.16 s |
| IR noise std 100 (left) | 3/8 | 13.95 s |
| IR drift (centre) | 4/8 | 5.62 s |
| IR drift (right) | 6/8 | 4.35 s |

  Total 82/120 by the fault's own model, 98/120 by either model (IR noise
  also trips the motor model because the PD loop reacts to it). No alarm
  before a fault's onset in any run.

## 7. Inference on the ESP

- **First version:** plain float32 C++ (`anomaly.cpp`, weights generated by
  `scripts/models/export_c.py`): ~2.3 ms (motor) + 2.0 ms (IR) per frame at
  `-O2`.
- **Replay agreement v1** (`scripts/esp/replay_scores.py`, 18 healthy test
  runs + one of each fault, 65,044 windows): scores within 4.5e-5 of Python,
  2,559 alarm windows with 0 mismatches; one wheel target differed by
  0.001 rad/s.
- **Live v1** (`data/esp_runs/live-v1`, 16 round trips): no alarm on the 4
  healthy runs; all 12 fault runs detected on the ESP by the right model
  (0.1-5.2 s); no pre-onset alarm; logged scores within 3.3e-5 of Python.

## 8. Fault response: stop and notify

- **Fault stop** (route option `F@<s>`, mission `esp_fault_stop_s` 2.0): once
  either alarm has stayed on that long, the ESP turns ~90° in place (encoder
  heading), drives 8 cm forward (`FAULT_PULLOVER`) and stops (`FAULT_STOP`);
  it gives up after 6 s (healthy pull-over ~2.8 s, 50% weak motor ~4.1 s).
- **Notifications** (`main/notify.cpp`): WiFi station + HTTPS POST to ntfy.sh
  from a task on core 0 (control is pinned to core 1, so sending never blocks
  control); up to 6 attempts with a DNS-cache flush between them (one of
  ntfy.sh's three servers reset every TLS handshake); 20 s timeout. Text is
  plain language with the place from the route's `E@` labels, e.g. "AGV
  stopped: motor problem / The robot found a motor problem, moved off the
  track and stopped. It is next to the track between Junction 3 and
  Junction 2." `LINE_LOST` and `JUNCTION_MISSED` stops also notify, naming any
  alarm from the last 10 s.
- **WiFi settings without recompiling:** a 12 KB `wificfg` NVS partition
  written by `scripts/esp/flash_wifi.ps1`.
- **Live check v1/v2 (24 runs, the conv models only):**

| Fault (4 runs) | Ends in | Notified |
|---|---|---|
| Healthy | `AT_DEST` 4/4, no alarm | - |
| IR dead left | `FAULT_STOP` 4/4, 8-18 cm off the line, ~3 s | 4/4 |
| IR noise std 200 | `FAULT_STOP` 4/4, 7-8 cm off the line | 4/4 |
| Motor weak left 50% | `FAULT_STOP` 2/4, `LINE_LOST` 2/4 | 4/4 |
| Motor stall left | `LINE_LOST` 3/4, `FAULT_STOP` 1/4 | 4/4 |

## 9. Long-memory detectors: the GRU experiment

`scripts/models/gru_experiment.py`, `models/esp-gru-v1`,
`reports/esp-gru-eval-v1`.

Goal: catch the weak faults (20-30% power loss, IR stuck, light IR noise, IR
drift) without more false alarms, by pairing the conv models with
long-memory detectors. Same data, no new collection; whole runs instead of
windows; same gating, persistence and threshold rule.

### Detectors compared

| Detector | Description |
|---|---|
| conv | deployed conv autoencoders |
| conv-max | same reconstruction, scored by the worst single feature |
| diff | mean squared sample-to-sample difference, worst feature |
| stuck | a sensor flat (<0.5 units) for 1 s while the robot drives |
| motor GRU | plant model: GRU(32) predicts both wheel speeds from the wheel targets only; score = worst wheel's mean squared error over 62 samples (2 s) |
| IR GRU | one-step forecaster of the three sensors from the previous sensors, targets and speeds |

The motor GRU gets no measured speeds as input: with them it can copy the
last value and never see a weak wheel.

### Results (120 fault runs, 36 held-out healthy runs)

| Method | Healthy validation alarms | Healthy test alarms | Own-model detections | Either-model detections |
|---|---|---|---|---|
| conv (deployed) | 0/18 | 2/18 | 82/120 | 98/120 |
| conv + motor GRU + stuck (GRU threshold ×1) | 0/18 | 3/18 | 94/120 | 110/120 |
| **conv + motor GRU ×2 + stuck (deployed)** | **0/18** | **2/18 (same runs)** | **94/120** | **110/120** |

| Fault | conv | conv + GRU×2 + stuck | Delay before → after |
|---|---|---|---|
| Motor weak 20% | 3/8 | **8/8** | 4.13 → 0.29 s |
| Motor weak 30% | 6/8 | **8/8** | 1.78 → 0.18 s |
| Motor weak 50% | 8/8 | 8/8 | 2.22 → 0.19 s |
| Motor stall | 8/8 | 8/8 | 0.38 → 0.11 s |
| Motor intermittent | 8/8 | 8/8 | 0.67 → 0.10 s |
| Encoder stuck | 8/8 | 8/8 | 0.56 → 0.11 s |
| Encoder noise | 8/8 | 8/8 | 1.06 → 0.18 s |
| IR stuck | 1/8 | **6/8** (6 observable) | 1.09 → 1.82 s |
| IR dead | 8/8 | 8/8 | 0.53 s |
| IR noise 50 / 100 / 200 / 100L | 0 / 3 / 8 / 3 of 8 | unchanged | - |
| IR drift centre / right | 4 (5 observable) / 6 of 8 | unchanged | - |

- **GRU threshold margin:** the motor GRU's weakest fault signal (20% loss)
  peaks at ≥25.7× its no-validation-alarm threshold (0.0123), its worst healthy
  test run at 1.01×. At 1× it adds one healthy test alarm, so the deployed
  threshold is 2× (0.0246); any factor from ~1.1× to ~20× gives the same table.
- **Observable ceiling:** 2 stuck and 3 centre-drift runs start the fault
  during a U-turn and lose the line before any monitored window exists; no
  detector can see them.
- **Did not help:** the IR GRU (one-step forecasting copies the slow drift ramp
  of 1.6-3.2 units/frame; +1 healthy test alarm), worst-feature conv scoring
  (thresholds rise with it), and the sample-difference score (line crossings
  make healthy differences large).
- **Unique catches:** the motor GRU catches 7 runs the motor conv misses; the
  motor conv catches 4 the GRU misses (stall, stuck encoder, 50% loss: runs
  that stop before the GRU's 2 s window fills). The IR conv is the only
  detector for IR noise and drift (33 IR detections; the stuck rule 14).

## 10. esp-dsp deployment

- **Library choice:** Espressif's **esp-dsp** (official DSP library;
  `dsps_dotprod_f32` and `dspm_mult_f32` have ESP32-S3 assembly using the S3's
  SIMD instructions). ESP-DL was not used for the deployed models: its Conv,
  Gemm and MatMul run only quantized (no float32), which would shift scores
  against thresholds set at the healthy-validation edge (see section 15 for
  what quantization does).
- **Layout:** each conv layer is one `dspm_mult_f32` on an im2col patch
  matrix; each dense/GRU row one `dsps_dotprod_f32`. The SIMD path needs
  16-byte alignment and sizes that are multiples of 4, so `export_c.py` writes
  the weights pre-arranged and zero-padded (`alignas(16)`), and permutes the
  encoder Linear for a time-major input.
- **Weights in RAM:** read from flash through the 32 KB cache shared with the
  WiFi stack, the three detectors took 1.96 + 1.51 + 1.32 ms (bound by ~52 KB
  of weights per frame over DIO flash, ~20 MB/s). Each detector copies its
  weights into internal RAM at start-up (~63 KB; free heap 228 → 165 KB).

| Per frame | Scalar C, flash | esp-dsp, flash | **esp-dsp, RAM** |
|---|---|---|---|
| Motor conv | 2.3 ms | 1.96 ms | **0.31 ms** |
| IR conv | 2.0 ms | 1.51 ms | **0.36 ms** |
| Motor GRU | - | 1.32 ms | **0.38 ms** (max 0.54) |

- **Replay agreement v2** (`reports/esp-replay-v2`, 10 runs: 2 healthy test,
  20%/30% weak motor, stall, stuck encoder, 2 stuck IR, dead IR, plus the
  failed live stuck run): conv scores within 4.95e-5, GRU scores within
  1.52e-5 (threshold 0.025), GRU/stuck/motor/IR alarms identical on every
  frame (e.g. 988/988 GRU alarm frames on a 20% weak motor run).
- **Protocol:** the `M` frame grew from 11 to 14 fields (conv scores, combined
  motor/IR alarms, GRU score, GRU alarm, stuck alarm); `Q` reports the timing
  of all three models.
- Image 0xeae30 bytes (~0.96 MB), 69% of the 3 MB app partition free.

## 11. Live fault-stop checks

### fault-stop-v3 (motor GRU and first stuck rule)

| Fault (4 runs) | Ends in | First alarm after onset | Notified |
|---|---|---|---|
| Healthy | `AT_DEST` 4/4, no alarm | - | - |
| Motor weak right 20% | `FAULT_STOP` 4/4, 8-9 cm off the line | GRU 0.1-0.3 s (5.4 s when the onset is in the U-turn) | 4/4 |
| Motor weak right 30% | `FAULT_STOP` 4/4, 8-9 cm off the line | GRU 0.1-0.2 s (4.2 s, U-turn) | 4/4 |
| IR stuck centre | `FAULT_STOP` 2/4, ran off the track 1/4, `LINE_LOST` 1/4 | stuck 1.1-1.7 s | 3/4 |

### fault-stop-v5 (search winners deployed, 2026-11-27)

Same routes, seeds, onsets and faults as fault-stop-v3, with the smaller motor
conv and GRU (section 16):

| Fault (4 runs) | Ends in | First alarm after onset | Notified |
|---|---|---|---|
| Healthy | `AT_DEST` 4/4, no alarm frame | - | - |
| Motor weak right 20% | `FAULT_STOP` 4/4 | 0.12-0.24 s (3.8 s, onset in the U-turn); v3 0.15-0.34 s (5.4 s) | 4/4 |
| Motor weak right 30% | `FAULT_STOP` 4/4 | 0.12-0.14 s (3.9 s, U-turn); v3 0.12-0.21 s (4.2 s) | 4/4 |
| IR stuck centre | `FAULT_STOP` 3/4, `LINE_LOST` 1/4 (D, onset in the U-turn) | 0.7-1.1 s | 4/4 |

12 notifications for the 12 stops; no alarm before an onset.

### parallel-edges-v1/v2 (edge-002 and edge-004, 2026-11-27)

The mission option `avoid_edges` plans without the listed edges, so the
shortest path takes the other edge of a parallel pair. Round trips at 2.5 and
3.5 rad/s, no faults:

| Edges | Routes | Global scale 0.92 (v1) | Per-edge scales (v2) |
|---|---|---|---|
| edge-004 (avoid edge-005) | B: right turn onto it at node-002; D: left at node-003 | 4/4 complete, stops 0.1-1.6 cm | 4/4, stops 0.1-1.5 cm |
| edge-002 (avoid edge-003/004/005) | A: straight onto it at node-004; C: left at node-001 | 0/4: `AT_DEST` 8.4-10 cm before the node | 4/4, stops 0.3-0.9 cm |
| Standard A-D, 3.0 rad/s | | | 4/4, stops 0.1-1.4 cm, no alarm frame |

- **Cause of the v1 failures:** odometry reads ~0.973 of the map length on
  the long edges (001, 002, 006) and ~0.836 on the loop (004, 005); 0.92 was
  their average on routes that combine one of each. A least-squares fit over
  237 legs (healthy ESP runs plus these) of one scale per edge type predicts
  every leg's arrival within 0.7 cm; it is the mission's
  `esp_odometry_scale_by_edge` (edge-003, never driven, keeps 0.92).
- Max path error 2.4-2.5 cm, as on the standard routes.
- **Alarms:** none on edge-004. On edge-002 the motor conv model raises
  short persistent alarms (≤0.29 s, below the 2 s fault-stop hold, so no
  stop) at the left turn from edge-001 onto edge-002 and on arrival after
  edge-002: manoeuvres missing from the training data. The v1 motor model
  does the same (≤0.35 s). Adding edge-002 runs to the healthy data would
  remove them.

### track2: a second, unseen track (2026-11-27)

`scripts/maps/make_track.py` draws `maps/image/track2.png` in the base
track's style (40 px lines, rounded corners, T-junctions only; a rounded
rectangle split by a vertical bar, with a horizontal bar from its middle to
the right side), converts it to `maps/paths/track2.json` and copies the base
track's world transform. `simulation/worlds/Self Diagnosis AGV track2.wbt`
shows it; `maps/routes/mission-track2.json` selects the map (`map_path`),
defines routes A-D covering all six edges (`routes`, with `avoid_edges`)
and uses one odometry scale, 0.973 (the base track's long-edge value, not
tuned here). Nothing was trained on this track.

**First run (`track2-v1`, firmware as on the base track):** healthy 3/8.
Detection carried over: on the three routes that navigated, 12/12 faults
stopped and notified, no pre-onset alarm, false alarms ≤0.32 s. Navigation
failed on geometry the base track never has:
- **Junction approached along a horizontal line (B, C):** the 2400 × 1800
  image is stretched onto the 1 m square arena, so horizontal lines are 2.2 cm
  thick and vertical ones 1.7 cm. On a horizontal line all three ground
  sensors read dark all the time, which is the junction pattern, so the ESP
  turned as soon as the ±0.10 m window opened, ~11 cm early.
- **Destination reached from the stem of a T (A 2.5 return):** the front
  sensors cross the line and see white ~4 cm before the stop distance; the
  750 ms line search ran out 0.4 mm before it (`LINE_LOST`).
- **90-degree corners onto a straight crossing line (B, C):** the ordinary
  latched turn (error ±1) has a ~5.4 cm radius at 3.5 rad/s and misses them.
  The base track's turns (61-76° measured 4 cm out) curve away onto its
  circle.

**Navigation fixes (firmware and host):**
- A junction pattern already present when the window opens waits until 3 cm
  before the junction distance (base-track junctions are taken 2.5-3.6 cm
  before the node).
- Line lost within 5 cm of the destination: straight on at 1 rad/s to the
  stop distance (≤3 s).
- Corners: the host sends `l`/`r` for turns of ≥85° measured 4 cm along each
  edge (track2's 90° corners; none on the base track, whose turns measure
  61-76°: its circle leaves at 90° but curves away). The ESP slows to
  1.5 rad/s, spins in place once the axle is on the crossing line (junction
  distance, 4 cm after the pattern appears, or 2 cm after the sensors run off
  the line's far side), and follows once ≥60° has turned with the centre
  sensor on a line.
- Corner spins are not monitored, like the U-turn (both firmware and the
  Python reference; a spin is a junction turn with opposite-sign wheel
  targets): before that, the motor conv and GRU alarmed for up to 1.86 s on
  healthy right-hand corner spins, near the 2 s stop hold.
- Tried and dropped: treating any already-dark L/R junction as a corner
  spun the robot at the base track's circle junctions, where the pattern is
  often present when the window opens.

**Results with the fixes (`nav-fix-v1`, `track2-v2`):**

| Runs | Result |
|---|---|
| Base track A-D × 2.5/3.5, round trips | 8/8, stops 0.1-1.5 cm |
| Base track over edge-002 and edge-004 | 8/8 |
| track2 A-D × 2.5/3.5, round trips | 8/8 (was 3/8), stops 0.1-1.8 cm; longest healthy alarm 0.42 s (was 1.86 s) |
| track2 faults, A/B/C/D, 20 runs | `FAULT_STOP` 20/20, every stop notified, none before onset; motor 0.10-0.27 s, dead IR 0.4-0.5 s, stuck IR 0.9-1.1 s |
| Replay of the 10 base runs | identical wheel targets and alarms |
| ESP smoke test | 47 checks, incl. stale junction, corner and blind arrival |

Max path error 2.5 cm on both tracks. Logged ESP alarms on the track2 runs
match the Python reference (spin gating included) on every frame.

### track3: the hard course (2026-11-27)

`python -m scripts.maps.make_track --track track3`: an octagonal outer loop
(every corner two true 45° bends), a Y junction (a branch leaving the top line
at 45°), a hairpin (180°, ~6.3 cm radius), a tight 90° curve (~4 cm radius) and
a cross bar; 4 junctions, 6 edges. Routes A-D
(`maps/routes/mission-track3.json`) cover every edge: A down the Y branch,
through the hairpin and the tight curve; B bears onto and off the Y branch; C
straight through the Y along the octagon; D the cross bar, a corner and the
long bottom loop. Nothing was trained on this track.

**First run (`track3-v1`):** healthy 2/8 (only D). The PD loop held the
hairpin and the tight curve (path error ≤2.2 cm); the failures were:
- **Odometry:** one scale (0.973) stopped A and C ~6-7 cm past the node. Mid-
  edge odometry ratios on this track are 0.86-0.94 on the polyline/curved
  edges and 1.00 on the straight cross bar; they are now the mission's
  per-edge scales.
- **Y junction:** the turn onto the 45° branch released after two frames (the
  pattern clears while the branch is still beside the old line) and the robot
  followed the top line.
- **Turn classification:** the skeleton puts the Y's node ~4 cm past the
  branch point, so the angle at the node's neighbours called C's straight
  pass a turn.

**Fixes:** L/R/S/corner are classified from the angle 4 cm along each edge
(new routes only; replays of older runs keep the old commands, and the base
track's commands are unchanged apart from edge-002's near-straight junction,
now `S`). Turns of 35-58° are sent as `<`/`>` (bear): the ESP waits for the
junction distance less 3 cm and holds the turn until it has turned 25°.

**Results (`track3-v2`, final firmware):**

| Runs | Result |
|---|---|
| track3 A-D × 2.5/3.5, round trips | 8/8, stops 0.3-2.0 cm, max path error 2.5 cm |
| track3 faults, A-D at 3.0, 16 runs | `FAULT_STOP` 16/16, notified, none before onset; motor 0.10-0.23 s, dead IR 0.29-0.50 s, stuck IR 1.0-1.1 s |
| Healthy false alarms | longest 0.77 s (A at 3.5, hairpin), below the 2 s stop hold |
| Regression: base track and track2 A-D at 3.5 | 8/8 |
| Replay of the 10 base runs; smoke test | identical; 48 checks |

- Marginal: at 3.5 rad/s the hairpin loses the line for up to 23 frames
  (0.74 s) against the 750 ms search limit; at 2.5 rad/s for 1 frame.
- Logged ESP alarms match the Python reference on every frame.

### fault-stop-v4 and pull-over-v1 (after both fixes)

| Runs | Ends in | Notes |
|---|---|---|
| Healthy, 4 | `AT_DEST` 4/4 | 0 alarm frames |
| IR stuck centre, 4 | `FAULT_STOP` 3/4, `LINE_LOST` 1/4 | stuck alarm +1.0-1.1 s; the v3 run that ran off the track now stops; route B pulled over left on the loop (7.6 cm from its line); route D's onset is in the U-turn (notified "lost the track") |
| IR dead left from 1 s on route C, 2 | `FAULT_STOP` 2/2 | pulled over left on edge-005 (Junction 2 → 3), 4-5 cm outside the loop |
| Motor weak 30% from 1 s on route C, 2 | `FAULT_STOP` 2/2 | alarm +1.5 s, past Junction 3, so right (edge-006); 5 cm from edge-004 at the junction |

No alarm came before an onset in any live run, and every ESP stop sent its
notification (checked on the ntfy topic).

## 12. Stuck-sensor rule fix

- **Failure (fault-stop-v3, route C 2.4 rad/s):** the centre sensor stuck at
  its on-the-line value (300). The ESP believed it was on the line and drove
  straight off the track; off the track the other two sensors read flat white
  floor (~765), so the first rule ("flat while another sensor's range exceeds
  50") only fired for ~1 s while crossing a line, short of the 2 s hold.
  Webots ended the run at its 12 cm path-error limit: no stop, no
  notification.
- **Fix:** flat (every sample-to-sample change < 0.5 for 1 s) **while the
  robot drives** (mean (|left| + |right|)/2 wheel speed > 1 rad/s). Healthy
  sensors never stay within 1.4 units over 1 s, even when the robot is still.
- **Checks:** offline the same detections (stuck 6/6 observable, dead 8/8)
  and no healthy or pre-onset alarm; replay 0 mismatches on 10 runs (the
  failed run now has 360 stuck-alarm frames); live stuck IR `FAULT_STOP` 3/4,
  the 4th unobservable.

## 13. Map-based pull-over side

- **Problem:** the pull-over always turned right. On the small loop
  (edge-004/edge-005, 12-19 cm apart), driving edge-005 from Junction 2 to
  Junction 3, a right turn ends on edge-004.
- **Fix:** `scripts/esp/route_actions.pull_over_sides` checks, every 1 cm
  along each route step (except 6 cm at each end, where lines meet anyway),
  the robot's swept path out to 8 cm + its 3.7 cm radius on both sides
  against all other edges and the arena walls. It picks right unless right
  comes within 3 cm and left is clearer. The controller sends one side per
  step (`P@RL…`, laid out like `E@`); the ESP turns that way.
- **Clearances on the base track** (worst along the step):

| Leg | Steps | Right | Left | Chosen |
|---|---|---|---|---|
| A out (node-003 → node-001) | edge-005, edge-001 | 4.7 / 6.8 cm | 0.1 / 5.9 cm | R R |
| A back / B out | edge-001, edge-005 | 5.9 / 0.0 cm | 5.9 / 4.6 cm | R **L** |
| C out (node-002 → node-004) | edge-005, edge-006 | 0.0 / 6.3 cm | 4.6 / 6.1 cm | **L** R |
| C back / D out | edge-006, edge-005 | 6.0 / 4.7 cm | 6.3 / 0.1 cm | R R |

  Half of the round trips' legs would have pulled over onto the other loop
  line before the fix.
- Computing the sides takes ~0.5 s per leg (numpy). The smoke test checks a
  left pull-over and rejects malformed `P@` options.

## 14. On-device latency table

`scripts/models/esp_latency.py`, firmware `B` command (`main/bench.cpp`),
`reports/esp-latency-v1/latency.csv`.

- The firmware runs the same esp-dsp forward passes as the detectors with
  runtime layer sizes and random weights in internal RAM, so any architecture
  can be timed without reflashing. 967 architectures were measured (median of
  31 runs each).
- **Agreement with the deployed detectors:** motor conv 307 µs (deployed
  314 µs); the deployed IR conv and GRU add ~70-100 µs for scaling, the score
  window and gating.
- **Conv autoencoders:** 88 µs - 1.44 ms per frame, 4.1-91.6 KB of weights
  (window 10-32, 8-32 channels, kernel 3/5, bottleneck 4-16, hidden 16-64).
- **GRU step:**

| Hidden units | 8 | 12 | 16 | 24 | 32 | 48 | 64 |
|---|---|---|---|---|---|---|---|
| Time (µs) | 63 | 95 | 127 | 195 | 267 | 421 | 589 |
| Weights (KB) | 1.4 | 2.6 | 4.3 | 8.6 | 14.5 | 30.8 | 53.0 |

## 15. INT8 / INT16 benchmark

`scripts/models/quantize_int8.py` (esp-ppq, separate Python 3.12 env),
`firmware/esp-dl-bench` + `scripts/esp/int8_bench.py` (ESP-DL on the ESP),
`reports/esp-int8-v1`.

### On the ESP (median of 101 runs)

| Model | float32 (esp-dsp) | INT8 (ESP-DL) | INT16 (ESP-DL) |
|---|---|---|---|
| Motor conv | **307 µs**, 20.3 KB | 747 µs, 5.6 KB | 737 µs, 11.1 KB |
| IR conv | **294 µs**, 17.8 KB | 743 µs, 4.8 KB | 768 µs, 9.6 KB |
| Motor GRU | **267 µs**, 14.5 KB | 813 µs, 5.0 KB | 821 µs, 20.2 KB |

- The ESP's INT8 outputs equal esp-ppq's simulation exactly (max difference
  0), INT16 within 5e-6, so the simulated accuracy below holds on the chip.
- ESP-DL is ~2.5× slower here: these models do ~17k (conv) or ~3k (GRU)
  multiply-adds per frame, so its fixed per-layer runtime cost (module
  dispatch, layout conversion, requantization, reshape/transpose copies, LUT
  activations) dominates. INT8 and INT16 take the same time, and all three
  models ~0.75-0.8 ms regardless of size.

### Accuracy (simulated on every run)

| Model | Precision | Score error p99 / threshold | Detected, float threshold | Healthy alarms, float threshold | Detected, own threshold | Healthy test alarms, own threshold |
|---|---|---|---|---|---|---|
| Motor conv | float32 | - | 49/56 | 0 val, 2 test | 49/56 | 2/18 |
| Motor conv | INT8 | 8.86 | 56/56 | **18 val, 18 test** | **12/56** | 0/18 |
| Motor conv | INT16 | 0.00 | 49/56 | 1 val, 2 test | 49/56 | 2/18 |
| IR conv | float32 | - | 33/64 | 0, 0 | 33/64 | 0/18 |
| IR conv | INT8 | 0.49 | 36/64 (8 pre-onset) | 3 val, 0 test | 31/64 | 0/18 |
| IR conv | INT16 | 0.50 | 36/64 (8 pre-onset) | 3 val, 0 test | 35/64 | 0/18 |
| Motor GRU | float32 | - | 52/56 | 0, 0 | 52/56 | 0/18 |
| Motor GRU | INT8 | 2.88 | 52/56 | 0, 0 | 52/56 | 0/18 |
| Motor GRU | INT16 | 0.05 | 52/56 | 0, 0 | 52/56 | 0/18 |

- **Motor conv INT8 is unusable:** its healthy reconstruction error (~0.03) is
  below INT8's rounding noise.
- **Decision: keep float32 on esp-dsp:** faster, matches Python to ~1e-5,
  and the motor model needs more than 8 bits. INT8's only gain is ~4× smaller
  weights, and RAM is not a constraint.

## 16. Architecture search (Optuna)

`scripts/models/search_architectures.py`, `reports/arch-search-v1`.

- **Method:** multi-objective TPE per detector: maximize detections on the
  **tuning** half of the fault runs (A 2.4, B 3.4, C 2.6, D 3.1 rad/s; minus
  0.05 × median delay as a tie-break), minimize the **measured** ESP time
  from section 14. Deployed threshold rule; the deployed architecture is
  trial 0. Trials use a reduced training budget (≤60 epochs, every 2nd
  training window; GRU 2,000 steps). Trials: motor conv 40, IR conv 40, GRU
  12.
- **Final check:** Pareto candidates retrained with the deployed models' full
  budget (≤300 epochs, all windows; GRU 4,000 steps) and compared with the
  deployed models on the **held-out** half (the other speed of each route)
  and the healthy test runs, neither seen by the search.

| Detector | Version | Architecture | Held-out detected | Median delay | Healthy test alarms | ESP time | Weights |
|---|---|---|---|---|---|---|---|
| Motor conv | deployed | window 20, 16 ch, bottleneck 8, hidden 32 | 25/28 | 0.70 s | 2/18 | 307 µs | 20.3 KB |
| Motor conv | **candidate 0** | window 10, 8 ch, bottleneck 12, hidden 16 | **28/28** | **0.40 s** | 2/18 | **100 µs** | 6.6 KB |
| Motor conv | candidate 1 | window 10, 8 ch, bottleneck 16, hidden 16 | 27/28 | 0.45 s | 1/18 | 104 µs | 7.5 KB |
| IR conv | deployed | window 20, 16 ch, bottleneck 8, hidden 32 | 17/32 | 2.75 s | **0/18** | 294 µs | 17.8 KB |
| IR conv | candidate 0 | window 10, 8 ch, bottleneck 4, hidden 32 | **22/32** | 2.48 s | 2/18 | **100 µs** | 6.3 KB |
| IR conv | candidate 1 | window 10, 24 ch, bottleneck 4, hidden 32 | 16/32 | 2.42 s | 2/18 | 263 µs | 14.4 KB |
| Motor GRU | deployed | 32 hidden, 62-sample score window | 27/28 | 0.16 s | 0/18 | 267 µs | 14.5 KB |
| Motor GRU | **candidate 0** | 16 hidden, 31-sample window | **28/28** | **0.13 s** | 0/18 | **127 µs** | 4.3 KB |
| Motor GRU | candidate 1 | 8 hidden, 47-sample window | 27/28 | 0.13 s | 0/18 | 63 µs | 1.4 KB |

No version alarmed on a healthy validation run or before a fault's onset.

- **Motor conv and GRU:** smaller models are better on every measure (2-3×
  faster, 3× smaller, equal or more detections, faster alarms).
- **IR conv:** candidate 0 detects 5 more held-out runs and is 3× faster, but
  raises 2/18 healthy test alarms where the deployed model has none: a
  trade-off.
- Candidate 1 of the IR search looked best during the search (21/32 tuning)
  but reached 16/32 after full retraining, which is why the final check is on
  held-out data with full training.
- **Pattern:** shorter windows (10 samples, 0.32 s) and fewer channels/units
  than the deployed sizes. The deployed architectures were reasonable but
  larger than needed.
- **Deployed (2026-11-27):** motor conv candidate 0 and GRU candidate 0
  (`models/esp-conv-v2` with the unchanged IR model, `models/esp-gru-v2`;
  the IR candidate stays undeployed because of its healthy alarms). The
  export and firmware now take each conv model's window, channels,
  bottleneck and hidden size, with buffers sized for the largest.
  - Re-exporting v1 with the new code changes only the size constants, not a
    weight.
  - **Replay** (`reports/esp-replay-v3`, the same 10 runs as v2): 0 alarm
    mismatches; conv within 1.3e-5, GRU within 2.5e-5 (at a score of 0.86,
    30× the threshold; the GRU check is now relative above 0.2).
  - **ESP time,** median over one replayed run on the same firmware:
    motor conv 342 → 123 µs, GRU 411 → 259 µs, IR conv 424 µs (unchanged);
    1.18 → 0.81 ms per frame.
  - **Offline, all 120 fault runs** (`reports/deployed-eval-v2`): 94/120 by
    the fault's own detector and 110/120 by either for both v1 and v2,
    0 pre-onset, same healthy alarms. Motor faults alarm sooner: median
    0.16 → 0.13 s, mean 1.57 → 1.32 s.
  - Live: fault-stop-v5 (section 11).

## 17. Current performance summary

| Area | Result |
|---|---|
| Route following | Base track A-D and its parallel edges, track2 and track3 A-D, round trips at 2.5/3.5 rad/s; stops 0.1-2.0 cm from the node; max path error 2.5 cm |
| Detection (offline, 120 fault runs) | 94/120 by the fault's own detector, 110/120 by either (stops the robot); was 82 / 98 |
| False alarms | Healthy validation 0/18, healthy test 2/18 (motor conv, unchanged), pre-onset 0/120; live healthy runs 16/16 without an alarm (live-v1, fault-stop-v1, -v3, -v4) |
| Motor faults | All 7 types 8/8, median alarm 0.10-0.29 s after onset |
| IR faults | Stuck 6/8 (6 observable), dead 8/8, noise std 200 8/8; noise std ≤100 0-3/8 and drift 4-6/8 by the IR model |
| Live fault stops (after fixes) | Weak motor 8/8, stuck IR 3/4 (4th unobservable), dead IR 2/2, every stop notified; same with the v2 models (fault-stop-v5); unseen tracks: track2 20/20, track3 16/16 |
| ESP compute | ~0.81 ms of each 32 ms frame (2.5%) after deploying the search winners; 1.18 ms for v1 measured the same way |
| ESP memory | ~51 KB of weights in RAM with the v2 models (both conv buffers sized for the larger IR model; was ~63 KB); image 0.94 MB (70% of the 3 MB partition free) |
| ESP vs Python | conv within 1.3e-5, GRU within 2.5e-5 (relative 3e-5), alarms identical on every frame (esp-replay-v3/-v4); logged live alarms match Python on track2/track3 |
| Tests | 47 unit tests; ESP smoke test, 48 checks (PD, junctions, stale junction pattern, corner, bear, blind arrival, U-turn, fault stop both sides, WiFi, malformed routes) |

## 18. Known limitations and open work

In priority order:

1. **IR drift and light IR noise** are the main detection gap: the IR model
   catches drift 4-6/8 and noise std ≤100 0-3/8 (the motor model reacts to
   most noise runs, so the robot still stops in 5-7/8). A one-step GRU does
   not help (it copies the slow ramp). Candidates: a level check (the
   on-line and floor readings shift with drift) or a cross-sensor model that
   predicts each sensor from the other two and the motion.
2. **Done: search winners deployed** (motor conv, GRU). Open: the smaller
   IR conv (22/32 vs 17/32 held-out, but 2/18 healthy test alarms) is a
   choice for the user.
3. **Done: COM3 reconnect.** `epuck_route` reopens the port for up to 30 s
   when a serial call fails, with Webots paused, and resends the frame; the
   ESP stops on `LINK_LOST` meanwhile and resumes. Tested against a fake
   port that drops and comes back; not yet with a real unplug.
4. **Done: parallel edges** driven with `avoid_edges` (section 11). Open: the
   healthy training data has no edge-002 runs, so the motor conv model
   raises sub-0.3 s alarms there (no stop).
5. **Faults during a U-turn** that lose the line before any monitored window
   cannot be detected (2 stuck and 3 drift runs of 120); they still stop with
   `LINE_LOST` and notify.
6. **Pull-over near a junction** can end ~5 cm from another line (lines meet
   there; the side check skips 6 cm at each end of an edge).
7. **Motor conv false alarms:** 2/18 healthy test runs (the same two runs in
   every configuration).
8. **Hairpin margin** (track3, 3.5 rad/s): the line search nearly runs out
   (0.74 of 0.75 s).
9. **Per-track odometry scales** come from each track's own healthy runs; a
   new track needs a few runs to measure them. The Y junction node is placed
   ~4 cm past the branch by the converter (the bear threshold allows for it).
10. One onset time per fault in the ESP fault set; a combined CNN+GRU model was discussed but
   not tried (separate models keep fault attribution and per-model
   thresholds, and compute is not a constraint).

## 19. Where everything is

| Path | Contents |
|---|---|
| `firmware/main/` | ESP firmware: `main.cpp` (control, protocol, fault stop), `anomaly.{h,cpp}` (detectors, esp-dsp), `anomaly_model.h` (generated weights), `bench.{h,cpp}` (latency benchmark), `notify.{h,cpp}` (WiFi/ntfy) |
| `firmware/esp-dl-bench/` | separate ESP-DL app for the INT8/INT16 benchmark |
| `maps/` | track images (`base track`, `track2`, `track3`), graph maps (`paths/`), missions (`routes/mission*.json`) |
| `simulation/worlds/` | one Webots world per track |
| `simulation/controllers/epuck_route/` | Webots controller and serial bridge |
| `scripts/maps/`, `scripts/telemetry/`, `scripts/faults/` | map tools (incl. `make_track.py` for track2/track3), data collection/preparation, fault runs and evaluation |
| `scripts/models/` | training, GRU experiment, export, latency sweep, quantization, architecture search |
| `scripts/esp/` | route conversion and pull-over sides, smoke test, replay, WiFi tools, INT8 bench |
| `models/` | `esp-conv-v2`, `esp-gru-v2` (deployed), `esp-conv-v1`, `esp-gru-v1` (previous), `esp-quantized-v1` (INT8/INT16 exports), older `conv-v1`/`dense-v1` |
| `reports/` | every evaluation referenced above |
| `data/esp_runs/` | healthy, fault and live ESP runs (manifests in git; the run CSVs are git-ignored, local only) |
| `docs/` | `USAGE.md` (commands), `STATUS.md`, `DECISIONS.md`, `TODO.md` |
