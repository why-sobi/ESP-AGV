# Handoff: ESP-AGV (single car), 2026-11-26

## What the project is

A simulated AGV (Webots e-puck) follows a line track between named nodes.
An ESP32-S3 on USB serial (COM3) is the robot's onboard computer:

1. The Supervisor controller (`simulation/controllers/epuck_route`) plans a
   route on the track graph and sends the ESP one route command: a turn
   action per junction, tagged with its expected odometry distance.
2. Every 32 ms simulation step, Webots sends the ESP the three ground-IR
   readings and both wheel-encoder positions.
3. The ESP runs the PD line follower, junction and destination logic, and two
   anomaly detectors (motor and IR autoencoders). It returns wheel targets,
   its state, and the anomaly scores and alarms.
4. Webots applies the wheel targets and logs everything to CSV.

Faults (weak, stalled or intermittent motor; stuck or noisy encoder; stuck,
dead, noisy or drifting IR sensor) are injected in Webots, so the ESP sees
them only through its inputs, as on real hardware.

Scope is one car. Webots stays the environment; there is no physical robot.

## Where it stands

The single-car pipeline is complete and verified end to end. Nothing from
the 2026-11-26 session is committed yet (see "Git state" below).

| Piece | State | Evidence |
|---|---|---|
| Route planning to ESP route command | Done | Routes A-D, one way and round trip |
| ESP PD control, junctions, destination, U-turn | Done | Stops within about 1.3 cm of the node; speeds 2.0-4.0 rad/s |
| Webots-ESP bridge | Done | Recovers from Webots pauses; collectors retry USB drops |
| Healthy and fault datasets under ESP control | Done | 108 healthy and 120 fault round trips |
| Motor and IR models (`models/esp-conv-v1`) | Done | Healthy-test false alarms: motor 2/18, IR 0/18 |
| Inference on the ESP | Done | Matches Python: scores within 4.5e-5, identical alarms |
| Live end to end | Done | 16/16 runs behave as expected |
| What the robot does on an alarm | **Open** | Alarms are only reported and logged |

### Detection results (`reports/esp-fault-eval-v1`, 120 fault runs)

- **Detected 8/8:** motor stall, intermittent motor, 50% motor power loss,
  stuck encoder, noisy encoder, dead IR sensor, IR noise std 200.
- **Partial:** 30% motor power loss 6/8; 20% loss 3/8; IR drift 4-6/8; IR
  noise std 100 3/8.
- **Missed:** IR stuck 1/8; IR noise std 50 0/8.
- **No alarm before a fault's onset** in any of the 120 runs.
- **IR noise also trips the motor model** (7-8/8 at std 100), because the PD
  loop reacts to the noisy readings. "Either model alarms" catches far more
  IR faults than the IR model alone, but it does not say which part failed.
- **The old Supervisor-trained models fail on ESP data.** `conv-v1` and
  `dense-v1` alarm on all 18 healthy ESP test runs, with 13-25% of windows
  over threshold (`reports/esp-model-comparison-v1`). Do not use them for
  the ESP.

### How the models are monitored (Python and ESP identically)

- **Score:** reconstruction MSE of a 20-sample (0.64 s) window. Motor
  features are the wheel targets and the encoder-derived wheel speeds; IR
  features are the three ground sensors.
- **Not monitored:**
  - windows starting in the first 0.5 s of a route;
  - windows overlapping `AT_DEST`, `UNLOADING` or `U_TURN`;
  - the first 0.5 s after a U-turn.
  These are commanded manoeuvres and settling transients. They are still used
  for training.
- **Threshold:** the lowest value at which no healthy validation run raises an
  alarm (`--threshold-rule no-validation-alarm`).
- **Alarm:** at least 3 of the last 5 windows exceed the threshold.

## What was verified

All of the following passed on 2026-11-26:

- **Unit tests:**
  `.\.venv\Scripts\python.exe -m unittest discover -s tests` (36 tests).
- **ESP smoke test:**
  `.\.venv\Scripts\python.exe -m scripts.esp.esp_serial_smoke_test --port COM3`.
  It covers PD output, odometry, junction windows, destination, missed
  junction, U-turn, route speed, anomaly-score start and inference timing,
  and malformed routes.
- **Supervisor regression:** a Supervisor-mode route A run at 2.5 rad/s is
  byte-identical, in the first 15 columns, to the committed
  `data/route_runs/route-20261114-001915-A-2.5.csv`.
- **ESP vs Python agreement:** `scripts/esp/replay_scores.py` on 18 healthy
  and 15 fault runs (65,044 windows). Scores agree within 4.5e-5 and every
  alarm decision matches (2,559 alarm windows). One wheel target in one run
  differed by 0.001 rad/s, a rounding difference of the `-O2` build. Saved in
  `reports/esp-replay-v1/agreement.json`.
- **Live end to end** (`data/esp_runs/live-v1`):
  - 4 healthy round trips: no alarms.
  - 50% weak motor: 4/4 detected by the motor model (0.5-5.2 s after onset).
  - Dead IR sensor: 4/4 detected by the IR model (0.1-3.9 s).
  - IR noise std 200: 4/4 detected by the IR model (0.7-3.9 s).
  - No alarm before any fault's onset. The logged live scores match Python
    within 3.3e-5.
- **ESP cost:** inference takes about 2.3 ms (motor) + 2.0 ms (IR) per
  frame. The firmware image is 0x336b0 bytes, leaving 80% of the app
  partition free.

## Read these before continuing

1. `docs/STATUS.md`: verified scope, with every result and its limitations.
2. `docs/DECISIONS.md`: why things are the way they are. Decisions 6-12
   cover odometry-gated junctions, link timeout, the U-turn, ESP retraining,
   manoeuvre gating, the threshold rule and float32 inference.
3. `firmware/README.md`: the serial protocol (route, sensor, motor and `Q`
   frames), controller behaviour and the anomaly-detection section.
4. `docs/USAGE.md`: every command. Start with "ESP-controlled runs",
   "ESP-controlled models (the ones to deploy)" and "Inference on the ESP".
5. `docs/TODO.md`: open items.
6. `scripts/README.md`: one-line guide to each script.

## Environment notes

- **ESP-IDF 6.0.1:**
  ```powershell
  $env:IDF_PYTHON_CHECK_CONSTRAINTS = "0"
  . C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1
  cd firmware; idf.py build; idf.py -p COM3 flash
  ```
  The constraints file is in `C:\Espressif\tools\`, hence the override. The
  build uses `-O2` (`sdkconfig.defaults`); a full rebuild takes several
  minutes.
- **Only one program may hold COM3:** Webots in ESP mode, the smoke test,
  the replay, flashing or the IDF monitor.
- **Python:** `.venv\Scripts\python.exe`. Webots is at
  `D:\apps\Webots\msys64\mingw64\bin\webots.exe`. The collectors run it
  headless (`--batch --mode=fast --no-rendering`).
- **Timing:** an ESP round trip takes 16-32 s of wall time. Training one conv
  model takes about 25 minutes on the CPU; `--reuse-weights` re-evaluates
  saved weights without training.
- **USB drops:** the ESP's USB serial link dropped a few times, including
  once when the laptop charger was moved. Runs that die with a
  `SerialException` are marked `serial_error` and repeated.
- **After retraining:** regenerate the weights header and reflash:
  `python -m scripts.models.export_c --model models\esp-conv-v1 --out firmware\main\anomaly_model.h`.

## Key files added or changed on 2026-11-26

- **Firmware:**
  - `main/main.cpp`: route speed `V@`, detector hook, 11-field motor frame,
    `Q` timing command, 12 KB task stack.
  - `main/anomaly.{h,cpp}`: forward pass and monitoring.
  - `main/anomaly_model.h`: generated weights.
  - `sdkconfig.defaults`: `-O2`.
- **Controller:** `epuck_route.py`:
  - sends `esp_speed_rad_s` as `V@`;
  - seeded `spawn_jitter`;
  - logs `esp_motor_score`, `esp_ir_score`, `esp_motor_alarm` and
    `esp_ir_alarm`.
- **Scripts:**
  - `telemetry/collect_route_runs.py`: `--control-mode`, `--round-trip`,
    `--seeds`, `--resume`, `--serial-retries`, and an ESP-aware health
    check.
  - `faults/collect_fault_runs.py`: matrix `mission`, `spawn_jitter` and
    per-run `seed`/`onset_s`; retries and `--resume`.
  - `telemetry/prepare_telemetry.py`: per-window monitoring flags.
  - `models/train_autoencoder.py`: `--threshold-rule` and
    `--reuse-weights`.
  - `faults/evaluate_faults.py`: applies the monitoring flags and skips
    `serial_error` runs.
  - New: `models/export_c.py` and `esp/replay_scores.py`.
  - `esp/esp_serial_smoke_test.py`: speed, anomaly and timing checks.
- **Data and models:**
  - `data/esp_runs/{healthy,faults,live-v1}`
  - `data/datasets/splits/esp-runs-v1.json` and
    `data/datasets/esp-baseline-v1`
  - `models/esp-conv-v1`
  - `reports/esp-fault-eval-v1`, `reports/esp-model-comparison-v1` and
    `reports/esp-replay-v1`

## Next steps (to wrap up)

1. **Commit** the session's work. Leave out the changed `__pycache__/*.pyc`
   files, which git tracks by mistake. Decide whether the run CSVs under
   `data/esp_runs` (tens of MB) go into git.
2. **Decide what an alarm does.** Two options:
   - *Report only:* the current behaviour, already finished.
   - *Safe stop:* on a persistent alarm the ESP stops with a new state such
     as `FAULT_STOP`, as it does for `LINE_LOST`. Add it to
     `ESP_STOP_STATES` in the controller, the smoke test and the docs, then
     verify with a few live fault runs. About an hour of work.
3. **Record a rendered demo:** open the world in Webots with
   `control_mode: esp32` and a mission `fault` block, and show the alarm
   columns or a console message when the ESP alarms.

Optional:
- **INT8 quantisation** as a comparison (float32 already fits easily).
- **Weak detections** (20% motor power loss, IR noise std 100 or less, IR
  stuck, IR drift). Try per-sensor max scoring or a sample-difference
  feature.
- **Track coverage:** ESP runs through the parallel edges (edge-002,
  edge-004).
- **Reconnect** for a single mission if COM3 drops.
- **Junction-miss policy:** whether `JUNCTION_MISSED`/`LINE_LOST` after a
  path-changing fault is the desired safe stop.

## Git state

- **Last commit:** `690cec8` ("chore: add build directory to .gitignore"),
  on top of `8a20f6b` (ESP PD control).
- **Uncommitted:** everything listed above.
- **`firmware/sdkconfig`** is gitignored. It was edited locally for `-O2`,
  and the committed `sdkconfig.defaults` carries the same setting for fresh
  builds.
