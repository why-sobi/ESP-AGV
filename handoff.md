# Handoff: ESP-AGV (single car), 2026-11-27

## What the project is

A simulated AGV (Webots e-puck) follows a line track between named nodes. An
ESP32-S3 on USB serial (COM3) is the robot's onboard computer:

1. The Supervisor controller (`simulation/controllers/epuck_route`) plans a
   route on the track graph and sends the ESP one route command per leg: a
   turn action per junction tagged with its expected odometry distance, plus
   options (speed `V@`, fault-stop hold `F@`, place labels `E@`, pull-over
   sides `P@`).
2. Every 32 ms Webots sends the three ground-IR readings and both encoder
   positions.
3. The ESP runs the PD line follower, junction/destination logic, and four
   anomaly detectors on esp-dsp: motor conv autoencoder, motor GRU, IR conv
   autoencoder, stuck-sensor rule. It returns wheel targets, state, scores
   and alarms.
4. On a persistent alarm it pulls over to the map-chosen side, stops, and
   sends a phone notification over WiFi (ntfy).

Faults are injected in Webots. Scope is one car; Webots stays the
environment.

**Read `REPORT.md` first**: it has the whole history, every decision and
every number. `docs/DECISIONS.md` (decisions 1-20), `docs/STATUS.md`,
`docs/TODO.md` and `docs/USAGE.md` (all commands) are up to date as of this
handoff.

## Where it stands

Everything below is done and verified. Nothing from 2026-11-26/27 is
committed yet; the user commits and pushes. 26 tracked `.pyc` files are
staged for removal (`git rm --cached`).

| Piece | State | Evidence |
|---|---|---|
| Route following, round trips A-D | Done | Stops 0.2-1.4 cm from the node |
| Detectors on the ESP (esp-dsp, float32) | Done | ~1.05 ms per 32 ms frame; replay within 5e-5, identical alarms (`reports/esp-replay-v2`) |
| Weak-motor and stuck-IR detection (motor GRU, stuck rule) | Done | Offline 82 -> 94/120 own-detector, 98 -> 110/120 either; no extra false alarm (`reports/esp-gru-eval-v1`) |
| Fault stop + notifications | Done | Live: weak motor 8/8, stuck IR 3/4 (4th unobservable), dead IR 2/2, every stop notified |
| Stuck rule needs the robot driving | Done | Fixes the stuck-on-line-value run that drove off the track (`fault-stop-v4`) |
| Pull-over side from the map (`P@`) | Done | Left on edge-005 Junction 2 -> 3, outside the loop (`pull-over-v1`) |
| On-ESP latency table | Done | 967 architectures (`reports/esp-latency-v1`) |
| INT8/INT16 benchmark | Done | ESP-DL ~2.5x slower than esp-dsp; INT8 breaks the motor conv model; float32 kept (`reports/esp-int8-v1`) |
| Optuna architecture search | Done, not deployed | Smaller motor conv and GRU beat the deployed ones on held-out data (`reports/arch-search-v1`) |
| Docs (REPORT.md, docs/, READMEs, requirements) | Done | |

## Open work (priority order)

1. **IR drift (4-6/8) and IR noise std <= 100 (0-3/8)** by the IR model. A
   one-step IR GRU does not help (it copies the slow ramp). Try a level check
   (on-line ~300 and floor ~760 readings shift with drift) or a cross-sensor
   model predicting each sensor from the other two plus motion. Evaluate in
   `scripts/models/gru_experiment.py` style against the same 120 runs.
2. **Deploy the search winners**: motor conv candidate 0 (window 10, 8
   channels, bottleneck 12, hidden 16: held-out 28/28 vs 25/28, 100 vs 307
   us) and GRU candidate 0 (16 hidden, 31-sample score window: 28/28 vs 27/28,
   127 vs 267 us). Weights: `reports/arch-search-v1/<model>/final-0/weights.pt`.
   The IR candidate (22/32 vs 17/32) adds 2/18 healthy test alarms: a choice
   for the user. Needed: configurable window/channels (and kernel) in
   `export_c.py` + `anomaly.cpp` (currently `kWindow = 20`, `kChannels =
   16`), save a proper model dir + config (thresholds from the final run),
   replay, live runs, docs. The search's `ConvAE` has the same layout as the
   deployed `ConvAutoencoder` with a kernel parameter.
3. **COM3 drop mid-run** fails the run (the ESP stops on `LINK_LOST`); add
   reconnect for a single mission.
4. **Parallel edges** edge-002/edge-004 never driven under ESP control.

## How the detectors work (Python and ESP identically)

- Motor sample: wheel targets (rounded to 3 decimals as sent) + encoder wheel
  speeds; IR sample: three ground sensors; scaled with the training scaler.
- Conv AE: reconstruction MSE of the last 20 samples; thresholds motor 0.1894,
  IR 0.4476.
- Motor GRU (32 hidden): predicts both wheel speeds from the targets, state
  from the route start; score = worst wheel's mean squared error over 62
  samples; threshold 0.0246 (2x the no-validation-alarm value).
- Stuck rule: a sensor's change < 0.5 between every pair of the last 32
  frames while mean wheel speed > 1 rad/s.
- Not monitored: windows starting in the first 0.5 s, or overlapping
  `AT_DEST`, `UNLOADING`, `U_TURN`, `FAULT_PULLOVER`, `FAULT_STOP`, or the
  first 0.5 s after a U-turn. Alarm: 3 of the last 5 windows. Motor alarm =
  conv or GRU; IR alarm = conv or stuck.
- The ESP resets all detectors (and the GRU state) on each route command, so
  the return leg starts fresh; the replay reference does the same per leg.

## Serial protocol additions this session

- `M` frame: 14 fields (`...,<motor_score>,<ir_score>,<motor_alarm>,<ir_alarm>,<gru_score>,<gru_alarm>,<stuck_alarm>`).
- `Q` -> `I,` with last/max microseconds for motor conv, IR conv, motor GRU.
- `B,C,...` / `B,G,...` latency benchmark (firmware/README.md).
- Route option `P@<R|L>...`, one side per step like `E@`.

## Verified at the end of the session

- Unit tests: `.\.venv\Scripts\python.exe -m unittest discover -s tests` (46).
- ESP smoke test (`scripts.esp.esp_serial_smoke_test --port COM3`): passes,
  incl. left pull-over and `P@` validation (its fault stops send real
  notifications).
- Replay: 10 runs, 0 alarm mismatches.
- The AGV firmware is flashed on the ESP (the ESP-DL bench app was flashed
  temporarily and replaced); WiFi connected.

## Environment notes

- **ESP-IDF 6.0.1:**
  ```powershell
  $env:IDF_PYTHON_CHECK_CONSTRAINTS = "0"
  . C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1
  cd firmware; idf.py build; idf.py -p COM3 flash
  ```
  esp-dsp comes from the component manager (`main/idf_component.yml`,
  fetched into `managed_components/` on the first build).
- **After retraining:** `python -m scripts.models.gru_experiment --reuse-weights`
  (writes the GRU `deploy` block), then
  `python -m scripts.models.export_c --model models\esp-conv-v1 --gru models\esp-gru-v1 --out firmware\main\anomaly_model.h`, reflash, replay.
- **esp-ppq** needs Python 3.11-3.12: separate `.venv-ppq` (uv, git-ignored).
- **ESP-DL bench app** (`firmware/esp-dl-bench`) replaces the AGV firmware and
  its partition table when flashed; reflash `firmware/` afterwards (the
  `wificfg` data survives). Its first build takes ~10 min.
- **Only one program may hold COM3.** USB drops: unplug/replug; runs that die
  with `SerialException` are marked `serial_error` and repeated.
- **Timing:** an ESP round trip takes 16-36 s wall time; a conv search trial
  3-6 min, a GRU trial 10-20 min on this CPU; full conv training ~10-25 min.
- **ntfy from the laptop:** if `curl https://ntfy.sh/...` returns HTTP 000,
  add `--resolve ntfy.sh:443:159.203.148.75`. Topic `agv-sobi-darwmq3i`;
  recent messages: `curl -s "https://ntfy.sh/agv-sobi-darwmq3i/json?poll=1&since=30m"`.
- **Windows shell:** Python one-liners through bash mangle backslashes (`\a`
  became a bell character once); write edit scripts to files.
