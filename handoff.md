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

Everything below is done and verified. The 2026-11-27 session (search
winners deployed, COM3 reconnect) is not committed yet; the user commits and
pushes.

| Piece | State | Evidence |
|---|---|---|
| Route following, round trips A-D | Done | Stops 0.2-1.4 cm from the node |
| Detectors on the ESP (esp-dsp, float32) | Done | ~0.81 ms per 32 ms frame (v1: 1.18 ms measured the same way); replay identical alarms, conv within 1.3e-5 (`reports/esp-replay-v3`) |
| Weak-motor and stuck-IR detection (motor GRU, stuck rule) | Done | Offline 82 -> 94/120 own-detector, 98 -> 110/120 either; no extra false alarm (`reports/esp-gru-eval-v1`) |
| Fault stop + notifications | Done | Live: weak motor 8/8, stuck IR 3/4 (4th unobservable), dead IR 2/2, every stop notified |
| Stuck rule needs the robot driving | Done | Fixes the stuck-on-line-value run that drove off the track (`fault-stop-v4`) |
| Pull-over side from the map (`P@`) | Done | Left on edge-005 Junction 2 -> 3, outside the loop (`pull-over-v1`) |
| On-ESP latency table | Done | 967 architectures (`reports/esp-latency-v1`) |
| INT8/INT16 benchmark | Done | ESP-DL ~2.5x slower than esp-dsp; INT8 breaks the motor conv model; float32 kept (`reports/esp-int8-v1`) |
| Optuna architecture search | Done | (`reports/arch-search-v1`) |
| Search winners deployed (`models/esp-conv-v2` motor + unchanged IR, `models/esp-gru-v2`) | Done | Offline 94/110 of 120 like v1, motor median alarm 0.16 -> 0.13 s (`reports/deployed-eval-v2`); live 16 runs as expected, 12/12 stops notified (`data/esp_runs/fault-stop-v5`) |
| COM3 reconnect (`EspLink` in `epuck_route`) | Done | Fake-port test; real unplug not tested |
| Second track (track2, `scripts/maps/make_track.py`) | Done | After the navigation fixes: healthy 8/8, faults 20/20 stopped and notified, base track still 16/16 and replay identical (`data/esp_runs/nav-fix-v1`, `track2-v2`; REPORT section 11) |
| Navigation fixes (stale junction pattern, blind arrival, `l`/`r` corners with spin, spins unmonitored, `<`/`>` bear onto Y branches) | Done | Smoke test 48 checks |
| Hard course (track3: 45° bends, Y junction, hairpin, tight curve) | Done | healthy 8/8, faults 16/16, longest false alarm 0.77 s; base and track2 regression 8/8 (`data/esp_runs/track3-v2`; REPORT section 11). Marginal: hairpin at 3.5 rad/s loses the line 0.74 s of the 0.75 s limit |
| Parallel edges (`avoid_edges`) + per-edge odometry scales | Done | edge-002/004 and standard routes 12/12, stops 0.1-1.5 cm (`data/esp_runs/parallel-edges-v2`); `replay_scores --per-edge-odometry` for new runs |
| Docs (REPORT.md, docs/, READMEs, requirements) | Done | |

## Open work (priority order)

1. **IR drift (4-6/8) and IR noise std <= 100 (0-3/8)** by the IR model. A
   one-step IR GRU does not help (it copies the slow ramp). Try a level check
   (on-line ~300 and floor ~760 readings shift with drift) or a cross-sensor
   model predicting each sensor from the other two plus motion. Evaluate in
   `scripts/models/gru_experiment.py` style against the same 120 runs.
2. **IR search candidate** (22/32 vs 17/32 held-out, +2/18 healthy test
   alarms): the user's choice. Deploying it is now only a config change in
   `models/esp-conv-v2` (per-model window/sizes) + export, replay, live.
3. **Real unplug test** of the COM3 reconnect.
4. **Healthy runs over edge-002** for training: the parallel edges are now
   driven (`avoid_edges`, per-edge odometry scales, `parallel-edges-v2`), but
   the motor conv model raises sub-0.3 s alarms at edge-002's turns (no stop).

5. **Hairpin margin at 3.5 rad/s** (track3): the line search nearly runs out
   (0.74 of 0.75 s). Options: a longer search, or slowing for tight curves.

## How the detectors work (Python and ESP identically)

- Motor sample: wheel targets (rounded to 3 decimals as sent) + encoder wheel
  speeds; IR sample: three ground sensors; scaled with the training scaler.
- Conv AE: reconstruction MSE of the last window; motor window 10 (8
  channels, threshold 0.0861), IR window 20 (16 channels, threshold 0.4476).
- Motor GRU (16 hidden): predicts both wheel speeds from the targets, state
  from the route start; score = worst wheel's mean squared error over 31
  samples; threshold 0.0285 (2x the no-validation-alarm value).
- Stuck rule: a sensor's change < 0.5 between every pair of the last 32
  frames while mean wheel speed > 1 rad/s.
- Not monitored: windows starting in the first 0.5 s, or overlapping
  `AT_DEST`, `UNLOADING`, `U_TURN`, `FAULT_PULLOVER`, `FAULT_STOP`, or the
  first 0.5 s after a U-turn. Alarm: 3 of the last 5 windows. Motor alarm =
  conv or GRU; IR alarm = conv or stuck.
- The ESP resets all detectors (and the GRU state) on each route command, so
  the return leg starts fresh; the replay reference does the same per leg.
- Spins in place (U-turn, corner turn) and the 0.5 s after them are not
  monitored.

## Serial protocol additions this session

- `M` frame: 14 fields (`...,<motor_score>,<ir_score>,<motor_alarm>,<ir_alarm>,<gru_score>,<gru_alarm>,<stuck_alarm>`).
- `Q` -> `I,` with last/max microseconds for motor conv, IR conv, motor GRU.
- `B,C,...` / `B,G,...` latency benchmark (firmware/README.md).
- Route option `P@<R|L>...`, one side per step like `E@`.
- Junction actions `l`/`r` (corner: spin onto a crossing line) and `<`/`>`
  (bear onto a Y branch), chosen by the host from the map.
- Mission keys: `map_path`, `routes` (per-track route table), `avoid_edges`,
  `esp_odometry_scale_by_edge`, `esp_sharp_turn_deg` (85).

## Verified at the end of the session

- Unit tests: `.\.venv\Scripts\python.exe -m unittest discover -s tests` (47).
- ESP smoke test (`scripts.esp.esp_serial_smoke_test --port COM3`): 48 checks
  pass, incl. corner, bear, stale junction and blind arrival (its fault stops
  send real notifications).
- Replay: 10 runs, identical wheel targets and alarms (`reports/esp-replay-v4`).
- A fresh export of the staged repo (as a clone) passes the tests, builds the
  firmware from scratch and drives all three tracks with the ESP.
- The final AGV firmware is flashed on the ESP; WiFi connected.

## Repository contents

Raw run logs (`route-*.csv`, `controller-errors.log`, `data/SensorData*.csv`) are
git-ignored and not in the repository; the manifests, the prepared datasets
(`data/datasets`), the models and all reports are. Replaying recorded runs
(`replay_scores`), the offline fault evaluation and GRU/search retraining
need the raw logs; driving, detecting, exporting and conv training do not.

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
  `python -m scripts.models.export_c` (defaults: `models\esp-conv-v2`, `models\esp-gru-v2`), reflash, replay.
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
