# Project Usage

Run commands from the repository root in PowerShell unless a section says
otherwise. This project is simulation-first: Webots supplies the robot,
environment, sensors, and encoder readings. The ESP32-S3 is an external
computation target connected by USB serial; this is **not** a physical AGV
workflow.

## Quick start (fresh clone with an ESP32-S3)

Needs Webots R2025a, Python 3.12+ and ESP-IDF 6.0.1; the sections below have
the details of each step.

1. **Python:** `python -m venv .venv`, `.\.venv\Scripts\Activate.ps1`,
   `pip install -r requirements.txt`. Point both controllers' `runtime.ini` at
   `.venv\Scripts\python.exe` ([Set up Python and Webots](#set-up-python-and-webots)).
2. **Firmware:** in an ESP-IDF 6.0.1 shell, `cd firmware`, `idf.py build`,
   `idf.py -p COM3 flash` (the first build downloads esp-dsp)
   ([ESP32-S3 firmware](#esp32-s3-firmware-and-serial-smoke-test)).
3. **Check the ESP:** `.\.venv\Scripts\python.exe -m scripts.esp.esp_serial_smoke_test --port COM3`
   (48 checks; its fault stops send notifications once WiFi is set).
4. **WiFi and phone notifications (optional):** copy `firmware\wifi.example.csv`
   to `firmware\wifi.csv` (git-ignored), set the network and an ntfy topic,
   run `.\scripts\esp\flash_wifi.ps1`, subscribe to the topic in the ntfy app
   ([Fault stop and phone notifications](#fault-stop-and-phone-notifications)).
5. **Drive it:** in `maps\routes\mission.json` set `control_mode` to `esp32`,
   `esp_serial_port` to the ESP's port, the start/destination nodes, and
   optionally `return_to_start: true` and `esp_fault_stop_s: 2.0`; open
   `simulation\worlds\Self Diagnosis AGV.wbt` and run it
   ([Let the ESP32-S3 control the Webots e-puck](#let-the-esp32-s3-control-the-webots-e-puck)).
6. **Headless runs and faults:** `scripts.telemetry.collect_route_runs`
   (healthy) and `scripts.faults.collect_fault_runs` (fault matrix) run Webots
   without the GUI ([Collect telemetry](#collect-telemetry),
   [Fault stop](#fault-stop-and-phone-notifications)).
7. **Other tracks:** `... track2.wbt` / `... track3.wbt` with
   `--mission maps\routes\mission-track2.json` / `mission-track3.json`
   ([Other tracks](#other-tracks-track2-track3)).

Only one program may hold the ESP's COM port at a time (close the IDF
monitor). Raw run logs are git-ignored: a clone has the models, maps, worlds,
manifests, prepared datasets and reports, but not the recorded `route-*.csv`
runs that `replay_scores` and the offline evaluations read.

## Project files and what they do

| Path | Purpose |
|---|---|
| `simulation/worlds/Self Diagnosis AGV.wbt` | Webots world, track texture, robot, and selected controller (`... track2.wbt`, `... track3.wbt`: the other tracks) |
| `simulation/controllers/epuck_route/` | Map route execution in Webots; uses Supervisor pose and ground-sensor correction |
| `simulation/controllers/epuck_data/` | Sensor-only PD line follower and per-session telemetry collector |
| `maps/image/` | Track images used by the world and map-conversion tools |
| `maps/paths/` | Editable graph-map JSON files |
| `maps/visualizations/` | Generated standalone HTML map previews |
| `maps/routes/mission.json` | Webots route mission, speed, ESP and fault-stop settings, per-edge odometry scales (`mission-track2.json`, `mission-track3.json`: the other tracks, with their map and route table) |
| `scripts/maps/` | Track-image conversion, map visualization, graph route planning, test-track generation (`make_track.py`) |
| `scripts/telemetry/` | Healthy Webots run collection and telemetry preparation |
| `scripts/models/` | Autoencoder training, GRU experiment, weight export, on-ESP latency sweep, INT8 quantization and architecture search |
| `scripts/faults/` | Fault-run collection and detection evaluation |
| `scripts/esp/` | Route-to-ESP conversion and pull-over sides, serial smoke test, replay agreement, WiFi tools, INT8 benchmark |
| `tests/` | Automated Python regression tests |
| `notebooks/` | Exploratory analysis notebooks |
| `models/` | Trained weights, thresholds, scalers and evaluation results |
| `data/fault_runs/` | Fault matrix and manifest of the Supervisor-era fault runs |
| `data/esp_runs/` | ESP-controlled healthy, fault, live and test-track runs (run CSVs are local only, git-ignored) |
| `reports/` | Fault-evaluation summaries, results and plots |
| `firmware/` | ESP-IDF project: control, anomaly detectors (esp-dsp), fault stop, notifications; `firmware/esp-dl-bench/` is a separate ESP-DL benchmark app |
| `data/route_runs/` | Per-run telemetry from the map-based Webots controller |
| `data/sensor_runs/` | Per-session telemetry from the sensor-only PD controller |
| `data/datasets/` | Prepared model-input arrays and dataset metadata |
| `REPORT.md` | The whole project: what was done, why, and every result |
| `docs/STATUS.md`, `docs/TODO.md`, `docs/DECISIONS.md` | Verified status, next work, and design history |

Build output, local ESP-IDF configuration, Python caches, and installed Python
environments are not project source files.

## Set up Python and Webots

Create/activate the virtual environment and install the repository dependencies:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

Confirm both controller `runtime.ini` files point to this repository's
`.venv\Scripts\python.exe`. Open
`simulation\worlds\Self Diagnosis AGV.wbt` in Webots and start the simulation.
The committed world selects `epuck_route`.

## Make a map from a new track image

1. Put the image in `maps\image` (or another chosen image folder). The
   converter expects a dark path against a lighter background.
2. Convert it to a draft JSON map; the JSON takes the image basename:

   ```powershell
   .\.venv\Scripts\python.exe -m scripts.maps.pathToMap `
     -img "maps\image\new-track.png" `
     -out "maps\paths"
   ```

3. Generate and open the HTML preview:

   ```powershell
   .\.venv\Scripts\python.exe -m scripts.maps.visualizeMap `
     -map "maps\paths\new-track.json"
   ```

   By default the preview is `maps\visualizations\new-track.html`. Zoom, pan,
   inspect the candidate graph, and hand-edit the JSON to correct node locations,
   edge connections, directionality, names, and geometry. Rerun the preview
   after edits.

4. To run that track in Webots, update the arena `ImageTexture.url` in
   `simulation\worlds\Self Diagnosis AGV.wbt` to the new image and make the
   arena `floorSize` match the real proportions. Review the map's
   `world_transform` so pixel coordinates align with that arena, then visually
   verify robot alignment. Image conversion does not automatically survey or
   calibrate the world transform.
5. Update the map path constants near the top of
   `simulation\controllers\epuck_route\epuck_route.py` to point to the new JSON.
   Edit `maps\routes\mission.json` for start/destination nodes and test the
   planned route. Do not rely on automatically generated IDs until the graph
   has been reviewed.

Useful converter options include `--threshold`, `--max-dimension`, and
`--pixels-per-meter`. Only provide a pixels-per-meter scale when measured; a
guessed scale makes distances and world alignment untrustworthy.

## Plan and run a route

Use the exact node IDs from the reviewed map:

```powershell
.\.venv\Scripts\python.exe -m scripts.maps.route_planner `
  "maps\paths\base track.json" node-003 node-001 `
  -out "maps\routes\active_route.json"
```

Edit `maps\routes\mission.json` to select `start_node`,
`destination_node`, and `wheel_speed_rad_s`. Then run the world in Webots. The
current route controller plans a shortest-distance path, uses Supervisor pose
to track map geometry, and uses the three ground sensors for local correction
and line-loss stopping. It writes one CSV per run to `data\route_runs`.

Supervisor pose is ground-truth simulation information, not an ESP input
mechanism. The ESP receives only relative junction actions with odometry
distances, plus the ground-sensor and encoder readings; the ESP-control mode
below converts the planned graph route to that form.

### Run one route on any track (headless)

Pick the track and the two nodes; nothing is edited and no Webots window
opens:

```powershell
.\.venv\Scripts\python.exe -m scripts.telemetry.collect_route_runs `
  --track track2 --start node-001 --destination node-004 --speeds 3.0 `
  --control-mode esp32 --round-trip --out data\route_runs\try --timeout 300
```

- `--track`: `base` (default), `track2` or `track3`; it selects the track's
  mission (`maps\routes\mission*.json`) and world. Every track has the nodes
  `node-001` to `node-004` (Junction 1-4); a wrong node lists the valid ones.
- Leave out `--control-mode esp32 --round-trip` for a one-way Supervisor run
  (no ESP needed). `--avoid-edges edge-005` forces another path, and
  `--speeds 2.5 3.5` runs the route once per speed.
- The console prints `healthy` or `failed` with the line-loss and path-error
  checks; the run CSV (`route-<time>-X-<speed>.csv`) and `manifest.csv` go to
  `--out`. Fault stops are off in these runs; use `collect_fault_runs` to
  test faults.
- To watch it, use the demo launcher below, or set the same nodes in `maps\routes\mission.json`
  (`start_node`, `destination_node`, `control_mode`, and `map_path` for
  track2/track3), open the matching `simulation\worlds\*.wbt` and press
  Reset (Ctrl+Shift+T).

### Demo: watch a clean or faulty run in Webots

`scripts.run_demo` opens the Webots window (real time) on the chosen track
and drives one route with the ESP, by default as a round trip with fault
stops on. Nothing is edited; Reset (Ctrl+Shift+T) replays the same run, and
closing the window ends the script.

```powershell
# Clean run
.\.venv\Scripts\python.exe -m scripts.run_demo --track track2 --start node-001 --destination node-004
# Weak right motor from 6 s: pulls over, stops, phone notification
.\.venv\Scripts\python.exe -m scripts.run_demo --track track2 --start node-001 --destination node-004 --fault weak-motor
```

- `--fault`: `weak-motor`, `stall`, `intermittent`, `encoder-stuck`,
  `encoder-noise`, `ir-stuck`, `ir-dead`, `ir-noise`, `ir-drift`, with the
  ESP fault matrix's targets and severities. `--target` and `--severity`
  override them; `--onset` sets the start time (default 6 s).
- A fault stop and its notification need an alarm that lasts 2 s before the
  route ends, so give the fault time: an early `--onset` on short routes
  (node-001 to node-004 arrives at ~10 s). For a demo, `weak-motor` and
  `ir-dead` stop within about a second of the alarm. `ir-noise` (std 200)
  can flicker on and off without stopping on some routes (seen on
  node-001 to node-004); `--severity 300` stopped there. `stall` and
  `intermittent` often stop on line loss first, and `ir-drift` is often not
  detected.
- `--speed` (default 3.0 rad/s), `--seed <n>` for start-pose jitter,
  `--one-way`, `--no-fault-stop` (alarms only), `--supervisor` (no ESP; one
  way only).
- The Webots console prints a timeline: `FAULT ON`, `ESP motor/IR alarm
  ON`/`off` and each ESP state (`FAULT_PULLOVER`, `FAULT_STOP`, ...). The
  alarm goes off during the pull-over because manoeuvres are not monitored.
- To show the whole chain (fault, its effect, detection, pull-over,
  notification), `ir-dead` is the clearest: the robot swerves visibly (up to
  6 cm off the line, against 0.2 cm before), the IR alarm follows ~0.25 s
  after the onset and the pull-over 2 s later. `weak-motor` is caught in
  ~0.1 s but barely changes the path, because the ESP's steering
  compensates by slowing the other wheel; `--severity 0.3` (30% of the
  commanded speed) makes that visible as a slowdown (6.0 to 2.1 cm/s on
  node-001 to node-004 at 3 rad/s), still within 0.7 cm of the line.
- `--fast`: the window runs as fast as the ESP allows (about 3-4x real
  time, like the >> button) instead of real time. `--headless`: no window;
  prints how the run ended, e.g. `ended FAULT_STOP`, `first motor alarm at
  6.11 s (0.11 s after the onset)`. Real time is what makes the window slow:
  Webots waits for the clock, while rendering itself costs little.
- On track3 a clean run can show a brief motor alarm (up to 0.77 s on that
  unseen course); only an alarm lasting 2 s stops the robot.
- Run CSVs go to `data\route_runs\demo` (`--out`).

### Let the ESP32-S3 control the Webots e-puck

Install the requirements so the Webots Python environment has `pyserial`:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Connect the ESP32-S3, close the IDF monitor so COM3 is free, and set
`control_mode` to `esp32` in `maps\routes\mission.json`. Run the world with
`epuck_route` selected. The Supervisor still plans the route, places the robot
at the start, tracks route progress, enforces path safety, and logs telemetry.
The default `supervisor` mode preserves the original Supervisor-controlled
run. `AGV_CONTROL_MODE=esp32` can override the mission setting for a single
Webots launch.

In ESP mode:

1. The Supervisor sends the route once, as relative junction actions with
   the expected wheel-odometry distance to each junction and to the
   destination, e.g. `R@0.265,D@0.779` for node-003 to node-001.
2. Every 32 ms step it sends the three ground-sensor readings and both wheel
   encoder positions.
3. The ESP runs the PD loop, counts distance from the encoders, takes each
   junction action only when the junction pattern appears within +/-0.10 m of
   its distance, and stops when it reaches the destination distance. Webots
   applies the returned wheel targets.

The distances are the map centerline distances multiplied by each edge's
scale in `esp_odometry_scale_by_edge`, or `esp_odometry_scale` (default
0.92) for edges not listed. The robot cuts inside curves, so its odometry
reads less than the drawn centerline: ~0.973 on the long edges, ~0.836 on
the curved loop (fitted on 237 ESP legs; REPORT.md section 11). 0.92 is the
average on the standard routes, which combine one of each.

`avoid_edges` (a list of edge ids) plans without those edges, to drive the
other edge of a parallel pair; the collector takes it as `--avoid-edges`:

```powershell
# edge-004
.\.venv\Scripts\python.exe -m scripts.telemetry.collect_route_runs --routes BD --speeds 2.5 3.5 --control-mode esp32 --round-trip --avoid-edges edge-005 --out data\esp_runs\parallel-edges-v2\edge-004
# edge-002
.\.venv\Scripts\python.exe -m scripts.telemetry.collect_route_runs --routes AC --speeds 2.5 3.5 --control-mode esp32 --round-trip --avoid-edges edge-003 edge-004 edge-005 --out data\esp_runs\parallel-edges-v2\edge-002
```

`replay_scores` rebuilds route commands with the global scale and plain
turns, as the runs recorded before 2026-11-27 15:00 were; pass
`--per-edge-odometry` for later runs (per-edge scales and `l`/`r` corners;
it does not apply `avoid_edges` or another track's map).

To calibrate the scales on another track, record a few healthy round trips and
take each edge's ratio of ESP odometry to map length along the middle of the
edge (how track3's `esp_odometry_scale_by_edge` was set; REPORT.md section
11). The firmware's protocol and behavior are described in
`firmware\README.md`.

The Supervisor no longer ends an ESP run itself when the robot reaches the
destination node. It waits for the ESP's `AT_DEST` and marks the run completed
only if the robot stopped within `esp_destination_tolerance_m` (default
0.05 m) of the node. ESP-reported `LINE_LOST`, `JUNCTION_MISSED`,
`FAULT_STOP` (see "Fault stop and phone notifications"), a serial timeout, or
a malformed response stops the motors and fails the run.
ESP-controlled runs add `esp_state`, `esp_route_index` and `esp_distance_m`
to their telemetry CSVs.

**Round trip.** Set `return_to_start` to `true` (ESP mode only) to come back
after the destination:

1. The robot stops at the destination and waits `unload_s` (default 1 s).
   Those rows are logged with `esp_state` `UNLOADING`.
2. The Supervisor plans the destination-to-start route and sends it to the
   ESP prefixed with a U-turn, e.g. `U@0.000,L@0.514,D@0.779`.
3. The ESP spins left in place until it has turned at least 160 degrees and
   is back on the line, then follows the return route.

The run completes when the ESP stops within `esp_destination_tolerance_m` of
the start node. Return-leg rows have the swapped `route_start` and
`route_destination`. The Supervisor's line-loss timer ignores the spin.

A headless ESP run takes about 10-15 s of wall time (about 18 s for a round
trip), because every simulation
step waits for the ESP's serial reply (about 8 ms per round trip). If the
controller raises an error, the traceback is appended to
`controller-errors.log` in the log directory, because Webots drops the
controller's last console lines when it quits.

The serial smoke test remains a standalone protocol check; it does not launch
Webots.

## Collect telemetry

### Map-based route runs

The configured `epuck_route` controller automatically writes a timestamped CSV
under `data\route_runs` when a route executes. Columns include route/edge IDs,
pose, centerline error, target motor speeds, wheel encoder positions, and all
three ground IR readings. The route controller follows map geometry using
Supervisor pose in its default mode, so those runs are useful for simulation
baselines but do not test the ESP control loop. Use `control_mode: "esp32"` for
ESP-controlled runs.

**Headless batch recording (preferred).** Record a route x speed matrix
without opening the Webots window:

```powershell
.\.venv\Scripts\python.exe -m scripts.telemetry.collect_route_runs `
  --routes ABCD `
  --speeds 2.0 2.5 3.0 3.5
```

Routes are A (node-003 to node-001), B (node-001 to node-003), C (node-002
to node-004) and D (node-004 to node-002). Each run takes about 2 s of wall
time. The script writes a temporary mission per run, starts Webots with
`--batch --mode=fast --no-rendering`, and points the controller at the
temporary mission through `AGV_MISSION_PATH`, `AGV_ROUTE_PATH` and
`AGV_LOG_DIR`. The committed `mission.json` is never edited. Files are named
`route-<timestamp>-<route>-<speed>.csv`. Each run is checked for completion,
line loss after 0.5 s, irregular sampling and path error, then appended to
`data\route_runs\manifest.csv` as `healthy` or `failed`. Completion comes from
the Webots exit code, because Webots drops the controller's last console lines
when it quits. A Webots window that is already open does not interfere; the
batch instance uses another port.

**ESP-controlled runs (the deployment dataset).** With the ESP on COM3 and no
other program holding the port:

```powershell
.\.venv\Scripts\python.exe -m scripts.telemetry.collect_route_runs `
  --control-mode esp32 --round-trip --routes ABCD `
  --speeds 2.0 2.25 2.5 2.75 3.0 3.25 3.5 3.75 4.0 --seeds 0 1 2 `
  --out data\esp_runs\healthy --timeout 300 --resume
```

- The speed is sent to the ESP as its route speed (`V@`, mission key
  `esp_speed_rad_s`).
- `--round-trip` records the outbound leg, the 1 s unload, the U-turn and
  the return leg in one CSV.
- `--seeds` repeats each route/speed with a seeded start-pose jitter
  (`--jitter-lateral-m`, default 3 mm; `--jitter-heading-deg`, default 4),
  because Webots otherwise repeats a run byte for byte.
- Each round trip takes 16-32 s of wall time; the 108-run matrix above took
  about 40 minutes.
- If the USB serial link drops mid-run (`SerialException` in
  `controller-errors.log`), the run is marked `serial_error` and repeated
  (`--serial-retries`, default 3). `--resume` skips runs already healthy.
- An ESP run is healthy when it completes and no line loss lasts longer than
  the firmware's 750 ms search limit. The sensors briefly leave the line on
  corners and in the last few centimetres into a terminal node (node-001,
  node-003), which is normal.

**Manual recording.** Edit `maps\routes\mission.json`, then in Webots press
Reset (Ctrl+Shift+T) so the controller restarts and rereads the mission, and
run in Fast mode. Wait for `Destination reached` in the console.

Webots is deterministic, so repeating the same route and speed produces a
byte-identical duplicate; vary the route or speed instead. If the ground
sensors lose the line for more than `max_line_loss_s` (default 2 s), the
controller prints `RUN FAILED` and stops logging. Do not use such runs as
healthy data.

### Sensor-only PD runs

Select `epuck_data` as the e-puck controller in Webots, then run one simulation
session per recording. It writes a new timestamped CSV to `data\sensor_runs`;
it does not append another simulation-time episode to a shared filename. The
logged features are target wheel speeds, wheel encoder positions, and the
three ground sensors.

Use separate healthy runs and label/keep fault-injected runs separately. Before
calling a run healthy, inspect whether the robot completed the intended track,
sampling cadence, sensor loss, and command ranges.

## Prepare motor and IR autoencoder inputs

The current dataset is built from an explicit split file, which assigns each
healthy run to train, validation, or test:

```powershell
.\.venv\Scripts\python.exe -m scripts.telemetry.prepare_telemetry `
  --split "data\datasets\splits\route-runs-v3.json" `
  --out "data\datasets\sim-baseline-v4" `
  --window 20 `
  --stride 1
```

Split files list run CSVs under `train`, `validation`, and `test`; relative
paths resolve from the repository root, and a file may appear in only one
split. `data\route_runs\manifest.csv` records the route, speed, and quality
notes for each run.

`route-runs-v3.json` (dataset `sim-baseline-v4`) uses 64 healthy runs: 4
routes x 16 speeds from 2.0 to 3.5 rad/s in 0.1 steps. Per route, alternate
interior speeds are held out, so every validation or test run has training runs
at +/-0.1 rad/s on the same route and direction. The split is 36 train, 14
validation and 14 test runs. The file's `speeds_rad_s` field lists the exact
assignment. `route-runs-v1.json` / `sim-baseline-v2` is the earlier 16-run
version recorded before the junction log-gap fix; those runs are marked
`superseded` in the manifest.

**ESP dataset.** `esp-runs-v1.json` (dataset `esp-baseline-v1`) uses the 108
healthy ESP round trips in `data\esp_runs\healthy`: 4 routes x 9 ESP speeds
(2.0-4.0 rad/s in 0.25 steps) x 3 start-pose seeds.
- Seeds 0 and 1 train (72 runs).
- Seed 2 alternates between validation and test over the speeds, with the
  phase flipped per route (18 + 18 runs).
- Every held-out run therefore has two training runs on the same route and
  speed from different start poses.
- That gives 74562 training, 18644 validation and 18640 test windows.

These are the models to deploy: the Supervisor-trained models raise alarms on
every healthy ESP run (see `docs/STATUS.md`).

```powershell
.\.venv\Scripts\python.exe -m scripts.telemetry.prepare_telemetry `
  --split "data\datasets\splits\esp-runs-v1.json" `
  --out "data\datasets\esp-baseline-v1"
```

The older `--runs` form still works and splits by run-id order with
`--validation-fraction` (no test split):

```powershell
.\.venv\Scripts\python.exe -m scripts.telemetry.prepare_telemetry `
  --runs `
    "data\route_runs\route-20261112-232535.csv" `
    "data\route_runs\route-20261112-232818.csv" `
  --out "data\datasets\sim-baseline-v1" `
  --window 20 `
  --stride 1
```

Replace those example paths with your healthy run files. The script:

- derives actual wheel angular velocities from encoder-position differences;
- builds independent temporal windows for motor and IR features;
- detects time resets in old appended files and splits those into episodes;
- rejects short episodes and exact duplicate telemetry runs;
- assigns independent runs to training and validation before windowing; and
- fits motor and IR scaling statistics on training runs only.

The output contains:

- `windows.npz`: `motor_*` and `ir_*` arrays for `train`, `validation`, and
  `test`, with shape `[windows, time, features]` (test arrays are empty
  without an explicit split);
- `metadata.json`: feature names, split method/source, per-split runs,
  sampling/sensor-quality summaries, scaler parameters, and exclusions.

At least two non-duplicate healthy runs are needed to create both training and
validation sets. Do not treat overlapping windows from the same run as
independent train/validation/test data. `sim-baseline-v4` has 15474 training,
5933 validation and 5929 test windows, all from healthy runs.
`sim-baseline-v1` is the earlier two-run pilot.

## ESP32-S3 firmware and serial smoke test

The ESP32-S3 N16R8 has an ESP-IDF controller project under `firmware`. Use
the VS Code ESP-IDF terminal configured for ESP-IDF 6.0.1, or activate it in
PowerShell:

```powershell
$env:IDF_PYTHON_CHECK_CONSTRAINTS = "0"
. C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1
cd firmware
idf.py build
idf.py -p COM3 flash
```

The constraints override is needed because this installation keeps
`espidf.constraints.v6.0.txt` in `C:\Espressif\tools\`, not where the
dependency check expects it.

Flashing is optional for building and overwrites firmware on the connected ESP.
Close the IDF monitor or any other process that owns COM3 before flashing or
running another serial test.

To check the flashed prototype from the repository root:

```powershell
.\.venv\Scripts\python.exe -m scripts.esp.esp_serial_smoke_test --port COM3
```

The script sends short distance-tagged routes and synthetic IR and encoder
frames, then checks PD wheel-speed outputs, reported odometry, that a junction
pattern is ignored outside its distance window and taken inside it, junction
clearing, the destination stop, the missed-junction stop, the U-turn (spin,
handover to line following, and giving up when no line is found), the route
speed, anomaly scores and inference timing, the fault stop (pull-over, stop,
and the time limit for a drive that cannot turn), the WiFi status, and
malformed-route rejection. It does **not** launch Webots; the data is synthetic and the
returned wheel commands are not applied to the simulated robot. If WiFi is
configured, its two fault stops each send a phone notification.

See `firmware\README.md` for the serial frame format and firmware limitations.

### Fault stop and phone notifications

When `esp_fault_stop_s` is set in `maps\routes\mission.json` (2.0 by default),
the ESP acts on a persistent anomaly alarm:

1. Either model's alarm (3 of the last 5 windows over threshold) has stayed on
   for `esp_fault_stop_s` seconds. Shorter alarms are only reported.
2. The ESP spins about 90 degrees in place, drives 8 cm forward off the
   line (`FAULT_PULLOVER`), and stops (`FAULT_STOP`). It turns to the side
   the route's `P@` option gives for the current step: `epuck_route` checks,
   along each edge, the pull-over path on both sides against the other track
   lines and the arena walls (`scripts/esp/route_actions.py`,
   `pull_over_sides`) and picks the right unless the right comes within 3 cm
   of a line or wall and the left is clearer. On the base track that means
   the left on edge-005 from Junction 2 to Junction 3, where a right turn
   ends on edge-004 inside the small loop; right everywhere else. If the turn and drive
   are not finished within 6 s, for example because a motor is stalled, it
   stops where it is.
3. The ESP itself sends a push notification over WiFi through
   [ntfy](https://ntfy.sh), in plain words, e.g. "AGV stopped: motor
   problem / The robot found a motor problem, moved off the track and
   stopped. It is next to the track between Junction 3 and Junction 2." If
   it could not finish moving off, it says it may be blocking the track.
4. A `LINE_LOST` or `JUNCTION_MISSED` stop also sends a notification ("AGV
   stopped: lost the track" or "missed a turn"), naming a motor or line
   sensor problem if an alarm was raised in the last 10 s. Motor faults often
   end this way: a stalled or weak wheel drags the robot off the line before
   the alarm has lasted 2 s, and the ESP's line-loss stop (750 ms) comes
   first.

The junctions come from the map's node names: the Supervisor sends one
label per route step with the route (`E@`, see `firmware\README.md`).

**Live check** (`data\esp_runs\fault-stop-v1` and `-v2`, the live-v1 round
trips with `esp_fault_stop_s` 2.0; v2 repeats the motor faults with the final
firmware):

| Fault (4 runs each) | Ends in | Notified |
|---|---|---|
| None (`healthy`, motor gain 1.0) | `AT_DEST` 4/4, no alarm | - |
| IR dead left | `FAULT_STOP` 4/4, pulled over 8-18 cm off the line in about 3 s | 4/4 |
| IR noise std 200 | `FAULT_STOP` 4/4, 7-8 cm off the line | 4/4 |
| Motor weak left 50% | `FAULT_STOP` 2/4 (pull-over about 4.1 s), `LINE_LOST` 2/4 | 4/4 |
| Motor stall left | `LINE_LOST` 3/4, `FAULT_STOP` 1/4 (time limit) | 4/4 |

- With a stalled wheel the robot can only pivot, so it stops near the line.
  With a weak wheel the forward drive curves back towards the line (2.7-5 cm
  off).
- Every `LINE_LOST` stop came with a motor alarm in the preceding second.
- Re-run the check with
  `.\.venv\Scripts\python.exe -m scripts.faults.collect_fault_runs --matrix data\esp_runs\fault-stop-v1\fault-matrix.json --out <dir> --timeout 300`.

**Live check with the motor GRU and stuck rule** (`data\esp_runs\fault-stop-v3`,
esp-dsp firmware; same routes, speeds and seeds):

| Fault (4 runs each) | Ends in | First alarm after onset | Notified |
|---|---|---|---|
| None (`healthy`) | `AT_DEST` 4/4, no alarm | - | - |
| Motor weak right 20% | `FAULT_STOP` 4/4, 8-9 cm off the line | GRU, 0.1-0.3 s (5.4 s when the onset falls in the U-turn) | 4/4 |
| Motor weak right 30% | `FAULT_STOP` 4/4, 8-9 cm off the line | GRU, 0.1-0.2 s (4.2 s, U-turn) | 4/4 |
| IR stuck center | `FAULT_STOP` 2/4; 1 ran off the track; 1 `LINE_LOST` | stuck rule, 1.1-1.7 s | 3/4 |

- Before the GRU, these weak motor faults were caught in 3/8 and 6/8
  offline runs, and the stuck sensor in 1/8.
- No alarm came before a fault's onset.
- The stuck run that ran off the track: the center sensor stuck at its
  on-the-line value, so the ESP kept driving straight and never saw a lost
  line; off the track the other two sensors read flat white floor too, so
  the stuck rule (which needs another sensor moving) only fired for ~1 s
  while crossing a line, short of the 2 s hold. Webots ended the run at its
  12 cm path-error limit. The `LINE_LOST` run had its onset in the U-turn
  and lost the line before any window was monitored (notified as "lost the
  track").
- Re-run with
  `.\.venv\Scripts\python.exe -m scripts.faults.collect_fault_runs --matrix data\esp_runs\fault-stop-v3\fault-matrix.json --out <dir> --timeout 300`.

**After the stuck-rule fix and the pull-over sides** (`data\esp_runs\fault-stop-v4`
and `pull-over-v1`; the stuck rule now needs the robot driving rather than
another sensor moving, and the pull-over side comes from the map):

| Runs | Ends in | Notes |
|---|---|---|
| Healthy, 4 | `AT_DEST` 4/4 | no alarm frame |
| IR stuck center, 4 | `FAULT_STOP` 3/4, `LINE_LOST` 1/4 | stuck alarm +1.0-1.1 s; the run that ran off the track in v3 now stops; B pulled over left on the loop, 7.6 cm from its line; D's onset is in the U-turn (notified "lost the track") |
| IR dead left from 1 s on route C, 2 | `FAULT_STOP` 2/2 | pulled over left on edge-005 (Junction 2 to 3), 4-5 cm outside the loop; a right turn there lands on edge-004 |
| Motor weak right 30% from 1 s on route C, 2 | `FAULT_STOP` 2/2 | alarm at +1.5 s, past Junction 3, so right (edge-006), 5 cm from edge-004 at the junction |

No alarm came before an onset and every stop notified.

Remove `esp_fault_stop_s` (or set it to `null`) to only report alarms. The
route command carries it as `F@<seconds>`. The data collectors
(`collect_route_runs`, `collect_fault_runs`) remove it, so dataset runs are
not cut short; a fault matrix can set it in its `mission` block.

**On the phone.** Install the ntfy app (Android: Play Store or F-Droid; iOS:
App Store) and subscribe to the topic in `firmware\wifi.csv` on the default
server `ntfy.sh`. No account is needed. Anyone who knows the topic name can
read and post to it, so use a hard-to-guess name. The phone can be on any
network; the ESP only needs WiFi with internet access.

**WiFi settings, without recompiling.** The network and topic live in their
own flash partition (`wificfg`, see `firmware\partitions.csv`), not in the
firmware. Copy `firmware\wifi.example.csv` to `firmware\wifi.csv` (gitignored)
and edit it:

```csv
key,type,encoding,value
wifi,namespace,,
ssid,data,string,YourNetwork
password,data,string,YourPassword
ntfy_topic,data,string,agv-yourname-abc123
```

An optional `ntfy_server` row points at a self-hosted ntfy server instead of
`https://ntfy.sh`. Then, with COM3 free (close Webots), write only that
partition; it takes a few seconds and leaves the firmware untouched:

```powershell
.\scripts\esp\flash_wifi.ps1            # -Port COM3 -Config firmware\wifi.csv
```

The ESP restarts and connects. Reflashing the firmware (`idf.py flash`) keeps
the WiFi settings. The partition table itself only changes with a full flash,
which was needed once when `wificfg` was added.

- WPA2/WPA3-Personal or open networks work; WPA2-Enterprise (eduroam) does
  not. A phone hotspot is the easiest option on campus.
- Without a `wificfg` partition, WiFi stays off and the robot behaves as
  before; fault stops still happen but send nothing.
- WiFi runs on core 0 and the control loop on core 1, so sending never delays
  a control step.
- Check the connection, and send a test notification with `--test` (COM3
  must be free):

  ```powershell
  .\.venv\Scripts\python.exe -m scripts.esp.wifi_status --port COM3 --test
  ```

  It prints `W,<OFF|CONNECTING|CONNECTED>,<ip>,<last send result>,heap=<bytes>`.
  The send result is `SENDING_<attempt>`, then `HTTP_200`, or an error such as
  `ESP_ERR_HTTP_CONNECT/<esp-tls error>/<mbedTLS error>`. The ESP tries 6
  times.

### Resource usage (heap, CPU, stacks)

Set `AGV_ESP_RESOURCE_LOG_S` (seconds) before any ESP-mode run and
`epuck_route` sends the ESP a `U` query at that interval between frames,
writing `<run>.resources.jsonl` next to the run CSV. Summarize a directory
(add `--port COM3` for a final snapshot, whose lowest-heap figure includes
the last notification):

```powershell
$env:AGV_ESP_RESOURCE_LOG_S = "1"
.\.venv\Scripts\python.exe -m scripts.faults.collect_fault_runs --matrix data\esp_runs\resources-v1\fault-matrix.json --out <dir> --timeout 300
.\.venv\Scripts\python.exe -m scripts.esp.resource_usage --runs <dir> --out reports\esp-resources-v1 --port COM3
```

Flash and static RAM per component: `idf.py size` and `idf.py size-components`
in `firmware`. Results: `reports\esp-resources-v1\summary.md`.

## Train and evaluate the autoencoders

PyTorch (CPU) is in `requirements.txt`. Train both the motor and the IR model
for one architecture:

```powershell
.\.venv\Scripts\python.exe -m scripts.models.train_autoencoder `
  --dataset "data\datasets\sim-baseline-v4" `
  --arch dense `
  --out "models\dense-v1" `
  --epochs 800 --patience 60
```

`--arch` is `dense` (Linear/ReLU, 80->32->8->32->80 for motor) or `conv` (two
Conv1d layers over time, a Linear bottleneck of 8, and the same dense decoder).
Both use only layers that are simple to port to C. Training uses training runs
only, keeps the epoch with the best validation loss, and is seeded.

- **Score:** mean squared reconstruction error of a scaled 20-sample (0.64 s)
  window.
- **Threshold:** the 99.5th percentile of validation scores
  (`--threshold-percentile`).
- **Alarm:** at least 3 of the last 5 windows exceed the threshold
  (`--persist-k`, `--persist-n`).
- **Warm-up:** windows starting in the first 0.5 s of a run (`--warmup-s`) are
  never monitored, because they contain the spawn/settling transient.

A PCA model with the same bottleneck size is scored the same way as a
baseline. The output directory contains `motor.pt`, `ir.pt`, `config.json`
(architecture, scaler, threshold, per-run test results) and score plots.

Healthy test results on `sim-baseline-v4` (14 runs; ideally no alarms):

| Model | Windows over threshold | Runs with a persistent alarm |
|---|---|---|
| Motor dense | 0.35% | 2/14 |
| Motor conv | 0.31% | 0/14 |
| Motor PCA baseline | 0.23% | 1/14 |
| IR dense | 0.59% | 2/14 |
| IR conv | 0.63% | 3/14 |
| IR PCA baseline | 0.68% | 5/14 |

The remaining IR alarms are short and occur at fixed places, such as C at
2.9 rad/s crossing the node-003 junction from edge-005 into edge-006. They are
healthy behaviour that occurs at only a few speeds. Fault detection results
from the labelled runs are summarized below.

## Fault injection and detection evaluation

`epuck_route` applies an optional `fault` block from the mission, starting at
`onset_s`. Every CSV row has `fault_type` and `fault_active` columns; healthy
runs have an empty type and 0. Fault types:

| Type | Target | Effect | `severity` |
|---|---|---|---|
| `motor_gain` | `left`/`right` | Applied wheel speed = command x severity | fraction, e.g. 0.7 |
| `motor_stall` | `left`/`right` | Wheel receives 0 | unused |
| `motor_intermittent` | `left`/`right` | Wheel receives 0 for `burst_s` every `period_s` | unused |
| `encoder_stuck` | `left`/`right` | Logged position freezes at its onset value | unused |
| `encoder_noise` | `left`/`right` | Gaussian noise added to the logged position | std (rad) |
| `ir_stuck` | `gs_left`/`gs_center`/`gs_right` | Reading freezes at its onset value | unused |
| `ir_dead` | ground sensor | Reading becomes a constant | the constant |
| `ir_noise` | ground sensor | Gaussian noise added to the reading | std |
| `ir_drift` | ground sensor | Offset that grows over time | units per second |

Motor faults change what the wheel does; the logged target is still the
controller's command. Encoder faults change only the log. IR faults change the
value the controller and the log both see. Without a `fault` block, runs are
identical to before: a regression run matched a committed healthy run
byte-for-byte in the original 15 columns.

Record the fault matrix (10 faults x 8 held-out test route/speeds, onset at 5 s)
and evaluate:

```powershell
.\.venv\Scripts\python.exe -m scripts.faults.collect_fault_runs
.\.venv\Scripts\python.exe -m scripts.faults.evaluate_faults
```

`data\fault_runs\fault-matrix-v1.json` defines the faults.
`data\fault_runs\manifest.csv` lists each run with its fault and outcome;
`stopped_early` means a line-loss or path safety stop, which is expected for
some faults. `evaluate_faults.py` scores every run with each model directory
and the PCA baseline, using the same threshold, 3-of-5 persistence and 0.5 s
warm-up. It writes `reports\fault-eval-v1\summary.md`, `results.json` and
per-model score plots. Motor and encoder faults count as detected by the motor
model and IR faults by the IR model; alarms from the other model are reported as
cross-coupling.

Results (`conv-v1`; `dense-v1` is similar):

| Fault | Detected | Median delay |
|---|---|---|
| Weak motor (70% / 50%) | 8/8, 8/8 | 0.45 s, 0.16 s |
| Stalled motor | 8/8 | 0.10 s |
| Intermittent motor | 8/8 | 0.11 s |
| Encoder stuck / noisy | 8/8, 8/8 | 0.10 s, 0.29 s |
| IR dead (reads 0) | 8/8 | 0.34 s |
| IR drift (+100/s) | 8/8 | 1.01 s |
| IR stuck (centre) | 1/8 | - |
| IR noise (std 50) | 2/8 | - |

- `conv-v1` had no pre-onset false alarms. `dense-v1` had one pre-onset false
  alarm on one route/speed.
- The PCA baseline matches the autoencoders on motor faults but detects IR dead
  in only 2/8 runs and IR drift in only 1/8.
- A stuck centre sensor freezes near its on-line value, which looks healthy
  except at corners and junctions.
- Noise with std 50 is about 0.2 of the side-sensor scale, so only spikes cross
  the threshold.
- A weak left motor, a stalled motor and IR drift also trip the other model
  (8/8). The robot's motion couples the subsystems: drifting off the line
  changes the IR readings, and losing the line stops the wheels.

### Severity sweep

`data\fault_runs\sweep-v1\fault-matrix.json` tests right-motor power loss from
5% to 50% and right ground-sensor noise from std 25 to 200 on the same 8
route/speeds (104 runs). When a fault type was tested at several severities,
the evaluator also plots detection rate and median delay against severity:

```powershell
.\.venv\Scripts\python.exe -m scripts.faults.collect_fault_runs --matrix data\fault_runs\sweep-v1\fault-matrix.json
.\.venv\Scripts\python.exe -m scripts.faults.evaluate_faults --manifest data\fault_runs\sweep-v1\manifest.csv --out reports\severity-sweep-v1
```

Detection limits (`reports\severity-sweep-v1`, conv-v1 / dense-v1):

| Right-motor power loss | 5% | 10% | 15% | 20% | 30% | 40% | 50% |
|---|---|---|---|---|---|---|---|
| Autoencoders detected | 0-1/8 | 1-2/8 | 4/8 | 7/8 | 8/8 | 8/8 | 8/8 |
| PCA baseline detected | 1/8 | 2/8 | 2/8 | 3/8 | 7/8 | 8/8 | 8/8 |
| Median delay (conv) | - | 3.2 s | 3.0 s | 1.1 s | 0.45 s | 0.26 s | 0.18 s |

| Right IR noise std | 25 | 50 | 75 | 100 | 150 | 200 |
|---|---|---|---|---|---|---|
| Autoencoders detected | 1-2/8 | 2/8 | 1/8 | 4/8 | 6-7/8 | 8/8 |
| PCA baseline detected | 2/8 | 2/8 | 2/8 | 3/8 | 3/8 | 6/8 |

- **Motor:** a power loss of 20% or more is reliably detected. The
  autoencoders beat PCA most clearly between 15% and 30% loss.
- **IR noise:** detection is only reliable at std 200, about 0.9 of the
  side-sensor scaler std. The window score averages squared error over all
  three sensors and 20 samples, which dilutes noise on one sensor. The threshold
  is also set by rare healthy junction crossings.
- **Low severities:** the 1-2/8 detections and their delays come from a few
  runs where the fault coincides with a near-threshold healthy moment. Treat
  them as the noise floor, not as reliable detection.
- **Possible improvements for single-sensor noise:** score each sensor
  separately and take the maximum, or add a high-frequency feature such as the
  sample-to-sample difference.

## ESP-controlled models (the ones to deploy)

The models above were trained on Supervisor-controlled runs. On healthy
ESP-controlled round trips, `conv-v1` alarms in all 18 test runs (34% of
motor and 20% of IR windows over threshold). The ESP's PD loop commands
different wheel targets, and the stop, unload and U-turn are new. The
deployment models are retrained on `esp-baseline-v1`:

```powershell
.\.venv\Scripts\python.exe -m scripts.models.train_autoencoder `
  --dataset data\datasets\esp-baseline-v1 --arch conv --out models\esp-conv-v1 `
  --threshold-rule no-validation-alarm
.\.venv\Scripts\python.exe -m scripts.faults.collect_fault_runs `
  --matrix data\esp_runs\faults\fault-matrix-esp-v1.json --timeout 300
.\.venv\Scripts\python.exe -m scripts.faults.evaluate_faults --models models\esp-conv-v1 `
  --manifest data\esp_runs\faults\manifest.csv --out reports\esp-fault-eval-v1
```

Training takes about 25 minutes per model on the CPU. `--reuse-weights`
re-evaluates saved weights, for example with another threshold rule.

- **Commanded manoeuvres are not monitored.** Windows that overlap the ESP
  states `AT_DEST`, `UNLOADING` or `U_TURN`, or the first 0.5 s after a
  U-turn while the line follower settles, are used for training but excluded
  from thresholds and alarms, like the warm-up.
  - `prepare_telemetry.py` stores the per-window flags.
  - Unmonitored, the stop and the U-turn exit caused most healthy false
    alarms. Both happen once or twice per run.
  - On the ESP the firmware knows its state, so it can apply the same gating.
- **Threshold rule `no-validation-alarm`:** the lowest threshold at which no
  healthy validation run raises a 3-of-5 alarm. With the 99.5th-percentile
  rule, 7/18 (motor) and 5/18 (IR) healthy test runs alarmed. The IR model
  also alarmed before onset in 15 of the 120 fault runs, on unseen speeds.
- **Fault matrix:** 15 faults x 8 round trips (120 runs).
  - The runs use speeds between the healthy grid points and new jitter
    seeds.
  - Onsets vary from 4 to 18 s, so they fall in the outbound leg, the
    unload/U-turn or the return leg.

Healthy test runs (18): the motor model alarms in 2/18 (0.09% of windows) and
the IR model in 0/18. The 2 motor alarms are the last 3 cm into a terminal
node (C at 3.75 rad/s) and leaving a right turn at 2.0 rad/s. No fault run
alarmed before its onset (0/120).

| Fault | Detected (own model) | Median delay | Other model fired |
|---|---|---|---|
| Motor stall / intermittent | 8/8, 8/8 | 0.38 s, 0.67 s | 0/8, 1/8 |
| Motor 50% / 30% / 20% power loss | 8/8, 6/8, 3/8 | 2.2 s, 1.8 s, 4.1 s | 1/8, 0/8, 1/8 |
| Encoder stuck / noisy | 8/8, 8/8 | 0.56 s, 1.06 s | 0/8, 0/8 |
| IR dead | 8/8 | 0.53 s | 0/8 |
| IR noise std 200 / 100 (right) / 100 (left) / 50 | 8/8, 3/8, 3/8, 0/8 | 2.2 s, 8.0 s, 14.0 s, - | 8/8, 7/8, 7/8, 5/8 |
| IR drift centre / right | 4/8, 6/8 | 5.6 s, 4.4 s | 5/8, 7/8 |
| IR stuck centre | 1/8 | 1.1 s | 1/8 |

- **Under ESP control, IR noise also shows up in the motor model.** The noisy
  readings make the PD loop command jittery wheel targets. Noise of std 100
  trips the motor model in 7-8/8 runs, while the IR model alone detects it in
  3-4/8. "Either model alarms" is a much better fault detector than the IR
  model alone, but it does not say which subsystem failed.
- Many faults stop the run (`LINE_LOST`, `JUNCTION_MISSED`): every stall,
  stuck encoder, dead sensor and drift run, and 7/8 stuck-sensor runs. The
  ESP's own safety stops are therefore a second line of defence. Undetected
  drift and stuck-sensor runs ended this way.
- The PCA baseline matches the conv model on motor faults but misses most IR
  faults (IR dead 0/8).

**Comparison with the Supervisor-trained models**
(`reports/esp-model-comparison-v1`): on the same ESP runs, gating and
3-of-5 rule, the old models are unusable.

| Model | Healthy test runs alarming (motor, IR) | Healthy windows over threshold |
|---|---|---|
| esp-conv-v1 | 2/18, 0/18 | 0.09%, 0.01% |
| conv-v1 | 18/18, 18/18 | 23.2%, 13.3% |
| dense-v1 | 18/18, 18/18 | 24.8%, 14.1% |

The old models "detect" almost every fault in 8/8 runs only because they are
already alarming before the onset, in nearly every fault run.

### Inference on the ESP

The firmware runs both conv models, the motor GRU and the stuck-sensor rule on
every sensor frame with esp-dsp, and returns the scores and alarms in the
motor frame (see `firmware/README.md`). After retraining, regenerate the
weights header and reflash:

```powershell
.\.venv\Scripts\python.exe -m scripts.models.export_c --model models\esp-conv-v2 --gru models\esp-gru-v2 --out firmware\main\anomaly_model.h
```

The deployed motor conv and GRU are the architecture-search winners
(`reports\arch-search-v1\<model>\final-0`, retrained with the full budget):
`models\esp-conv-v2` holds that motor model next to the unchanged IR model,
with each model's window and sizes in `models.<name>`, and `models\esp-gru-v2`
the 16-hidden GRU. Their thresholds are the search's final-run thresholds
(no persistent alarm on a healthy validation run; GRU x2); each config's
`source` and `search_result` record where they came from. `esp-conv-v1` and
`esp-gru-v1` still export and replay with the same commands.

The v1 GRU and its threshold came from the offline experiment, which also
writes the `deploy` block of `models\esp-gru-v1\config.json`
(`--reuse-weights` re-evaluates without training, ~2 min):

```powershell
.\.venv\Scripts\python.exe -m scripts.models.gru_experiment --reuse-weights
```

**Agreement with Python.** `scripts/esp/replay_scores.py` replays recorded
ESP runs through the ESP: the same route commands and sensor/encoder frames,
without the host-side unload rows. It compares the ESP's wheel targets,
scores and alarms with the Python reference:

```powershell
.\.venv\Scripts\python.exe -m scripts.esp.replay_scores --port COM3 <run.csv> ... --out reports\esp-replay-v1\agreement.json
```

On the 18 healthy test runs and one run of each of the 15 faults (65044
windows):
- Scores differed by at most 4.5e-5. Thresholds are 0.19 (motor) and 0.45
  (IR).
- The alarm decisions were identical on every window: 2559 alarm windows,
  0 mismatches.
- One wheel target in one run differed by 0.001 rad/s. The `-O2` build
  rounds one value the other way.

With the esp-dsp firmware, the motor GRU and the stuck rule
(`reports\esp-replay-v2\summary.json`: 2 healthy test runs, 20%/30% weak
motor, stall, stuck encoder, stuck and dead IR): conv scores within 5e-5,
GRU scores within 1.6e-5 (threshold 0.025), and the GRU, stuck, motor and IR
alarms identical on every frame. The Python reference for the GRU and the
stuck rule restarts per leg like the ESP.

**Live end-to-end** (`data\esp_runs\live-v1`, 16 round trips with the ESP
driving and detecting):
- The 4 healthy runs raised no alarm.
- 50% weak left motor: detected by the motor model 4/4 (0.5-5.2 s).
- Dead left IR: detected by the IR model 4/4 (0.1-3.9 s).
- IR noise std 200: detected by the IR model 4/4 (0.7-3.9 s), and the motor
  model fired too.
- No alarm came before a fault's onset.
- The logged live scores matched Python within 3.3e-5, with no alarm
  mismatches.

The first plain C++ inference cost about 4.5 ms per frame. With esp-dsp and
the weights in RAM the three detectors took about 1.05 ms, and about 0.81 ms
with the architecture-search winners (see `firmware/README.md`).

Raw run logs (`route-*.csv`, `controller-errors.log`, `data/SensorData*.csv`) are
git-ignored and not in the repository; the manifests, the prepared datasets
(`data/datasets`), the models and all reports are. Replaying recorded runs
(`replay_scores`), the offline fault evaluation and GRU/search retraining
need the raw logs; driving, detecting, exporting and conv training do not.

### Other tracks (track2, track3)

```powershell
.\.venv\Scripts\python.exe -m scripts.maps.make_track   # image + map (with world transform)
.\.venv\Scripts\python.exe -m scripts.maps.make_track --track track3   # the hard course; mission-track3.json, "Self Diagnosis AGV track3.wbt"
.\.venv\Scripts\python.exe -m scripts.telemetry.collect_route_runs --routes ABCD --speeds 2.5 3.5 --control-mode esp32 --round-trip --mission maps\routes\mission-track2.json --world "simulation\worlds\Self Diagnosis AGV track2.wbt" --out data\esp_runs\nav-fix-v1\track2
.\.venv\Scripts\python.exe -m scripts.faults.collect_fault_runs --matrix data\esp_runs\track2-v2\faults\fault-matrix.json --mission maps\routes\mission-track2.json --world "simulation\worlds\Self Diagnosis AGV track2.wbt" --timeout 300
```

A mission's `map_path` selects the map (the world must show the same image)
and its `routes` table (`{"A": {"start", "destination", "avoid_edges"}}`)
replaces the base track's routes in both collectors. `replay_scores` still
uses the base track map.

### Latency table, INT8 benchmark and architecture search

These back the architecture choices with measurements (results in
`REPORT.md` sections 14-16).

**On-ESP latency table.** The AGV firmware's `B` command times any conv
autoencoder or GRU size with the same esp-dsp code. Sweep the search space
(967 architectures, ~2 min; needs COM3):

```powershell
.\.venv\Scripts\python.exe -m scripts.models.esp_latency --port COM3 --out reports\esp-latency-v1
```

**Architecture search.** One Optuna study per detector (`motor`, `ir`,
`gru`), tuning on half the fault runs and minimizing the measured latency;
studies resume from `reports\arch-search-v1\<model>\study.db`. Conv trials
take ~3-6 min, GRU trials ~10-20 min on this laptop's CPU:

```powershell
.\.venv\Scripts\python.exe -m scripts.models.search_architectures --model motor --trials 40
.\.venv\Scripts\python.exe -m scripts.models.search_architectures --model ir --trials 40
.\.venv\Scripts\python.exe -m scripts.models.search_architectures --model gru --trials 12
```

Retrain chosen architectures (JSON `{model: [architecture, ...]}`, e.g.
`reports\arch-search-v1\chosen_conv.json`) with the full budget and compare
them with the deployed models on the held-out faults and healthy test runs:

```powershell
.\.venv\Scripts\python.exe -m scripts.models.search_architectures --final reports\arch-search-v1\chosen_conv.json --final-models motor ir
```

**INT8 / INT16.** esp-ppq needs Python 3.11-3.12, so it has its own
environment (`.venv-ppq`, git-ignored):

```powershell
uv venv --python 3.12 .venv-ppq
uv pip install --python .venv-ppq\Scripts\python.exe "esp-ppq[cpu]"
.\.venv-ppq\Scripts\python.exe -m scripts.models.quantize_int8 --quant w8a8 w16a16
```

This exports `.espdl` models to `models\esp-quantized-v1\<quant>\` and
writes the simulated accuracy to `reports\esp-int8-v1
esults.json`. To time
them on the ESP, flash the separate ESP-DL app (it replaces the AGV firmware
and uses the default large-app partition table; the `wificfg` data is kept),
run the benchmark, then flash the AGV firmware again:

```powershell
cd firmware\esp-dl-bench; idf.py build; idf.py -p COM3 flash; cd ..\..
.\.venv\Scripts\python.exe -m scripts.esp.int8_bench --port COM3
cd firmware; idf.py -p COM3 flash; cd ..
```

ESP-DL's prebuilt loader calls `esp_log_write`, which this IDF 6.0 build does
not provide; the bench app defines a weak empty one.

## Deployment and integration status

The control half of the loop works: in ESP mode, `epuck_route` is the Python
bridge, and all four two-edge routes (A-D) complete with the ESP computing the
wheel commands, one way or as round trips, at 2.0-4.0 rad/s. The ESP stopped
1.0-1.4 cm from the destination node, and the maximum path error was 2.4-2.5
cm, the same as the Supervisor controller.

Anomaly detection also runs on the ESP and agrees with Python (see above).
On a persistent alarm the robot pulls over to the map-chosen side, stops and
notifies the phone.

The same ESP firmware drives track2 and track3 (see "Other tracks"): routes
complete as round trips with stops within 2 cm, and every injected fault
stops the robot.

Still to do (see `docs/TODO.md`): IR drift and light IR noise detection, a
real-unplug test of the COM3 reconnect, the track3 hairpin margin at
3.5 rad/s, and healthy training runs over edge-002.

## Tests

Run the map and telemetry-preparation tests:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_*.py" -v
```

These are unit tests for the Python tools (46 at last verification). The serial
smoke test requires the flashed ESP and an available COM port. Neither replaces
a closed-loop run: set `control_mode` to `esp32` and run routes A-D in Webots.
