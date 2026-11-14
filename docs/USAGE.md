# Project Usage

Run commands from the repository root in PowerShell unless a section says
otherwise. This project is simulation-first: Webots supplies the robot,
environment, sensors, and encoder readings. The ESP32-S3 is an external
computation target connected by USB serial; this is **not** a physical AGV
workflow.

## Project files and what they do

| Path | Purpose |
|---|---|
| `simulation/worlds/Self Diagnosis AGV.wbt` | Webots world, track texture, robot, and selected controller |
| `simulation/controllers/epuck_route/` | Map route execution in Webots; uses Supervisor pose and ground-sensor correction |
| `simulation/controllers/epuck_data/` | Sensor-only PD line follower and per-session telemetry collector |
| `maps/image/` | Track images used by the world and map-conversion tools |
| `maps/paths/` | Editable graph-map JSON files |
| `maps/visualizations/` | Generated standalone HTML map previews |
| `maps/routes/mission.json` | Webots route mission, speed, and simulation settings |
| `scripts/pathToMap.py` | Converts a path image into a draft graph map |
| `scripts/visualizeMap.py` | Generates an HTML map editor/preview |
| `scripts/route_planner.py` | Validates the graph and finds shortest paths |
| `scripts/prepare_telemetry.py` | Converts independent telemetry runs into scaled motor/IR windows |
| `scripts/collect_route_runs.py` | Records route x speed runs headlessly in Webots and updates the run manifest |
| `scripts/train_autoencoder.py` | Trains/evaluates motor and IR autoencoders (dense or conv) against a PCA baseline |
| `models/` | Trained weights, thresholds, scalers and evaluation results |
| `scripts/collect_fault_runs.py` | Records labelled fault-injection runs headlessly from a fault matrix |
| `scripts/evaluate_faults.py` | Measures detection rate, delay, pre-onset false alarms and cross-coupling |
| `data/fault_runs/` | Fault matrix, fault-run CSVs and their manifest |
| `reports/` | Fault-evaluation summaries, results and plots |
| `scripts/esp_serial_smoke_test.py` | Tests the flashed ESP controller over COM3 with simulated sensor frames |
| `esp32-s3/` | ESP-IDF project, serial protocol, and prototype PD/junction firmware |
| `data/route_runs/` | Per-run telemetry from the map-based Webots controller |
| `data/sensor_runs/` | Per-session telemetry from the sensor-only PD controller |
| `data/datasets/` | Prepared model-input arrays and dataset metadata |
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
   .\.venv\Scripts\python.exe scripts\pathToMap.py `
     -img "maps\image\new-track.png" `
     -out "maps\paths"
   ```

3. Generate and open the HTML preview:

   ```powershell
   .\.venv\Scripts\python.exe scripts\visualizeMap.py `
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
.\.venv\Scripts\python.exe scripts\route_planner.py `
  "maps\paths\base track.json" node-003 node-001 `
  -out "maps\routes\active_route.json"
```

Edit `maps\routes\mission.json` to select `start_node`,
`destination_node`, and `wheel_speed_rad_s`. Then run the world in Webots. The
current route controller plans a shortest-distance path, uses Supervisor pose
to track map geometry, and uses the three ground sensors for local correction
and line-loss stopping. It writes one CSV per run to `data\route_runs`.

Supervisor pose is ground-truth simulation information, not an ESP input
mechanism. The eventual ESP route interface will use relative junction actions;
graph-to-action conversion and the live Webots serial bridge are not yet
implemented.

## Collect telemetry

### Map-based route runs

The configured `epuck_route` controller automatically writes a timestamped CSV
under `data\route_runs` when a route executes. Columns include route/edge IDs,
pose, centerline error, target motor speeds, wheel encoder positions, and all
three ground IR readings. The route controller follows map geometry using
Supervisor pose, so these runs are useful for simulation baselines but do not
test the ESP control loop.

**Headless batch recording (preferred).** Record a route x speed matrix
without opening the Webots window:

```powershell
.\.venv\Scripts\python.exe scripts\collect_route_runs.py `
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
.\.venv\Scripts\python.exe scripts\prepare_telemetry.py `
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

The older `--runs` form still works and splits by run-id order with
`--validation-fraction` (no test split):

```powershell
.\.venv\Scripts\python.exe scripts\prepare_telemetry.py `
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

The ESP32-S3 N16R8 has a prototype ESP-IDF controller project under
`esp32-s3`. Use the VS Code ESP-IDF terminal configured for ESP-IDF 6.0.1:

```powershell
cd esp32-s3
idf.py set-target esp32s3
idf.py build
idf.py -p COM3 flash
```

Flashing is optional for building and overwrites firmware on the connected ESP.
Close the IDF monitor or any other process that owns COM3 before flashing or
running another serial test.

To check the flashed prototype from the repository root:

```powershell
.\.venv\Scripts\python.exe scripts\esp_serial_smoke_test.py --port COM3
```

The script sends a short route and synthetic IR frames, then checks PD
wheel-speed outputs, one junction action, junction clearing, stop at destination,
and invalid-route rejection. It does **not** launch Webots; the data is
synthetic and the returned wheel commands are not applied to the simulated
robot.

See `esp32-s3\README.md` for the serial frame format and firmware limitations.
The current firmware consumes route actions and three IR readings and returns
wheel speeds. The motor autoencoder needs target and measured wheel features,
and its inference needs to be implemented and added to the ESP protocol.

## Train and evaluate the autoencoders

PyTorch (CPU) is in `requirements.txt`. Train both the motor and the IR model
for one architecture:

```powershell
.\.venv\Scripts\python.exe scripts\train_autoencoder.py `
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
healthy behaviour that occurs at only a few speeds. No fault runs exist yet, so
detection rate and delay are not measured.

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
cd scripts
..\.venv\Scripts\python.exe collect_fault_runs.py
..\.venv\Scripts\python.exe evaluate_faults.py
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

## Deployment and integration status

The next step is ESP-compatible inference (exporting the weights to C).

After that, implement the Python Webots bridge. It must stream the needed
telemetry to the ESP, send the route/action sequence, read wheel commands and
anomaly results, and apply/log them in Webots. The end-to-end loop—Webots to
ESP and back to Webots—has not yet been built or tested.

## Tests

Run the map and telemetry-preparation tests:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s scripts -p "test_*.py" -v
```

These are unit tests for the Python tools. The serial smoke test requires the
flashed ESP and an available COM port. Neither test suite replaces full
Webots/ESP closed-loop validation.
