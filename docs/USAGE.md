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
| `scripts/maps/` | Track-image conversion, map visualization, and graph route planning |
| `scripts/telemetry/` | Healthy Webots run collection and telemetry preparation |
| `scripts/models/` | Autoencoder training and healthy-run evaluation |
| `scripts/faults/` | Fault-run collection and detection evaluation |
| `scripts/esp/` | Route-to-ESP action/distance conversion and the ESP serial smoke test |
| `tests/` | Automated Python regression tests |
| `notebooks/` | Exploratory analysis notebooks |
| `models/` | Trained weights, thresholds, scalers and evaluation results |
| `data/fault_runs/` | Fault matrix, fault-run CSVs and their manifest |
| `reports/` | Fault-evaluation summaries, results and plots |
| `firmware/` | ESP-IDF project, serial protocol, and prototype PD/junction firmware |
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

The distances are the map centerline distances multiplied by
`esp_odometry_scale` (default 0.92). The robot cuts inside curves, so its
odometry reads less than the drawn centerline; 0.92 is the ratio measured on
the 64 healthy base-track runs. To re-measure it on another track, compare
each healthy run's final odometry (0.02 m x mean wheel rotation) with the
route's map length. The firmware's protocol and behavior are described in
`firmware\README.md`.

The Supervisor no longer ends an ESP run itself when the robot reaches the
destination node. It waits for the ESP's `AT_DEST` and marks the run completed
only if the robot stopped within `esp_destination_tolerance_m` (default
0.05 m) of the node. ESP-reported `LINE_LOST`, `JUNCTION_MISSED`, a serial
timeout, or a malformed response stops the motors and fails the run.
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
handover to line following, and giving up when no line is found), and
malformed-route rejection. It does **not** launch Webots; the data is synthetic and the
returned wheel commands are not applied to the simulated robot.

See `firmware\README.md` for the serial frame format and firmware limitations.
The firmware consumes the route, three IR readings and both encoder positions
and returns wheel speeds. The sensor frame therefore already carries the inputs
the motor autoencoder needs (targets are the ESP's own outputs; actual wheel
speeds come from encoder differences), but inference is not implemented on the
ESP yet.

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

## Deployment and integration status

The control half of the loop works: in ESP mode, `epuck_route` is the Python
bridge, and all four two-edge routes (A-D) complete with the ESP computing the
wheel commands. The ESP stopped 1.0-1.4 cm from the destination node, and the
maximum path error was 2.4-2.5 cm, the same as the Supervisor controller.

Still to do:

- ESP-compatible anomaly inference (exporting the weights to C) and a way to
  report scores in the motor frame.
- Check whether the models trained on Supervisor-controlled runs raise false
  alarms on ESP-controlled runs. The ESP's PD loop commands different wheel
  targets, so a healthy dataset recorded under ESP control may be needed.

## Tests

Run the map and telemetry-preparation tests:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_*.py" -v
```

These are unit tests for the Python tools (36 at last verification). The serial
smoke test requires the flashed ESP and an available COM port. Neither replaces
a closed-loop run: set `control_mode` to `esp32` and run routes A-D in Webots.
