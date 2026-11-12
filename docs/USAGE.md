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

The current pilot illustrates the preparation workflow:

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

- `windows.npz`: `motor_train`, `ir_train`, `motor_validation`, and
  `ir_validation` arrays, with shape `[windows, time, features]`;
- `metadata.json`: feature names, run split, sampling/sensor-quality summaries,
  scaler parameters, exclusions, and an explicit note that there is no final
  test split yet.

At least two non-duplicate healthy runs are needed to create both training and
validation sets. One unique run creates training windows only. Do not treat
overlapping windows from the same run as independent train/validation data.
The checked-in pilot uses one 2.5 rad/s run for training and one 3.0 rad/s run
for validation on the same route. It validates the pipeline, not model quality;
there is no trained model or controlled-fault evaluation yet.

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

## Model, deployment, and integration status

No autoencoder training/evaluation scripts or trained model artifacts are
available yet. The next steps are to collect more distinct healthy runs, create
controlled fault runs, train and evaluate motor/IR models, determine thresholds
from held-out healthy data, and then implement ESP-compatible inference.

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
