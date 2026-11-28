# ESP-AGV demo and architecture guide

This guide explains what to run, what the demo demonstrates, how faults are
created, what the detectors and scripts do, where their weights live, and what
the current evidence does (and does not) establish. Commands assume PowerShell
from the repository root.

For the full command reference and configuration details, see
[USAGE.md](USAGE.md). The measured results and their experimental context are
in [REPORT.md](../REPORT.md); verified scope and open items are in
[STATUS.md](STATUS.md) and [TODO.md](TODO.md).

## What this project is

ESP-AGV is a simulation-first autonomous guided vehicle prototype. Webots
simulates an e-puck robot driving a track. A Python controller plans the route,
reads Webots sensors and encoders, and bridges each frame over USB serial to an
ESP32-S3. The ESP runs the line-following controller and anomaly detectors;
Webots applies the wheel commands returned by the ESP.

```text
Webots: track + simulated robot + sensor/encoder readings
                 |
                 v
Python epuck_route bridge -- USB serial / COM3 --> ESP32-S3
                 ^                                |
                 |                                | PD control, detectors,
                 +--- wheel commands, alarms -----+ fault response
                 |
                 +--- logs and applies wheel commands in Webots
```

The ESP is real hardware in the loop, but the robot, motors, encoders, and
ground sensors in this project are simulated. The ESP does not directly drive
a physical AGV. This is a prototype and a simulation-based feasibility
demonstration, not a field-ready safety system.

## Quick start: watch a run

### Requirements

- Python 3.12 or newer and packages from `requirements.txt`.
- Webots R2025a installed.
- For the normal ESP demo: an ESP32-S3 flashed with this project's firmware,
  connected on the serial port selected in the mission (default `COM3`).
- For phone notifications: WiFi/ntfy configuration on the ESP is optional.

Create the Python environment once:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

Confirm both `simulation\controllers\epuck_route\runtime.ini` and
`simulation\controllers\epuck_data\runtime.ini` point to
`.venv\Scripts\python.exe`. If the firmware needs to be built or flashed, use
the ESP-IDF 6.0.1 environment; see [firmware/README.md](../firmware/README.md).
Close the ESP-IDF monitor and any other serial program before starting the
demo, since only one process can own the COM port at a time.

### Recommended demo: healthy run, then a fault

The demo launcher creates a temporary mission, so it does not edit
`maps\routes\mission.json`. It opens Webots in a window, uses the ESP by
default, and writes the run CSV under `data\route_runs\demo`.

```powershell
# Healthy ESP-controlled round trip
.\.venv\Scripts\python.exe -m scripts.run_demo `
  --track base --start node-001 --destination node-004

# Weak right motor, injected at 6 seconds; fault-stop is enabled
.\.venv\Scripts\python.exe -m scripts.run_demo `
  --track base --start node-001 --destination node-004 --fault weak-motor
```

Watch the console for the injection time, alarm changes, and ESP states. A
clear fault demo is a dead IR sensor:

```powershell
.\.venv\Scripts\python.exe -m scripts.run_demo `
  --track track2 --start node-001 --destination node-004 --fault ir-dead
```

The dead sensor often visibly steers the robot off its usual line before the
IR detector alarms. A weak motor is caught quickly but the PD controller can
compensate, so its path may look nearly normal. The `ir-dead` or
`weak-motor` presets are generally clearer than `ir-drift`, which the current
detector often misses.

Useful launcher options:

| Option | Effect |
|---|---|
| `--track base`, `track2`, `track3` | Choose the Webots world and mission. |
| `--speed 3.0` | Set wheel speed in rad/s. |
| `--fault <name>` | Inject one of the fault presets listed below. |
| `--target <wheel-or-sensor>` | Override the preset target. |
| `--severity <value>` | Override its severity; interpretation depends on fault type. |
| `--onset 6` | Set fault start time in simulated seconds. |
| `--seed <n>` | Apply reproducible start-pose jitter. |
| `--one-way` | Do not return to the start after reaching the destination. |
| `--no-fault-stop` | Report alarms but do not ask the ESP to pull over and stop. |
| `--fast` | Run the Webots window in fast mode. |
| `--headless` | Run without a window and print the final state and first alarms. |
| `--supervisor` | Use Webots' controller instead of the ESP; only supports one-way demos. |
| `--out <path>` | Choose where the run CSV is written. |
| `--webots <path>` | Specify the Webots executable when it is not found automatically. |

For a quick headless check:

```powershell
.\.venv\Scripts\python.exe -m scripts.run_demo `
  --track base --fault weak-motor --headless
```

For a visual run without an ESP, use `--supervisor --one-way`. This is useful
to see the Webots fault effect, but it does **not** demonstrate ESP anomaly
detection, ESP fault-stop behavior, or ESP notifications. Without WiFi
configured, the ESP can still detect and stop; it just cannot deliver the
phone notification.

### What a successful demo looks like

- A healthy run ends at `AT_DEST` (and, unless `--one-way` is used, returns to
  the start).
- On a detectable injected fault, an alarm appears after the fault onset. If
  the alarm persists for the configured hold (2 seconds in the demo), the ESP
  enters `FAULT_PULLOVER`, then `FAULT_STOP`. A configured WiFi connection
  queues a notification.
- A CSV is saved under the output directory. ESP runs include state, distance,
  model scores, and alarms in addition to sensor and motor telemetry.
- A run may instead end in `LINE_LOST`, `JUNCTION_MISSED`, or a simulation
  path-safety failure. These are not successful anomaly detections; inspect
  the console and CSV. Some severe faults physically drive the simulated robot
  off the line before the two-second fault-stop hold can complete.

To check the firmware/protocol without opening Webots, connect the ESP and run:

```powershell
.\.venv\Scripts\python.exe -m scripts.esp.esp_serial_smoke_test --port COM3
```

This sends synthetic sensor and encoder frames directly over serial. It
checks controller and protocol behavior; it is not a Webots fault-evaluation
run.

## Faults: how they are injected and what they mean

An optional `fault` object in a Webots mission configures the
`FaultInjector` in `simulation\controllers\epuck_route\epuck_route.py`.
At and after `onset_s`, it modifies values in the simulation loop. Each run
logs `fault_type` and `fault_active`, so the evaluator can compare alarm time
with the known injected onset.

| Fault type | What changes in Webots | Target and severity |
|---|---|---|
| `motor_gain` | Applied motor command is multiplied by the severity. The logged target remains the controller command. | `left`/`right`; fraction of command, e.g. `0.7` means 30% power loss. |
| `motor_stall` | Applied command to one wheel becomes zero. | `left`/`right`; severity unused. |
| `motor_intermittent` | One wheel is stopped for `burst_s` each `period_s`. | `left`/`right`; period and burst parameters. |
| `encoder_stuck` | One reported wheel position freezes at its onset value. | `left`/`right`; severity unused. |
| `encoder_noise` | Gaussian noise is added to one reported wheel position. | `left`/`right`; severity is standard deviation in radians. |
| `ir_stuck` | One ground-sensor reading freezes at its onset value. | `gs_left`, `gs_center`, or `gs_right`; severity unused. |
| `ir_dead` | One ground-sensor reading becomes a constant. | Ground sensor; severity is the constant reading. |
| `ir_noise` | Gaussian noise is added to one ground-sensor reading. | Ground sensor; severity is standard deviation in sensor units. |
| `ir_drift` | An offset grows linearly from onset. | Ground sensor; severity is sensor units per second. |

Motor faults alter the applied simulated motor velocity. Encoder faults alter
the reported encoder values, not the underlying wheel motion. IR faults alter
the readings that both the controller and the log see. The injected signals
are useful, controlled disturbances, but they are software models of faults,
not measured failures from physical hardware. The `encoder_noise` and `ir_noise` disturbances use
seeded Gaussian noise.

`fault_active=1` means the injector is active. It does not mean a detector has
recognized it. Likewise, an `esp_*_alarm=1` row is a detector output, and a
fault-stop is a later response to a sustained alarm. Keep these separate when
reading a run.

## Are we just creating simulated noise and training on it?

No. The deployed models are trained on **healthy** telemetry, not on the
injected fault samples:

- The motor and IR convolutional autoencoders learn to reconstruct normal
  10- or 20-frame telemetry windows. Unusual windows have higher
  reconstruction error.
- The motor GRU learns from healthy runs to predict measured wheel speeds
  from the commanded wheel speeds. A mismatch between expected and measured
  behavior raises its prediction-error score.
- The stuck-sensor detector is a fixed rule, not a trained model.

Data is split by whole run; windows from a run are not spread between train
and test. The feature scaler is fit on training data only. Healthy validation
runs determine thresholds. Fault runs are then used to measure detection
rate and delay, and healthy test runs measure false alarms. Faults are
controlled simulation interventions, so the ground-truth onset and type are
known, which makes repeatable testing possible.

This design answers a limited but useful question: can detectors learned from
normal simulated behavior notice selected deviations, and can those detectors
run in the target ESP32-S3 control loop? It does **not** show that these exact
fault signatures cover real motor wear, sensor contamination, lighting
changes, wheel slip, or other physical failures.

## How does this prove the concept?

It is evidence for an end-to-end **simulation-to-microcontroller prototype**,
not proof of real-world fault diagnosis. The evidence is more than plotting
synthetic noise:

1. The models learn normal behavior from healthy runs. Faults are kept out of
   training and injected later as test interventions.
2. Evaluation uses held-out runs and reports false alarms, detection rate,
   onset-to-alarm delay, and alarms before onset—not just example scores.
3. The ESP32-S3 runs the model inference itself. It receives the simulated
   inputs over USB serial; Python replay comparisons found matching
   predictions and alarm decisions.
4. The system closes the loop: the ESP controls the simulated robot, and a
   persistent alarm can make it pull over and stop. Runs have been repeated
   on three different simulated track layouts.

Current summary from `REPORT.md` (results are simulation results):

- On 120 labelled ESP-controlled fault runs, the fault's primary detector
  caught 94/120; either motor or IR alarm caught 110/120. No pre-onset alarms
  were reported in those evaluated runs.
- Healthy validation had no persistent alarms; the healthy test set had
  motor alarms on 2/18 runs and IR alarms on 0/18.
- The live fault-stop checks have shown the expected response on the tested
  faults and tracks; notably, some stuck-sensor or line-loss cases stop via
  `LINE_LOST`, not via a learned alarm.
- The current three learned detector computations take about 0.81 ms per
  32 ms sensor frame on the ESP32-S3, using the project's float32 esp-dsp
  implementation.

These results demonstrate feasibility and quantify known behavior under the
tested Webots scenarios. They do not establish field reliability, safety
certification, generalization to arbitrary faults, or performance on a
physical e-puck. A physical validation campaign with recorded real telemetry
and safely controlled faults would be needed for those claims.

## How many models are there, and why?

The current ESP firmware runs **four detectors**: **three learned neural
models** and **one rule**. Motor and IR alarms combine complementary detectors
using OR logic: motor conv OR motor GRU, and IR conv OR stuck-sensor rule.
An alarm indicates unusual behavior, not a definitive diagnosis; faults can
cross-couple (for example IR noise can make the motor controller jitter).

| Detector | Input and method | Why it is present |
|---|---|---|
| Motor convolutional autoencoder | Last 10 samples of left/right target and encoder-derived wheel speeds; reconstruction error. | Detects unusual motor/encoder patterns; the smaller search-selected model is fast and complements the GRU on short, abrupt faults. |
| IR convolutional autoencoder | Last 20 samples of the three ground-IR readings; reconstruction error. | Detects unusual IR patterns such as dead sensors, substantial noise, and some drift. |
| Motor GRU plant model | Stateful prediction of both wheel speeds from the two commanded wheel speeds; prediction error averaged over 31 samples. | Adds temporal context and improves detection of weaker motor-power loss; does not use measured speed as an input that could hide a weak wheel. |
| Stuck-IR rule | Checks whether any ground sensor is nearly unchanged for about one second while the robot is moving. | Catches a sensor frozen at a plausible value, where a general reconstruction model may not see a sufficiently unusual pattern. |

The GRU and autoencoders have separate learned weights, scores, and
thresholds. The OR-combined alarm is used for a robust response, not for
reliable fault-type classification. Detector thresholds alarm on 3 of the
last 5 monitored windows. Startup, turns, destination/unload, and pull-over
periods are gated out as documented in [firmware/README.md](../firmware/README.md).
The fault-stop has its own alarm-hold duration, separate from the 3-of-5
detector persistence.

There are also older models and experiments in the repository; they are not
all run together on the current firmware:

| Directory | Role |
|---|---|
| `models/esp-conv-v2` + `models/esp-gru-v2` | Current deployed motor/IR conv models and motor GRU. |
| `models/esp-conv-v1` + `models/esp-gru-v1` | Earlier ESP model versions and GRU experiment; useful for comparison/replay. |
| `models/conv-v1` + `models/dense-v1` | Earlier models trained under Supervisor control; they do not transfer cleanly to ESP-controlled telemetry. |
| `models/esp-quantized-v1` | INT8/INT16 ESP-DL export experiment, not the deployed detector path. |

## Where are the weights?

The current Python model checkpoints and configuration are:

- `models\esp-conv-v2\motor.pt` and `ir.pt`: learned conv autoencoder
  weights.
- `models\esp-conv-v2\config.json`: features, architecture sizes, scalers,
  thresholds, window lengths, dataset/source metadata.
- `models\esp-gru-v2\gru_motor.pt`: learned motor GRU weights.
- `models\esp-gru-v2\config.json`: GRU architecture, scaler, threshold,
  persistence, and stuck-sensor rule parameters.

The `.pt` files are PyTorch state dictionaries for training, evaluation, and
export. The firmware does not load PyTorch files. Instead,
`scripts\models\export_c.py` converts weights, scalers, thresholds, and
monitoring parameters into
`firmware\main\anomaly_model.h`. That generated header is compiled into the
firmware image in flash; at startup detector weights are copied into internal
RAM for faster inference. The export source comment says “Do not edit”:
regenerate it from the model directories instead.

To regenerate and flash after intentionally changing trained weights:

```powershell
.\.venv\Scripts\python.exe -m scripts.models.export_c `
  --model models\esp-conv-v2 `
  --gru models\esp-gru-v2 `
  --out firmware\main\anomaly_model.h

Set-Location firmware
idf.py build
idf.py -p COM3 flash
Set-Location ..
```

Keep the model config and generated header consistent. Export does not itself
flash the device. The float32 esp-dsp path is used because the quantized
ESP-DL experiments were slower for these small models and quantization changed
the motor model's behavior.

## What do the ESP states and faults mean?

States are control or response states, not all anomaly labels:

| State | Meaning |
|---|---|
| `FOLLOWING` | Following the line. |
| `TURN_LEFT`, `TURN_RIGHT`, `STRAIGHT` | Executing the next odometry-gated junction action. |
| `U_TURN` | Turning around for the return leg. |
| `AT_DEST` | Destination distance reached and wheels stopped. |
| `UNLOADING` | Host-side wait before configuring the return route. |
| `FAULT_PULLOVER` | Turning and moving off the line in response to a persistent alarm. |
| `FAULT_STOP` | Pull-over response finished; wheels stopped due to the alarm. |
| `LINE_LOST` | Bounded attempt to reacquire the line failed. This is a safety stop, not proof that a model detected the fault. |
| `JUNCTION_MISSED` | Expected junction was not seen in its allowed odometry window. |
| `LINK_LOST` | Sensor-frame serial link timed out; the ESP stops until frames resume. |
| `NO_ROUTE` | No valid route has been configured. |

The most useful log fields are:

- `fault_type`, `fault_active`: what the simulator injected and when.
- `esp_motor_score`, `esp_ir_score`, `esp_gru_score`: model scores (`-1`
  means the first full score window is not yet available).
- `esp_motor_alarm`, `esp_ir_alarm`, `esp_gru_alarm`, `esp_stuck_alarm`:
  detector-level alarms.
- `esp_state`: what the controller did, including stops and manoeuvres.
- `esp_distance_m`, `esp_route_index`: ESP odometry and route progress.

An alarm may be brief and not trigger the fault stop. The stop requires an
alarm to remain asserted for the configured hold time; Webots line-loss and
path-safety stops are independent mechanisms.

## What each important script does

Run scripts as Python modules from the repository root (`python -m
scripts....`) so project imports and relative paths work.

### Demo, route, and data preparation

| Script | Purpose |
|---|---|
| `scripts.run_demo` | Opens the Webots world and runs a one-off clean or preset-fault demo. Creates a temporary mission and writes a CSV; does not overwrite the mission file. |
| `scripts.maps.pathToMap` | Turns a track image into a draft graph map. |
| `scripts.maps.visualizeMap` | Creates an interactive HTML preview of a graph map. |
| `scripts.maps.route_planner` | Validates a map and computes a shortest path between named nodes. |
| `scripts.maps.make_track` | Generates the track2/track3 test layouts and corresponding map assets. |
| `scripts.telemetry.collect_route_runs` | Runs healthy Webots route/speed/seed combinations headlessly and writes per-run telemetry plus a manifest. |
| `scripts.telemetry.prepare_telemetry` | Validates telemetry, derives wheel speeds, builds time windows, splits by whole run, and fits scalers on training data. |

### Training and evaluation

| Script | Purpose |
|---|---|
| `scripts.models.train_autoencoder` | Trains the motor and IR dense or convolutional autoencoders on healthy prepared telemetry; sets thresholds and measures healthy false alarms. |
| `scripts.models.gru_experiment` | Compares longer-memory detectors, trains/evaluates the motor GRU, and configures the stuck-sensor rule. |
| `scripts.faults.collect_fault_runs` | Runs a fault matrix in Webots and records labelled telemetry. |
| `scripts.faults.evaluate_faults` | Scores model outputs on fault runs and summarizes detection, delay, pre-onset alarms, and cross-coupling. |
| `scripts.models.search_architectures` | Searches detection/latency trade-offs and evaluates selected candidates on held-out faults. |
| `scripts.models.esp_latency` | Measures candidate model inference time on the ESP using the firmware benchmark command. |
| `scripts.models.quantize_int8` | Creates experimental INT8/INT16 ESP-DL exports and compares their simulated outputs. |

### Firmware integration and checks

| Script | Purpose |
|---|---|
| `scripts.models.export_c` | Converts current PyTorch checkpoints and metadata into the generated C++ header consumed by firmware. |
| `scripts.esp.esp_serial_smoke_test` | Sends synthetic protocol frames and checks ESP control/state responses without Webots. |
| `scripts.esp.replay_scores` | Replays recorded sensor/encoder data through the ESP and compares scores/alarms with Python; requires raw run CSVs and the board. |
| `scripts.esp.resource_usage` | Summarizes ESP resource logs. |
| `scripts\esp\flash_wifi.ps1` | Writes WiFi/ntfy settings to the ESP configuration partition. |
| `scripts.esp.wifi_status` | Checks WiFi status and optionally sends a test notification. |
| `scripts.esp.int8_bench` | Benchmarks experimental quantized models on the separate ESP-DL firmware. |

The `notebooks\ETL.ipynb` notebook is exploratory, not part of the current
command-line training/deployment path. The full command and option reference
is in [USAGE.md](USAGE.md).

## Optuna architecture search: what was searched?

There are two different counts; they should not be confused:

- **967 architecture configurations were benchmarked on the ESP32-S3** by
  `scripts.models.esp_latency`. The firmware benchmark ran each candidate
  with random weights in internal RAM and recorded the median of 31 timing
  repetitions. This measured execution time; it did **not** train 967 models.
- **92 Optuna trials trained candidate networks**: 40 motor convolutional
  autoencoder trials, 40 IR convolutional autoencoder trials, and 12 motor
  GRU trials. Each count includes trial 0, the deployed architecture used as
  the search baseline.

The Optuna search uses multi-objective TPE. Each candidate is trained only on
healthy training telemetry, then evaluated for detection on the tuning half
of the labelled fault runs. The search rewards more detected faults (using
median delay as a tie-break) and lower ESP inference latency from the 967-entry
benchmark table. The tuning examples are route/speed pairs A at 2.4, B at 3.4,
C at 2.6, and D at 3.1 rad/s. The other speed for each route is held out from
the search, as are the healthy test runs.

The 92 search trials use a reduced training budget: at most 60 epochs using
every second training window for the convolutional models, and 2,000 training
iterations for the GRU. Two selected candidates per detector family (six
candidate networks total) were then retrained with the full budget—up to 300
epochs or 4,000 GRU iterations—and compared against the deployed baselines on
the held-out faults and healthy test runs.

| Family | Deployed baseline -> selected candidate 0 | Search result | Deployment decision |
|---|---|---|---|
| Motor conv | Window 20, 16 channels -> window 10, 8 channels | Held-out detections 25/28 -> 28/28; median delay 0.70 -> 0.40 s; ESP time 307 -> 100 us; healthy test alarms 2/18 for both. | Candidate 0 deployed in `models\esp-conv-v2`. |
| IR conv | Window 20, 16 channels -> window 10, 8 channels | Held-out detections 17/32 -> 22/32; median delay 2.75 -> 2.48 s; ESP time 294 -> 100 us; healthy test alarms 0/18 -> 2/18. | Candidate not deployed: the extra healthy test alarms were not accepted; the original IR model remains in `esp-conv-v2`. |
| Motor GRU | 32 hidden units -> 16 hidden units | Held-out detections 27/28 -> 28/28; median delay 0.16 -> 0.13 s; ESP time 267 -> 127 us; healthy test alarms 0/18 for both. | Candidate 0 deployed in `models\esp-gru-v2`. |

This is why the current deployed set has the smaller searched motor conv and
GRU, but keeps the prior IR conv. It is also why "best model" depends on the
trade-off: maximizing fault detections alone would choose the smaller IR
candidate, while including healthy false alarms makes that choice less clear.
Full search method and tables are in
[REPORT.md, architecture search](../REPORT.md#16-architecture-search-optuna);
the archived study databases, trial records, selected candidates, and results
are under `reports\arch-search-v1`.

To run a fresh search, the measured latency table and local raw healthy/fault
run CSVs must be available. The latency sweep requires the ESP; the Optuna
training/search then uses the saved timing table and CPU training. Use a new
output directory to keep the archived search intact:

```powershell
# Optional: regenerate the on-device timing table (requires the ESP on COM3)
.\.venv\Scripts\python.exe -m scripts.models.esp_latency `
  --port COM3 --out reports\esp-latency-repeat

# Train/search 40 motor conv, 40 IR conv, and 12 motor GRU candidates
.\.venv\Scripts\python.exe -m scripts.models.search_architectures `
  --model motor --trials 40 --out reports\arch-search-repeat
.\.venv\Scripts\python.exe -m scripts.models.search_architectures `
  --model ir --trials 40 --out reports\arch-search-repeat
.\.venv\Scripts\python.exe -m scripts.models.search_architectures `
  --model gru --trials 12 --out reports\arch-search-repeat
```

If you regenerate the latency table into a different directory, pass its
`latency.csv` with `--latency reports\esp-latency-repeat\latency.csv` on each
search command. Search runtime depends on the machine; the recorded run used
the historical timings and trial data already saved in
`reports\arch-search-v1`.

The full-budget held-out comparison of the archived finalists can be rerun
separately (also requires the raw healthy and fault run CSVs):

```powershell
.\.venv\Scripts\python.exe -m scripts.models.search_architectures `
  --final reports\arch-search-v1\chosen_conv.json `
  --final-models motor ir `
  --out reports\arch-search-final-repeat
.\.venv\Scripts\python.exe -m scripts.models.search_architectures `
  --final reports\arch-search-v1\chosen_gru.json `
  --final-models gru `
  --out reports\arch-search-final-repeat
```

## Train or re-evaluate

The checked-in prepared ESP healthy dataset can be used to retrain the
convolutional models (this changes output files, so do not run it casually if
you want to preserve the checked-in weights):

```powershell
.\.venv\Scripts\python.exe -m scripts.models.train_autoencoder `
  --dataset data\datasets\esp-baseline-v1 `
  --arch conv `
  --out models\esp-conv-experiment `
  --threshold-rule no-validation-alarm
```

For training, use healthy training runs only. Do not mix the fault evaluation
runs into the healthy training split. GRU retraining, offline fault
evaluation, replay, and architecture search need raw run CSVs; those CSVs are
git-ignored and are not included in a fresh clone. Existing prepared datasets,
model weights/configurations, manifests, and reports are tracked. See
[USAGE.md](USAGE.md) before collecting or evaluating additional runs.

The project unit tests do not need Webots or an ESP:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_*.py" -v
```

## Current gaps and interpretation

- IR drift and low-level IR noise remain difficult. Reported IR-only detection
  is about 4-6/8 for drift and 0-3/8 for noise severity 100 or below, depending
  on target/run set. IR noise can also trigger a motor alarm because it alters
  PD steering.
- The motor detector has persistent alarms on 2/18 healthy test runs. A
  no-alarm validation threshold does not guarantee zero alarms on new runs.
- Some injected faults begin during a U-turn. Manoeuvres are intentionally
  unmonitored; if the robot then loses the line before normal monitoring
  resumes, the safety stop is `LINE_LOST`, not a model detection.
- The healthy training data lacks some base-track parallel-edge driving.
  Short alarms can occur on those manoeuvres, although observed ones are
  shorter than the fault-stop hold.
- Track3's hairpin at the highest tested speed approaches the line-search
  timeout. Pull-over near junctions can finish close to another track line.
- Serial reconnect has been tested with a simulated/fake drop, not yet with a
  real unplug during a run.
- No evaluation here substitutes for testing real motors, encoders, ambient
  light changes, battery variation, wheel slip, or safety behavior on a
  physical robot.

For full result tables, version history, and limitations, consult
[REPORT.md](../REPORT.md), not only the summary figures above.
