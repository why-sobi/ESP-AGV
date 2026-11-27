# ESP-AGV

A simulation-first AGV project using the e-puck robot in Webots. Webots remains
the environment; an ESP32-S3 on USB serial is the robot's onboard computer. It
runs the line control, four anomaly detectors (motor and IR conv autoencoders,
a motor GRU and a stuck-sensor rule), and on a fault pulls over, stops and
sends a phone notification.

**[REPORT.md](REPORT.md)** describes the whole project, its decisions and every
result.

- Follow the track with PD line control and map-based route planning.
- Collect simulated motor/encoder and ground-IR telemetry.
- Train motor and IR autoencoders on healthy telemetry and evaluate injected
  faults.
- Send telemetry and route actions from Webots through a Python/USB-serial
  bridge to the ESP32-S3; apply returned wheel commands in Webots.

## Repository layout

```
data/          Run manifests and prepared datasets (raw run logs are git-ignored)
docs/          Design notes, status, TODO, and usage guide
models/        Trained detectors (deployed: esp-conv-v2, esp-gru-v2)
reports/       Evaluation results referenced by REPORT.md
firmware/      ESP-IDF firmware and serial controller prototype
maps/          Track images (base, track2, track3), graph maps and missions
scripts/       Automation tools grouped by purpose
tests/         Python regression tests
notebooks/     Exploratory analysis notebooks
simulation/    Webots worlds (one per track) and robot controllers
```

## Setup

Create a virtual environment and install the dependencies:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```
For Webots, confirm `simulation/controllers/epuck_data/runtime.ini` and
`simulation/controllers/epuck_route/runtime.ini` point to the project Python
interpreter.

To run the simulation, open `simulation/worlds/Self Diagnosis AGV.wbt` in Webots
(`... track2.wbt` / `... track3.wbt` for the other tracks, with
`maps/routes/mission-track2.json` / `mission-track3.json`).

**With the ESP32-S3:** install ESP-IDF 6.0.1, then build and flash `firmware/`
(`idf.py build`, `idf.py -p COM3 flash`; the first build downloads esp-dsp).
For phone notifications copy `firmware/wifi.example.csv` to `firmware/wifi.csv`
(git-ignored), fill in the WiFi network and an ntfy topic, and write it with
`scripts\esp\flash_wifi.ps1`. Set the mission's `esp_serial_port` and
`control_mode: esp32`. Details: [firmware/README.md](firmware/README.md).

Raw run logs (`route-*.csv`, `controller-errors.log`, `data/SensorData*.csv`) are
git-ignored and not in the repository; the manifests, the prepared datasets
(`data/datasets`), the models and all reports are. Replaying recorded runs
(`replay_scores`), the offline fault evaluation and GRU/search retraining
need the raw logs; driving, detecting, exporting and conv training do not.


## Usage

Convert a track image to a draft graph map:

```powershell
python -m scripts.maps.pathToMap -img "maps\image\base track.png" -out "maps\paths"
```

Preview the map in a browser:

```powershell
python -m scripts.maps.visualizeMap -map "maps\paths\base track.json"
```

Plan a route between two nodes:

```powershell
python -m scripts.maps.route_planner "maps\paths\base track.json" node-001 node-004
```

Run the tests:

```powershell
python -m unittest discover -s tests -p "test_*.py" -v
```

See [scripts/README.md](scripts/README.md) for a quick script/test reference
and [docs/USAGE.md](docs/USAGE.md) for the full workflows.

## Status

- **Navigation:** the map planner drives the Webots route controller.
- **ESP control:** with `control_mode: esp32`, the ESP32-S3 runs the PD
  line-following loop over COM3. Junctions are identified with wheel odometry
  (per-edge scales), and Webots applies the ESP's wheel commands. Routes
  complete as round trips on three tracks, stopping 0.1-2.0 cm from the node:
  the base track (incl. its parallel edges), track2 (90° corners, thick
  lines) and track3 (45° bends, a Y junction, a hairpin, a tight curve).
- **Anomaly detection on the ESP:** a motor conv autoencoder and motor GRU
  (`models/esp-conv-v2`, `esp-gru-v2`, from the architecture search), an IR
  conv autoencoder and a stuck-sensor rule run every frame in ~0.81 ms
  (esp-dsp).
  - On 120 labelled fault runs: 94/120 by the fault's own detector, 110/120
    by either; no alarm before a fault's onset; healthy test false alarms
    motor 2/18, IR 0/18.
  - The ESP's scores and alarms agree with Python (replay, identical alarms).
- **Fault response:** on a persistent alarm the robot pulls over to the
  map-chosen side, stops and sends a phone notification. Live: every injected
  weak-motor, stuck-IR and dead-IR fault stopped the robot on all three
  tracks (track2 20/20, track3 16/16, nothing trained there).
- **Earlier models:** `conv-v1`/`dense-v1` were trained on Supervisor-driven
  runs. They do not transfer to ESP control.
- **Open:** IR drift / light IR noise detection, a real-unplug test of the
  COM3 reconnect, the track3 hairpin margin at 3.5 rad/s. See
  [handoff.md](handoff.md).

See [docs/USAGE.md](docs/USAGE.md) for commands and project-file workflows,
[docs/STATUS.md](docs/STATUS.md) for verified scope, and
[docs/TODO.md](docs/TODO.md) for the next steps.
