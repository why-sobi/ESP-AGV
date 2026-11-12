# ESP-AGV

A simulation-first AGV project using the e-puck robot in Webots. Webots remains
the environment; an ESP32-S3 is being integrated as an external compute target
for line control and anomaly inference.

- Follow the track with PD line control and map-based route planning.
- Collect simulated motor/encoder and ground-IR telemetry.
- Train motor and IR autoencoders on healthy telemetry and evaluate injected
  faults.
- Send telemetry and route actions from Webots through a Python/USB-serial
  bridge to the ESP32-S3; apply returned wheel commands in Webots.

## Repository layout

```
data/          Logged sensor data (CSV)
docs/          Design notes, status, TODO, and usage guide
esp32-s3/      ESP-IDF firmware and serial controller prototype
maps/          Track image and generated graph maps
scripts/       Map tools, route planner, telemetry preparation, and tests
simulation/    Webots world and robot controllers
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

To run the simulation, open `simulation/worlds/Self Diagnosis AGV.wbt` in Webots. 


## Usage

Convert a track image to a draft graph map:

```powershell
python scripts\pathToMap.py -img "maps\image\base track.png" -out "maps\paths"
```

Preview the map in a browser:

```powershell
python scripts\visualizeMap.py -map "maps\paths\base track.json"
```

Plan a route between two nodes:

```powershell
python scripts\route_planner.py "maps\paths\base track.json" node-001 node-004
```

Run the tests:

```powershell
python -m unittest discover -s scripts -p "test_*.py" -v
```

See [docs/USAGE.md](docs/USAGE.md) for more detail.

## Status

The map planner is connected to the Webots route controller, and the configured
two-edge mission has been visually and headlessly tested. The ESP32-S3 firmware
has been flashed and its serial controller smoke-tested. Telemetry preparation
works, but the current dataset is a small pipeline pilot: model training,
inference deployment, and the Webots-to-ESP bridge are not yet implemented.

See [docs/USAGE.md](docs/USAGE.md) for commands and project-file workflows,
[docs/STATUS.md](docs/STATUS.md) for verified scope, and
[docs/TODO.md](docs/TODO.md) for the next steps.
