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
firmware/      ESP-IDF firmware and serial controller prototype
maps/          Track image and generated graph maps
scripts/       Automation tools grouped by purpose
tests/         Python regression tests
notebooks/     Exploratory analysis notebooks
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
- **Anomaly models:** 64 healthy runs (4 routes x 16 speeds) train motor and
  IR autoencoders. They are evaluated on 184 labelled fault runs, including a
  severity sweep.
- **ESP control:** with `control_mode: esp32`, the ESP32-S3 runs the PD
  line-following loop over COM3. Junctions are identified with wheel odometry,
  and Webots applies the ESP's wheel commands. All four two-edge routes
  complete, stopping within 1.4 cm of the destination.
- **Not yet implemented:** anomaly inference on the ESP.

See [docs/USAGE.md](docs/USAGE.md) for commands and project-file workflows,
[docs/STATUS.md](docs/STATUS.md) for verified scope, and
[docs/TODO.md](docs/TODO.md) for the next steps.
