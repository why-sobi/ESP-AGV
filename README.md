# ESP-AGV

A line-following AGV (automated guided vehicle) project built on the e-puck robot in Webots. The goals are:

- Follow a line track with a PD controller.
- Log motor and ground IR sensor telemetry for anomaly detection.
- Plan routes over a named graph of the track.

## Repository layout

```
data/          Logged sensor data (CSV)
docs/          Design notes, status, TODO, and usage guide
maps/          Track image and generated graph maps
scripts/       Map tools, route planner, ETL notebook, and tests
simulation/    Webots world and robot controllers
```

## Setup

Create a virtual environment and install the dependencies:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```
**IMPORTANT**: In `simulation/controller/epuck_data/runtime.ini` update the python path to .venv   

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

The route planner is not connected to the Webots controller yet, and the track map is still an uncalibrated draft. See [docs/STATUS.md](docs/STATUS.md) and [docs/TODO.md](docs/TODO.md) for progress and next steps.
