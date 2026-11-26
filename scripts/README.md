# Scripts quick guide

Run commands from the repository root. Use `python -m ...` so the tools can
share imports and find project files consistently. The examples below use the
project virtual environment in PowerShell.

| Script | What it does | Use it when |
|---|---|---|
| `maps/pathToMap.py` | Converts a track image into a draft graph-map JSON. | Starting a map from a new track image. |
| `maps/visualizeMap.py` | Creates an interactive HTML preview of a graph map. | Reviewing or correcting a generated/edited map. |
| `maps/route_planner.py` | Validates a graph and plans a shortest route between nodes. | Checking a route or writing a route JSON. |
| `telemetry/collect_route_runs.py` | Runs Webots headlessly over route/speed (and jitter-seed) combinations and records healthy telemetry; `--control-mode esp32 --round-trip` for ESP-driven round trips. | Collecting more healthy simulation runs. |
| `telemetry/prepare_telemetry.py` | Validates run CSVs, builds motor/IR windows, splits by run, and fits training scalers. | Preparing model input from telemetry. |
| `models/train_autoencoder.py` | Trains dense or convolutional motor/IR autoencoders, sets thresholds and evaluates healthy-run alarms (`--reuse-weights` re-evaluates without training). | Training or comparing models. |
| `models/export_c.py` | Writes a trained conv model pair to `firmware/main/anomaly_model.h`. | After retraining, before reflashing the ESP. |
| `faults/collect_fault_runs.py` | Runs Webots fault scenarios from a fault matrix and records labelled telemetry. | Collecting fault-evaluation data. |
| `faults/evaluate_faults.py` | Scores fault runs and summarizes detection, delay, false alarms, and cross-coupling. | Evaluating trained models on fault data. |
| `esp/esp_serial_smoke_test.py` | Sends synthetic sensor frames to the ESP over serial and checks its responses. | Checking the flashed firmware; requires the ESP, a COM port, and `pyserial`. |
| `esp/replay_scores.py` | Replays recorded ESP runs through the ESP and compares its wheel targets, anomaly scores and alarms with Python. | Verifying on-ESP inference after a firmware or model change; requires the ESP. |

Example:

```powershell
.\.venv\Scripts\python.exe -m scripts.maps.route_planner `
  "maps\paths\base track.json" node-003 node-001
```

The full options and workflows are in [`docs/USAGE.md`](../docs/USAGE.md).
`notebooks/ETL.ipynb` is an exploratory notebook, not part of the current
command-line pipeline.

## Tests

Run the Python unit tests from the repository root:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_*.py" -v
```

`unittest` discovers files named `test_*.py` and runs their `TestCase` methods.
The tests cover map conversion/visualization and route planning, telemetry
validation and splitting, model training/persistence behavior, and fault-score
evaluation. They use small test inputs; they do not launch Webots or need the
ESP connected.

The ESP serial smoke test is a separate hardware/protocol check, not a unit
test. The Webots collection scripts are also integration tools and launch
Webots when run; neither is run by the unit-test command.
