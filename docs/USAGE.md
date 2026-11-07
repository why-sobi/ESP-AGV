# Project Usage

Run commands from the repository root in PowerShell. The image tools use the
project virtual environment, which includes Pillow and NumPy:

```powershell
.\.venv\Scripts\Activate.ps1
```

Or invoke `.\.venv\Scripts\python.exe` directly in each command.

## Convert a path image to a draft map

The converter treats dark pixels as the path, extracts an approximate centerline,
and saves a JSON file using the image's basename:

```powershell
.\.venv\Scripts\python.exe scripts\pathToMap.py `
  -img "maps\image\base track.png" `
  -out "maps\paths"
```

This creates `maps\paths\base track.json`. Useful options:

```powershell
--threshold 128          # grayscale threshold, from 0 to 255
--max-dimension 600      # processing size; higher retains more detail
--pixels-per-meter 100   # optional calibrated scale for edge distances
```

Do not pass a guessed scale. Without calibration, geometry and distances remain
in image pixels, and the JSON explicitly marks the Webots transform as
uncalibrated. Conversion is a starting point, not automatic map surveying.

## Visualize and edit the map

Generate a standalone HTML preview beside the JSON:

```powershell
.\.venv\Scripts\python.exe scripts\visualizeMap.py `
  -map "maps\paths\base track.json"
```

Open `maps\paths\base track.html` in a browser. Scroll to zoom, drag to pan,
and hover over nodes or edges to see details. If the source image is available
in a nearby `image` or `textures` folder, it appears as a faded background.

Edit the JSON in a text editor, then rerun the visualization command to refresh
the preview. Hand edits commonly include:

- Give automatically generated node IDs meaningful, stable names.
- Move, add, or remove nodes to match real junctions, endpoints, and stations.
- Correct each edge's `from` and `to` node IDs, centerline geometry, and
  `bidirectional` flag.
- Set calibrated edge `distance` values and optional `speed_limit` values.

The map uses:

- `nodes`: unique `id` plus numeric `x` and `y` coordinates; optional `name`.
- `edges`: unique `id`, `from`, `to`, positive `distance`, optional
  `speed_limit`, `bidirectional` boolean, and optional `geometry` as an ordered
  list of `[x, y]` points.
- `coordinate_system`: the origin and units used by the coordinates.

Keep all coordinates in one coordinate system. For Webots execution, the image
coordinates still need a calibrated transform to world coordinates.

## Plan a route

The planner accepts a map JSON and start/destination node IDs:

```powershell
.\.venv\Scripts\python.exe scripts\route_planner.py `
  "maps\paths\base track.json" node-001 node-004
```

It prints a JSON route containing the ordered node IDs and edge steps. Dijkstra's
algorithm minimizes the sum of edge `distance` values. At present, `speed_limit`
is carried into route output but does not affect route selection.

The draft map IDs are temporary: inspect the preview and use the exact node IDs
present in the JSON. The output is useful for checking graph connectivity, but
the current planner is not connected to the Webots robot driver.

## Current Webots and anomaly-detection status

The current Webots controller is a line-following and telemetry-logging
prototype. It does not yet load the map or execute planned routes. Webots
Supervisor localization, route progress detection, and the route-to-controller
interface still need implementation and simulation testing.

The original anomaly-detection objective remains separate from route planning:
collect healthy data while line following, derive motor velocities from wheel
odometry, build temporal windows, train the motor and IR autoencoders, and
validate thresholds using fault-injected runs. Include route and speed context
when collecting data so expected differences between paths are not mislabeled
as faults.

## Run tests

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s scripts -p "test_*.py" -v
```

The tests cover image conversion, map visualization, and graph route planning;
they do not test physical/Webots route following.
