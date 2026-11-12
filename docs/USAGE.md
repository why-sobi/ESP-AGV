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

Generate a standalone HTML preview in the sibling `visualizations` folder:

```powershell
.\.venv\Scripts\python.exe scripts\visualizeMap.py `
  -map "maps\paths\base track.json"
```

Open `maps\visualizations\base track.html` in a browser. Scroll to zoom, drag to pan,
and hover over nodes or edges to see details. If the source image is available
in a nearby `image` or `textures` folder, it appears as a faded background.

Use `-out-dir <folder>` to choose another preview folder, or `-out <file.html>`
to set an exact output file.

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
`-out` can save the route as JSON for reuse:

```powershell
.\.venv\Scripts\python.exe scripts\route_planner.py `
  "maps\paths\base track.json" node-003 node-001 `
  -out "maps\routes\active_route.json"
```

### Run the route in Webots

The current world is configured with a Supervisor-enabled e-puck and the
`epuck_route` controller. Edit `maps\routes\mission.json` to select
`start_node` and `destination_node`, then open
`simulation\worlds\Self Diagnosis AGV.wbt` in Webots and start the simulation.
The controller calculates the shortest route, spawns the robot at the selected
start node for repeatable simulation tests, follows the edge centerlines, uses
ground IR readings for local correction/line-loss stopping, and writes telemetry
to `data\route_runs`.

The current map-to-world transform assumes the image fills the 1 m by 1 m
arena. It is explicitly marked uncalibrated; the mission allows that transform
for simulation testing only. Check the robot against the track in Webots before
trusting route execution. Set `spawn_at_start_node` to `false` to disable the
simulation-only teleport, but then the robot must already be positioned at the
configured start node. This Supervisor controller is not a hardware driver;
physical hardware needs a localization adapter and its own safe start logic.

The previous sensor-only controller remains available as `epuck_data` for
line-following experiments without map navigation.

## Current Webots and anomaly-detection status

The route-aware Webots controller now loads the draft graph and mission, runs
the shortest-path planner, uses Supervisor pose to track the planned
centerlines, and logs route telemetry. It still needs visual Webots validation
because image-to-world calibration has not been confirmed. This is a
simulation-only first integration, not yet validated for hardware.

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
