# Navigation Decisions and Lessons Learned

## Current direction

The AGV should navigate a **known, mostly fixed track map** rather than infer
route intent from three line sensors. Represent the track as a graph of named
nodes and named connections. Given a start and destination, a planner computes
a route; the robot driver follows that route while the line follower handles
local steering. This extends the original line-following project; it does not
replace line following.

Anomaly detection remains an independent project objective. Motor and IR
autoencoders train on healthy Webots telemetry and monitor simulated runs while
the route planner chooses where to go. Route and operating-mode context should
be recorded so expected path/speed variation is not mislabeled as a fault.
Healthy training, validation, and test runs should cover the intended
simulation routes and speeds.

The agreed target is **simulation-only operation with an ESP32-S3 compute
target**: Webots produces sensor/encoder telemetry; a Python bridge streams it
to the ESP over USB serial; the ESP computes control and, eventually, anomaly
scores; the bridge applies returned wheel commands in Webots.

The planner and driver should exchange a versioned route message containing
node and edge IDs, for example:

```json
{
  "map_id": "factory-floor-1",
  "map_version": 1,
  "start": "loading",
  "destination": "station-b",
  "nodes": ["loading", "junction-1", "station-b"],
  "steps": [
    {"edge_id": "loading-to-junction-1", "from": "loading", "to": "junction-1", "speed_limit": 3.0},
    {"edge_id": "junction-1-to-station-b", "from": "junction-1", "to": "station-b", "speed_limit": 2.0}
  ],
  "total_distance": 12.5
}
```

For the ESP simulation loop, the route planner's graph output must be converted
to relative junction actions and transmitted by the Python bridge. Webots
Supervisor pose may be used for simulation route following, while the ESP's
prototype route follower uses relative junction actions and IR readings. The
live bridge and graph-to-action conversion are not implemented yet.

The planner's input is a graph description; distances and node coordinates must
come from a surveyed/calibrated map, not guesses from the texture:

```json
{
  "map_id": "factory-floor-1",
  "version": 1,
  "nodes": [
    {"id": "loading", "name": "Loading"},
    {"id": "junction-1", "name": "Main junction"},
    {"id": "station-b", "name": "Station B"}
  ],
  "edges": [
    {"id": "loading-to-junction-1", "from": "loading", "to": "junction-1", "distance": 8.0, "bidirectional": true},
    {"id": "junction-1-to-station-b", "from": "junction-1", "to": "station-b", "distance": 4.5, "speed_limit": 2.0, "bidirectional": false}
  ]
}
```

Once a surveyed map exists, the planner can be called with
`python scripts/route_planner.py <map.json> <start-id> <destination-id>`.

## Implemented foundation

- `scripts/pathToMap.py` converts a dark-path raster to a **draft** graph and
  saves `<image-name>.json` in the requested output directory. Its generated
  pixel coordinates, skeleton, node clusters, and edge geometry need human
  review; the image-to-Webots transform is not calibrated automatically.
- `scripts/visualizeMap.py -map <map.json>` writes a standalone HTML preview
  under the sibling `maps/visualizations/` directory by default. It displays
  edge centerlines, node/edge IDs, coordinates,
  and (when it can locate the source image) a faded image overlay. Zoom/pan in
  the browser, edit the JSON in an editor, then rerun the command to refresh.
- `maps/paths/base track.json` is the current draft generated from
  `maps/image/base track.png`. It is not yet a surveyed Webots-world map.
  The current extraction has four candidate junction nodes and six edges,
  including parallel connections between some nodes. The configured route was
  visually checked and tested, but all candidate nodes, edges, and alternate
  route semantics still need review before being treated as validated.
- `scripts/route_planner.py` loads and validates named-node maps, then computes
  the minimum-distance route with Dijkstra's algorithm. Edges can be one-way or
  bidirectional and may carry a speed limit. It emits JSON to stdout or a file
  with `-out`.
- `scripts/test_route_planner.py` exercises loops, one-way connections,
  unreachable destinations, invalid map data, and the zero-length route case.
- `simulation/controllers/epuck_route/epuck_route.py` connects the graph planner
  to a Supervisor-enabled Webots e-puck. It follows planned edge centerlines,
  uses ground sensors for local line correction and line-loss stopping, and
  records route telemetry. The previous sensor-only controller remains as
  `epuck_data`.
- `simulation/controllers/epuck_data/epuck_data.py` logs each PD session to a
  unique CSV under `data/sensor_runs`, preserving run boundaries.
- `scripts/prepare_telemetry.py` derives actual wheel velocities, creates
  separate motor/IR windows, splits by run before windowing, fits scalers on
  training data only, and rejects exact duplicate runs. The pilot dataset has
  one unique training run and one different-speed validation run; there is no
  final test set or model yet.
- `esp32-s3/` is an ESP-IDF project for the ESP32-S3. Its current firmware
  accepts route actions and three simulated IR readings over serial, runs the
  PD/junction prototype, and returns wheel targets. A standalone COM3 smoke
  test passed. The Webots bridge, motor telemetry inputs, and anomaly inference
  are still future work.
- The current Webots route integration is still a prototype: its pixel-to-world
  transform uses the explicitly configured 1 m by 1 m arena. A headless
  Webots run completed the two-edge mission with all ground sensors seeing the
  line and a maximum recorded centerline error of 0.0228 m. The user also
  visually confirmed the configured route. This validates that tested
  simulation setup, not every map route.
- `docs/USAGE.md` describes the converter, visualizer, planner, and test
  commands. Use it as the quick-start workflow; this file records the design
  decisions and failure history.

## Decisions

1. **Use a named graph, not raw image interpretation, as the planner input.**
   A bitmap is useful for drawing and surveying a track, but a graph makes
   junction identity, directionality, distance, speed limits, and destinations
   explicit and testable.
2. **Start with shortest distance; keep edge metadata for later constraints.**
   Dijkstra's algorithm is simple and sufficient for the current graph. Traffic,
   priority, time, battery, or fault constraints can later change the edge cost
   without changing the route-message contract.
3. **Keep path planning separate from low-level line following and transport.**
   The route planner chooses the path, the controller follows it, and the
   transport only carries commands/telemetry. The planned Webots-to-ESP link is
   USB serial via a Python bridge.
4. **Do not treat a sensor pattern as a globally unique junction ID.**
   Three ground IR sensors can indicate local line position, but the same
   pattern may occur on a curve, a broad section, or multiple junctions.
5. **Keep navigation and anomaly detection as parallel layers.**
   Route planning selects the intended path; line following controls motion on
   it; anomaly detection monitors the control/telemetry behavior. Navigation
   metadata should be logged so healthy route variation is represented during
   model training and evaluation.

## Failure points observed

- The original threshold controller switched wheel commands abruptly and made
  high-frequency, jerky steering corrections.
- The PD controller's derivative term amplified sudden sensor-pattern changes.
  At a 32 ms timestep, a one-step error change could create a large command
  correction. High steering relative to base speed also made low-speed behavior
  worse by allowing a wheel command to reach or cross zero.
- In the logged PD runs, all three sensors sometimes reported the dark path for
  seconds at a time. That is not enough information to distinguish a junction
  from a wide/overlapping line segment or the circular feature.
- A first junction heuristic interpreted simultaneous side-sensor readings as
  a branch and latched a straight/side choice. In the circular section those
  local readings recur, so the robot could repeat the loop and exit incorrectly.
  Updating "last turn" from ordinary tracking also made branch choices depend
  on incidental sensor offsets.
- Starting pose can bias initial sensor readings, but it does not explain or
  solve repeated route-choice errors by itself.
- Earlier logs showed uniform 32 ms sampling, but some runs were appended to
  shared filenames. Keep each run identifiable when comparing tuning or routes.

## Remaining work

1. Validate the reviewed graph semantics and branch choices at all junctions;
   the initial Webots run tested only one route.
2. Refine the generated graph and names against the visualized path; pixel
   skeleton extraction can merge nearby segments or produce parallel edges.
3. Keep route tracking/localization behind an interface that can be replaced
   for hardware; the current Supervisor controller is simulation-only.
4. Validate routes through the circular feature and multi-branch junctions,
   including unreachable destinations and changed maps.
5. Collect diverse healthy Webots runs and controlled fault runs; train and
   evaluate the motor/IR autoencoders.
6. Deploy preprocessing and inference to the ESP32-S3 and compare its scores
   with the Python reference.
7. Implement and validate the Webots Python serial bridge, route-action
   conversion, and end-to-end closed-loop behavior.
