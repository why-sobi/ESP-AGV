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

For the ESP simulation loop, the route planner's graph output is converted to
relative junction actions, each tagged with its expected odometry distance,
and sent by the `epuck_route` controller acting as the Python bridge. The
Supervisor uses ground-truth pose only for placement, progress tracking,
safety checks and logging. The ESP steers from the IR readings and identifies
junctions with its wheel odometry.

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
`python -m scripts.maps.route_planner <map.json> <start-id> <destination-id>`.

## Implemented foundation

- `scripts/maps/pathToMap.py` converts a dark-path raster to a **draft** graph and
  saves `<image-name>.json` in the requested output directory. Its generated
  pixel coordinates, skeleton, node clusters, and edge geometry need human
  review; the image-to-Webots transform is not calibrated automatically.
- `scripts/maps/visualizeMap.py -map <map.json>` writes a standalone HTML preview
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
- `scripts/maps/route_planner.py` loads and validates named-node maps, then computes
  the minimum-distance route with Dijkstra's algorithm. Edges can be one-way or
  bidirectional and may carry a speed limit. It emits JSON to stdout or a file
  with `-out`.
- `tests/test_route_planner.py` exercises loops, one-way connections,
  unreachable destinations, invalid map data, and the zero-length route case.
- `simulation/controllers/epuck_route/epuck_route.py` connects the graph planner
  to a Supervisor-enabled Webots e-puck. It follows planned edge centerlines,
  uses ground sensors for local line correction and line-loss stopping, and
  records route telemetry. The previous sensor-only controller remains as
  `epuck_data`.
- `simulation/controllers/epuck_data/epuck_data.py` logs each PD session to a
  unique CSV under `data/sensor_runs`, preserving run boundaries.
- `scripts/telemetry/prepare_telemetry.py` derives actual wheel velocities, creates
  separate motor/IR windows, splits by run before windowing, fits scalers on
  training data only, and rejects exact duplicate runs. The current dataset
  is `sim-baseline-v4` (64 healthy runs with train/validation/test splits),
  and trained models are in `models/` (see `docs/STATUS.md`).
- `firmware/` is an ESP-IDF project for the ESP32-S3. It accepts a
  distance-tagged route and per-step IR and encoder readings over serial, runs
  the PD loop and odometry-gated junction actions, and returns wheel targets.
  In `esp32` mode, `epuck_route` drives Webots with those targets; routes A-D
  complete. Anomaly inference is still future work.
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
6. **Identify junctions on the ESP with odometry, not the sensor pattern alone
   (2026-11-14).** The PD loop still does all steering. Wheel odometry only
   decides *which* junction the robot is at:
   - The Supervisor sends each action with its expected odometry distance.
   - The ESP accepts the all-dark junction pattern only within +/-0.10 m of
     that distance.
   - It stops at the destination distance.
   - Distances are computed from the map on every run, so a rescaled world
     needs only a correct `world_transform`.

   Alternatives considered:
   - Track markers beside the line (sensor-only, but changes the texture and
     the healthy data).
   - Fixing the arena aspect ratio (invalidates all recorded runs and models,
     and still leaves a T-junction invisible when approached along a wide
     line).
   - Letting the Supervisor signal junctions (moves junction detection to
     ground truth).

   The encoder readings this needs are also inputs to the motor autoencoder.
7. **Treat the ESP's link timeout as a pause, not a latched fault, in
   simulation.** Webots stops in wall-clock time while starting up or under
   load, and simulation time stops with it. The ESP outputs zero wheel targets
   after 250 ms without frames and resumes the route when frames return.
8. **Turn around on the ESP for the return leg.** At the destination the
   robot faces away from home, and line following cannot reverse direction.
   - The return route starts with a `U` step: the ESP spins left in place,
     measures the turn from the encoders, and hands over to the PD loop once
     it has turned at least 160 degrees and the center sensor is on the line.
   - The angle gate makes it ignore branches swept earlier in the spin.
   - Teleporting the robot with the Supervisor was not considered, because
     the ESP would no longer be driving.

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
- **Wide horizontal lines.** The 2400x1800 track image is stretched onto a
  1 m x 1 m arena (0.417 mm/px horizontally, 0.556 mm/px vertically), so
  horizontal lines are about a third wider than vertical ones. On horizontal
  stretches all three ground sensors can see dark track.
  - The first ESP routing attempt counted junctions from that pattern alone.
    It took route A's `R` correctly at node-002, then reported the
    destination 0.24 m early on the straight before node-001.
  - In the healthy runs, the final 0.24 m into node-001 reads all-dark the
    whole way.
  - On horizontal stretches the side sensors also flicker around the
    threshold, so "all-dark starts here" occurs dozens of times.
- **Odometry vs map distance.** Wheel odometry reads about 0.92 of the map
  centerline length on routes A-D, because the robot cuts inside the
  semicircle and the corner. The all-dark pattern at a junction appears about
  5-7 cm of odometry before the scaled map distance, which the +/-0.10 m
  window covers.
- **First ESP run failures.**
  - The flashed firmware stopped with `LINE_LOST` on a single all-light
    frame at 0.16 s, which the Supervisor controller rides through.
  - The UART read waited for 64 bytes or 50 ms, so every 35-byte frame was
    handled 50 ms late.
  - The ESP's wall-clock link timeout latched `LINK_LOST` whenever Webots
    paused. It then sent an extra reply that the controller read as a
    sequence mismatch.

## Remaining work

1. Validate the reviewed graph semantics and branch choices at all junctions;
   the initial Webots run tested only one route.
2. Refine the generated graph and names against the visualized path; pixel
   skeleton extraction can merge nearby segments or produce parallel edges.
3. Keep route tracking/localization behind an interface that can be replaced
   for hardware; the current Supervisor controller is simulation-only.
4. Validate routes through the circular feature and multi-branch junctions,
   including unreachable destinations and changed maps.
5. Done: healthy and fault runs collected, motor/IR autoencoders trained and
   evaluated (see `docs/STATUS.md`). Check them on ESP-controlled runs.
6. Deploy preprocessing and inference to the ESP32-S3 and compare its scores
   with the Python reference.
7. Done for control: the bridge, distance-tagged route conversion and ESP
   closed-loop line following complete routes A-D. Remaining: anomaly scores
   over the link, return legs, and fault scenarios under ESP control.
