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
  It also runs the motor and IR anomaly models and returns their scores and
  alarms. In `esp32` mode, `epuck_route` drives Webots with those targets;
  routes A-D complete, one way or as round trips.
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
9. **Retrain the anomaly models on ESP-controlled runs (2026-11-26).** The
   motor features include the controller's wheel targets, and the ESP's PD
   loop commands different targets from the Supervisor controller. The round
   trip also adds the stop, unload and U-turn.
   - The Supervisor-trained `conv-v1`/`dense-v1` alarm on every healthy ESP
     run.
   - The new healthy data covers the ESP speed grid (route speed `V@`) and
     three seeded start poses per route and speed, because Webots repeats an
     identical run byte for byte.
   - INT8 quantisation alone would not have helped: the data changed, not
     the arithmetic.
10. **Do not monitor commanded manoeuvres.** Windows overlapping `AT_DEST`,
    `UNLOADING` or `U_TURN`, and the first 0.5 s after a U-turn, are trained
    on but not scored. These transients happen once or twice per run and
    caused most healthy false alarms. The ESP knows its own state, so the
    same gating runs on the device. A fault that starts during a manoeuvre
    is detected once normal driving resumes.
11. **Set thresholds so that no healthy validation run alarms.** The 99.5th
    percentile of validation windows means that about 0.5% of healthy windows
    exceed it by definition. Those cluster enough to trip the 3-of-5 rule:
    7/18 motor and 5/18 IR healthy test runs alarmed, and the IR model fired
    before onset in 15 of 120 fault runs. The run-level rule cut this to
    2/18, 0/18 and 0/120, at the cost of mild faults (20% power loss 3/8).
12. **Run the models on the ESP in plain float32 C++.** The two conv
    autoencoders (about 9.7k parameters) need about 4.5 ms per 32 ms frame at
    `-O2`, and the image leaves 80% of the app partition free. A runtime
    library or INT8 was not needed. The weights header is generated from the
    trained model (`scripts/models/export_c.py`), and a replay test checks
    the ESP against Python window by window.
    (Superseded by decisions 15-16: the detectors now run on esp-dsp.)
13. **On a persistent alarm, pull over, stop and notify (2026-11-26).** An
    alarm that has lasted the route's `F@` hold (2 s) makes the ESP turn ~90
    degrees, drive 8 cm off the line and stop, then post a notification to
    ntfy.sh. Stopping on the line would block the track; driving on with a
    fault risks the robot. Sending runs on the other core so control never
    waits for WiFi. Motor faults often lose the line first; `LINE_LOST`
    then also notifies, naming any alarm from the last 10 s.
14. **Pair the conv autoencoders with a motor GRU and a stuck-sensor rule,
    not replace them (2026-11-26).** The conv models are good at sudden
    faults; the GRU (a plant model predicting wheel speeds from the targets
    only, so it cannot hide a weak wheel by copying the last speed) catches
    20-30% power loss; the rule catches a frozen sensor. Each keeps its own
    threshold and the motor/IR alarm is either detector's. Separate models
    also keep fault attribution ("motor" vs "line sensor") for the
    notification. The GRU threshold is 2x the no-validation-alarm value: its
    weakest fault peaks at >=25x, its worst healthy test run at 1.01x.
    An IR GRU, worst-feature scoring and a sample-difference score were
    tried and did not help (`reports/esp-gru-eval-v1`).
15. **Use Espressif's esp-dsp for inference, not ESP-DL (2026-11-26).**
    esp-dsp's float32 dot products and matrix multiplies use the S3's SIMD
    instructions and keep the float match with Python (1e-5). ESP-DL runs
    Conv/Gemm only quantized; INT8 breaks the motor conv model and, at these
    model sizes, ESP-DL is ~2.5x slower than esp-dsp (decision 19).
16. **Keep the weights in internal RAM.** Read from flash through the cache
    shared with WiFi, the detectors were bound by flash bandwidth (~4.8 ms
    per frame); copied to RAM once (~63 KB) they take ~1.05 ms.
17. **Stuck sensor = flat while the robot drives, not while another sensor
    moves (2026-11-27).** A centre sensor stuck at its on-the-line value
    makes the ESP drive off the track, where all three sensors read flat
    floor, so the "another sensor moving" condition failed live. Healthy
    sensors are never flat for 1 s even when the robot is still.
18. **Choose the pull-over side from the map on the Supervisor
    (2026-11-27).** The ESP has no map; the Supervisor checks each step's
    swept pull-over path against other lines and walls and sends one side
    per step (`P@`). A blind right turn landed on the other line of the
    small loop on half the round-trip legs. Sensing the side on the ESP was
    not used: the fault being reacted to may be an IR fault.
19. **Keep float32; INT8 only as a benchmark (2026-11-27).** With ESP-DL on
    the ESP, INT8 and INT16 take ~0.75-0.8 ms per model vs ~0.3 ms for
    float32 on esp-dsp (fixed per-layer runtime cost dominates models of
    3-17k multiply-adds). INT8 makes every healthy run alarm on the motor
    conv model (its healthy error is below INT8's rounding noise). INT8's
    only gain, ~4x smaller weights, is not needed (165 KB heap free).
20. **Back architectures with measured latency and a held-out search
    (2026-11-27).** Every architecture's ESP time comes from a benchmark on
    the chip (967 measured), and Optuna tunes on half the fault runs while
    the other half and the healthy test runs judge the final candidates,
    retrained with the full budget. The search found smaller motor conv and
    GRU models that are 2-3x faster and detect as well or better; the
    smaller IR model trades 2/18 healthy test alarms for 5 more detections.
    (`reports/arch-search-v1`).
21. **Deploy the motor conv and GRU winners, keep the IR model
    (2026-11-27).** Both are better or equal on every measure; the IR
    candidate adds healthy alarms, so it is left for the user. The motor and
    IR models now use different windows (10 and 20), so each conv model
    carries its own sizes (`models.<name>` in `esp-conv-v2`, fields of
    `ConvAutoencoder` in the header) and the firmware sizes buffers for the
    largest. Result: 1.18 → 0.81 ms per frame, same detections offline, live
    behaviour unchanged.
22. **Reconnect COM3 in the host, not the ESP (2026-11-27).** The ESP already
    stops on `LINK_LOST` and resumes when frames return; the controller
    reopens the port (up to 30 s, Webots paused) and resends the frame.
23. **Per-edge odometry scales (2026-11-27).** Driving edge-002 showed the
    single 0.92 scale was an average of ~0.973 (long edges) and ~0.836 (the
    loop): routes with two long edges stopped 8-10 cm short. One scale per
    edge type fits all 237 measured legs within 0.7 cm and also brings the
    standard routes' stops closer. `avoid_edges` drives the parallel edges
    without changing the planner.
24. **Corners are marked by the map, not guessed by the ESP (2026-11-27).**
    The ESP cannot tell a 90° corner onto a straight line from the base
    track's 90° junctions onto its circle from the sensors; the host measures
    the turn 4 cm along each edge and sends `l`/`r` at ≥85° (base track
    61-76°, track2 90°). Guessing from an already-dark pattern broke the base
    track's circle junctions. Corners spin in place like the U-turn and, like
    it, are not monitored.
25. **Y branches are bear turns, also marked by the map (2026-11-27).** A
    45° branch peels off beside the old line, so the junction pattern clears
    before the robot has left it. Holding every turn for a minimum angle would
    change the base track's turns (some release after 5°), so the host sends
    `<`/`>` for 35-58° and only those wait for the junction distance and
    hold until 25° have turned. Each track gets per-edge odometry scales from
    its own healthy runs (mid-edge ratios).

## Failure points observed

- **Supervisor-trained models on ESP data:** 13-25% of healthy windows over
  threshold, and alarms on every run (see decision 9).
- **Healthy-run check too strict for the ESP:** the ESP's sensors briefly
  leave the line on corners and in the last few centimetres into a terminal
  node (node-001, node-003), where the line ends. An ESP run now fails the
  check only if the line is lost for longer than the firmware's own 750 ms
  search limit.
- **Stuck sensor on the line value (fault-stop-v3):** the ESP drove off the
  track with the first stuck rule; see decision 17.
- **Blind right pull-over:** would land on the small loop's other line; see
  decision 18.
- **Weights in flash:** esp-dsp alone barely helped until the weights moved
  to RAM; see decision 16.
- **USB serial drops:** COM3 dropped mid-run a few times (`SerialException`,
  "Access is denied"), once when the laptop charger was moved. The collectors
  mark such runs `serial_error` and repeat them.

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

1. Validate the reviewed graph semantics and branch choices at all junctions
   (the parallel edges edge-002 and edge-004 are now driven with
   `avoid_edges`; edge-003 has never been driven).
2. Refine the generated graph and names against the visualized path; pixel
   skeleton extraction can merge nearby segments or produce parallel edges.
3. Keep route tracking/localization behind an interface that can be replaced
   for hardware; the current Supervisor controller is simulation-only.
4. Detect IR drift and light IR noise (a level check or a cross-sensor
   model; a one-step GRU does not help).
5. Done: architecture-search winners (motor conv, GRU) deployed.
6. Done: reconnect when COM3 drops mid-run (real unplug test to do).
7. Done: healthy and fault runs under ESP control, models on the ESP that
   agree with Python, the response to an alarm (pull over, stop, notify),
   weak-motor and stuck-sensor detection, and evidence for the architectures
   (latency table, INT8 benchmark, architecture search). See `REPORT.md`.
