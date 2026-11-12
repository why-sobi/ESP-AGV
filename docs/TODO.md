# Project TODO

## Currently Doing

- **Review and name the graph** in `maps/paths/base track.json`
  using `maps/visualizations/base track.html`. Confirm nodes correspond to real
  route decision points and that the circular section's connections are right.
  Rename generated node/edge IDs to stable meaningful names.
- Expand Webots route tests beyond the first verified two-edge mission; check
  route selection and stopping behavior around the loop and all junctions.

## Next: Make a testable navigation loop

1. **Validate the graph and route output.**
   Run the planner for selected start/destination pairs, including both
   directions through the circular feature and each multi-branch junction.
   Confirm every planned edge exists and follows the intended path.
3. **Validate and refine route execution.**
   The initial Supervisor-based route controller is now connected. Confirm it
   follows one edge, reaches a junction, selects the planned branch, and exits
   the circular feature correctly.
4. **Test in Webots incrementally.**
   Start with one edge, then one junction, then the circular feature, then
   multi-edge missions. Log the route ID, current edge/node, pose, sensor data,
   wheel targets, and odometry for each run.

## Preserve the original anomaly-detection objective

6. **Stabilize healthy line following and data collection.**
   Tune the PD controller on known paths, check wheel commands and sensor
   behavior, and write each simulation run to a clearly identified output file.
   Confirm logging has uniform timestep intervals and flushes data on shutdown.
7. **Build the feature pipeline.**
   Derive wheel velocities from odometry, create temporal windows, split by
   separate simulation runs, and fit scaling only on healthy training data.
8. **Train and validate the dual autoencoders.**
   Train the motor and ground-IR models on healthy data, set thresholds from
   held-out healthy validation runs, and test false-positive rates.
9. **Evaluate controlled faults.**
   Inject motor and sensor faults in Webots, check which model responds first,
   and document cross-coupling and fault-isolation limits.
10. **Carry route context into anomaly analysis.**
    Record map, route, edge, and speed context. Ensure healthy training and
    evaluation data cover expected routes and operating modes so normal route
    changes are not mistaken for faults.

## Later / Hardware Readiness

- Select and validate a physical localization method; encoder dead reckoning
  alone is not reliable node identification.
- Define the route-message interface and transport (local file for simulation;
  HTTPS/MQTT may be used for a hardware planner).
- Verify route/map versioning, stale-route handling, and safe stop behavior if
  localization or route progress is lost.
- Revisit route cost once requirements are known. The current Dijkstra planner
  minimizes distance; edge speed limits are carried as metadata but do not yet
  affect route selection.
