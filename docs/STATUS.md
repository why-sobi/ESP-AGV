# Project Status

## Agreed project goal

Keep the AGV in the Webots simulation environment. Webots provides simulated
sensors and odometry; the ESP32-S3 will run the control and anomaly-detection
computation; Webots applies the returned wheel commands to its simulated robot.
The eventual loop is:

```text
Webots sensors/encoders -> Python bridge over USB serial (COM3) -> ESP32-S3
ESP32-S3 wheel commands/anomaly scores -> Python bridge -> Webots motors/logging
```

The Webots simulation remains the environment throughout this plan.

## Verified

- The graph planner, path-image converter, map visualizer, and Webots route
  controller are implemented. The user visually checked the configured
  simulation route.
- The configured two-edge route has completed headless Webots runs at 2.5 and
  3.0 rad/s. These runs are saved in `data/route_runs`.
- The ESP32-S3 N16R8 project builds and has been flashed over COM3 with an
  ESP32-S3/16 MB-flash configuration.
- `scripts/esp_serial_smoke_test.py` successfully sent route and simulated
  sensor messages over COM3 and verified PD wheel outputs, junction-action
  handling, destination stop, and invalid-route rejection.
- `epuck_data` now writes a separate timestamped CSV per session under
  `data/sensor_runs` instead of appending to a shared filename.
- `scripts/prepare_telemetry.py` derives wheel velocities, creates 20-step
  motor/IR windows, splits by run before windowing, fits scalers on training
  data only, and rejects exact duplicate runs.
- The pilot dataset under `data/datasets/sim-baseline-v1` contains 454 training
  windows from a 2.5 rad/s run and 385 validation windows from a 3.0 rad/s run.
  Exact-duplicate detection is implemented and unit-tested; duplicate recordings
  are not treated as independent runs.
- The Python script test suite passes (18 tests at last verification).

## Not yet implemented

- The Python Webots-to-ESP bridge. Webots still runs its own controller; the
  smoke test is a separate host script and does not drive Webots.
- A graph-route-to-relative-junction-action converter for the ESP.
- Anomaly-detection model definitions, training/evaluation scripts, fault
  injection harness, threshold selection, and ESP model inference.
- End-to-end testing where the ESP computes the commands that Webots applies.

## Current data/model status

The current 2-run dataset proves the telemetry-preparation path, not model
quality. It covers one route at two speeds, has no independent final test split,
and does not cover controlled faults. The next data milestone is multiple
distinct healthy runs across the routes/speeds we want to support, followed by
separately labeled fault-injection runs. Train only on healthy data; use
different healthy runs for validation and final testing.

The anomaly-detection target remains simulated telemetry. ESP deployment means
running the trained inference computation on the ESP32-S3 while Webots remains
the source of sensor data and the destination for wheel commands.

## References

- `docs/USAGE.md`: commands, project files, data collection, preparation, ESP
  checks, and what is or is not wired together.
- `docs/TODO.md`: ordered next steps and completion criteria.
- `docs/DECISIONS.md`: navigation, simulation-first architecture, and lessons.
