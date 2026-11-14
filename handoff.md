# Handoff: ESP PD Control in Webots

## Goal

The Supervisor plans the route and tells the ESP32-S3 which way to go at each
junction. Webots streams simulated telemetry to the ESP, which runs the PD
line-following loop (and, later, anomaly detection); Webots applies the
returned wheel commands.

## Current status (2026-11-14)

- **Done: ESP closed-loop control.** All four two-edge routes (A-D) complete
  headlessly with `control_mode: esp32`:
  - The ESP takes each turn 2.5-3.6 cm before the junction node.
  - It stops 1.0-1.4 cm from the destination.
  - Maximum path error is 2.4-2.5 cm, the same as the Supervisor controller.
- **Protocol:**
  - Route: `R,<action>@<odometry m>,...,D@<m>`, e.g. `R,R@0.265,D@0.779`.
  - Sensor frame: `S,seq,dt,gsL,gsC,gsR,encL,encR`.
  - Reply: `M,seq,left,right,state,route_index,distance_m`.
  - Details in `firmware/README.md`.
- **Why the distances:** on the base track, wide horizontal lines put all
  three ground sensors on dark track, the same pattern as a junction. The ESP
  therefore only accepts a junction within +/-0.10 m of its expected odometry
  distance, and stops at the destination distance. Map distances are scaled
  by `esp_odometry_scale` (0.92, measured from the healthy runs). See
  `docs/DECISIONS.md` decision 6.
- **Firmware fixes:**
  - Bounded line search instead of an immediate `LINE_LOST`.
  - The UART read no longer adds 50 ms per frame (round trip is now about
    8 ms).
  - The link timeout starts at the first sensor frame and recovers when frames
    return.
- **Controller changes (`epuck_route.py`):**
  - Sends encoders and the distance-tagged route.
  - Skips the ESP's stale `LINK_LOST` reply.
  - In ESP mode, waits for the ESP's `AT_DEST` and checks it within 0.05 m.
  - Logs `esp_distance_m`.
  - Appends crash tracebacks to `<log dir>/controller-errors.log`.
- **Round trips:** `return_to_start: true` (ESP mode) unloads for 1 s, then
  sends the return route with a leading `U@0.000`. The ESP spins left in
  place by encoder heading (at least 160 degrees, then back on the line) and
  follows it home. All four round trips complete, stopping within 1.3 cm of
  the start node. See `docs/DECISIONS.md` decision 8.
- **Verified:**
  - 36 unit tests pass.
  - The smoke test passes on the flashed firmware.
  - Supervisor mode is unchanged: it reproduces the committed healthy route
    A 2.5 run byte-for-byte in the original columns.

## Build/flash

```powershell
$env:IDF_PYTHON_CHECK_CONSTRAINTS = "0"
. C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1
cd firmware; idf.py build; idf.py -p COM3 flash
```

The constraints file is in `C:\Espressif\tools\`, not where the check expects
it; hence the override. The build directory is clean (no stale `esp32-s3`
paths). The flashed firmware matches `firmware/main/main.cpp`.

## Next steps

1. Score ESP-controlled healthy runs with `models/conv-v1`. The models were
   trained on Supervisor-controlled runs, whose wheel commands differ from the
   ESP's PD loop. If false alarms appear, record a healthy dataset under ESP
   control and retrain.
2. Export the chosen model to C, run inference on the ESP, and add scores to
   the motor frame. The sensor frame already carries the encoder data.
3. Test ESP-mode fault runs. Decide whether `JUNCTION_MISSED` after a
   path-changing fault is acceptable.

## Preserve

The worktree has substantial uncommitted changes, including the intentional
`esp32-s3/` to `firmware/` rename and script reorganization. Do not discard or
stage unrelated changes.
