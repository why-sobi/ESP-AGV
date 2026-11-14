# ESP32-S3 Controller

This ESP-IDF project targets the ESP32-S3 and uses the board's CH343 USB-to-UART
connection (currently COM3) on UART0 GPIO43 (TX) and GPIO44 (RX) at 115200 baud.
The firmware runs the PD line follower and the distance-gated junction-action
state machine. Webots sends it simulated ground-sensor and wheel-encoder values
and applies the returned wheel targets; the firmware does not directly drive a
physical motor or sensor.

## Build and flash

Use the VS Code ESP-IDF PowerShell terminal configured for ESP-IDF 6.0.1, or
activate it from a plain PowerShell:

```powershell
$env:IDF_PYTHON_CHECK_CONSTRAINTS = "0"
. C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1
cd firmware
idf.py set-target esp32s3   # only for a fresh build directory
idf.py build
idf.py -p COM3 flash
```

`IDF_PYTHON_CHECK_CONSTRAINTS=0` is needed on this machine because the
installer put `espidf.constraints.v6.0.txt` in `C:\Espressif\tools\` instead of
`C:\Espressif\`, where the dependency check looks for it.

Do not flash while another program (including the IDF monitor or a running
Webots ESP mission) has COM3 open. Flashing is not required to build.

## Check the ESP over COM3

Close the ESP-IDF monitor, then run this from the repository root:

```powershell
.\.venv\Scripts\python.exe -m scripts.esp.esp_serial_smoke_test --port COM3
```

The test configures short routes and sends simulated sensor and encoder frames.
It checks centered-line speed, PD steering direction, reported odometry, that a
junction pattern is ignored outside its distance window and taken inside it,
junction clearing, the destination stop, the missed-junction stop, the U-turn,
and malformed-route rejection. It does not involve Webots.

## Let the ESP control the Webots e-puck

The `epuck_route` controller still uses the Supervisor for map planning,
initial placement, route progress, safety checks, and logging. Set
`control_mode` to `esp32` in `maps/routes/mission.json` (or launch with
`AGV_CONTROL_MODE=esp32`) to send the planned route and stream ground-sensor
and encoder readings to the ESP; Webots applies the wheel targets returned by
the ESP. The default `supervisor` mode preserves the existing controller.
Close the IDF monitor before starting Webots so the controller can own COM3.

## Serial protocol

Lines are ASCII, newline-terminated, 8-N-1, 115200 baud. Only one program may
own COM3 at a time.

- Configure one route: `R,<step>,<step>,...\n`, where each step is
  `<action>@<metres>`. Actions are `L`, `S`, or `R` for each junction, and the
  last step must be the destination `D`. Distances are expected wheel-odometry
  distances from the start of the route and must not decrease, e.g.
  `R,R@0.265,D@0.779\n`. An optional first step `U@0.000` makes the robot
  U-turn in place before following, e.g. `R,U@0.000,L@0.514,D@0.779\n` for a
  return leg. The controller replies `A,ROUTE_OK\n`, or `E,BAD_ROUTE\n`. At
  most 32 steps.
- Supply sensor frames:
  `S,<sequence>,<dt_ms>,<left>,<center>,<right>,<enc_left_rad>,<enc_right_rad>\n`.
  Lower ground-sensor values mean darker track, matching the Webots sensors.
  Encoder values are the wheel position sensors in radians.
- Read motor frames:
  `M,<sequence>,<left_rad_s>,<right_rad_s>,<state>,<route_index>,<distance_m>\n`.
  `distance_m` is the ESP's odometry distance since the route started.
- Errors are returned as `E,<reason>\n`.

States: `FOLLOWING`, `TURN_LEFT`, `STRAIGHT`, `TURN_RIGHT`, `U_TURN`,
`AT_DEST`, `LINE_LOST`, `LINK_LOST`, `JUNCTION_MISSED`, `NO_ROUTE`.

Wheel targets are within +/-5 rad/s. They are only negative during a U-turn.

## Behavior

- **Line following:** PD on the ground-sensor line offset, with base speed
  between 2.5 and 3.5 rad/s. If the line is lost, the controller turns slowly
  towards where it was last seen for up to 750 ms, then stops with
  `LINE_LOST`.
- **Odometry:** distance = 0.02 m x the mean wheel rotation since the first
  sensor frame of the route.
- **Junctions:** a junction is both side sensors on dark track for two frames.
  It only counts within +/-0.10 m of the distance given for the next junction.
  Elsewhere the pattern is ignored and line following continues. The action is
  then applied until the side sensors clear for three frames.
- **Missed junction:** if odometry passes a junction's distance plus 0.10 m
  without seeing it, the controller stops with `JUNCTION_MISSED`.
- **Destination:** `D` stops the wheels when odometry reaches its distance.
- **U-turn:** a leading `U` spins the robot left in place at 2 rad/s per
  wheel. The turn angle comes from the encoders (wheel base 0.052 m). Once
  it has turned at least 160 degrees and the center sensor sees the line, line
  following takes over; lines crossed earlier in the spin are ignored. If it
  turns 270 degrees without finding the line, it stops with `LINE_LOST`. A
  spin leaves the odometry distance unchanged, so the following junction
  distances still count from the turn point.
- **Link loss:** the 250 ms sensor timeout starts at the first sensor frame
  after a route is configured. On timeout the controller sends one unsolicited
  `LINK_LOST` frame (repeating the last sequence number) and outputs zero
  wheel targets; when frames resume it continues the route where it stopped.
  The simulation pauses with the host, so this is not a hazard in Webots.
- After unloading, send a new route command for the return leg (planned from
  destination back to dock, starting with `U`); it resets the route state and
  odometry. `epuck_route` does this when the mission sets `return_to_start`.

Why distance gating: on the base track, all three ground sensors see dark track
on the wide horizontal lines as well as at junctions, so the sensor pattern
alone cannot identify a junction or the destination. See
`docs/DECISIONS.md`.

## Important limitations

- ESP-side anomaly inference is not implemented yet. The sensor frame now
  carries the encoder readings that the motor autoencoder needs; it still
  lacks the inference itself and a way to report scores.
- Junction distances come from the map scaled by the mission's
  `esp_odometry_scale` (0.92, measured on the base track). A track with
  different curvature may need a re-measured scale, and junctions closer than
  about 0.2 m along a route would make the +/-0.10 m windows overlap.
- Wheel slip makes odometry read short. Slip is small in the current
  simulation; a motor fault that changes the driven path may push a junction
  out of its window and stop the run with `JUNCTION_MISSED`.
- This firmware emits wheel speeds in Webots-compatible rad/s and ignores the
  mission's `wheel_speed_rad_s`.
