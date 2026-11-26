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
the route speed, and malformed-route rejection. It does not involve Webots.

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
  return leg. An optional leading `V@<rad/s>` sets the straight-line wheel
  speed for the route (1.0-4.5, default 3.5), e.g. `R,V@2.750,R@0.265,D@0.779\n`;
  it must come before every other step. The controller replies `A,ROUTE_OK\n`,
  or `E,BAD_ROUTE\n`. At most 32 steps.
- Supply sensor frames:
  `S,<sequence>,<dt_ms>,<left>,<center>,<right>,<enc_left_rad>,<enc_right_rad>\n`.
  Lower ground-sensor values mean darker track, matching the Webots sensors.
  Encoder values are the wheel position sensors in radians.
- Read motor frames:
  `M,<sequence>,<left_rad_s>,<right_rad_s>,<state>,<route_index>,<distance_m>,<motor_score>,<ir_score>,<motor_alarm>,<ir_alarm>\n`.
  `distance_m` is the ESP's odometry distance since the route started. The
  scores are the motor and IR anomaly scores of the latest window (`-1` until
  a full window exists), and the alarms are the persistent 3-of-5 decisions
  (`0`/`1`). `epuck_route` logs them as `esp_motor_score`, `esp_ir_score`,
  `esp_motor_alarm` and `esp_ir_alarm`.
- Inference timing: `Q\n` replies
  `I,<motor_last_us>,<motor_max_us>,<ir_last_us>,<ir_max_us>\n`.
- Errors are returned as `E,<reason>\n`.

States: `FOLLOWING`, `TURN_LEFT`, `STRAIGHT`, `TURN_RIGHT`, `U_TURN`,
`AT_DEST`, `LINE_LOST`, `LINK_LOST`, `JUNCTION_MISSED`, `NO_ROUTE`.

Wheel targets are within +/-5 rad/s. They are only negative during a U-turn.

## Behavior

- **Line following:** PD on the ground-sensor line offset. The base speed is
  the route speed `V` (default 3.5 rad/s) on a centered line, falling to 5/7 of
  it (2.5 rad/s by default) for sharp corrections. If the line is lost, the controller turns slowly
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

## Anomaly detection

The firmware runs the motor and IR autoencoders of `models/esp-conv-v1` on
every sensor frame, as `scripts/models/train_autoencoder.py` defines them.
`main/anomaly_model.h` is generated; regenerate it after retraining:

```powershell
.\.venv\Scripts\python.exe -m scripts.models.export_c --model models\esp-conv-v1 --out firmware\main\anomaly_model.h
```

- **Samples:** one per frame, from the second frame of a route. The motor
  sample is the frame's wheel targets (rounded to the 3 decimals sent) and
  the wheel speeds from the encoder change. The IR sample is the three
  ground sensors. Each is scaled with the training scaler.
- **Score:** reconstruction MSE of the last 20 samples (0.64 s), computed in
  `main/anomaly.cpp`. The model is a float32 forward pass with no
  libraries: 5208 motor and 4500 IR parameters, held in flash.
- **Monitoring:** a window is only monitored when it starts at least 0.5 s
  after the route started and none of its samples is `AT_DEST`, `U_TURN` or
  within 0.5 s after a U-turn. The alarm is set when 3 of the last 5
  windows exceed the model's threshold.
- **Reset:** a new route command clears the windows and alarm history.
- **Cost:** about 2.3 ms (motor) + 2.0 ms (IR) per frame with `-O2`
  (`CONFIG_COMPILER_OPTIMIZATION_PERF`). Image size is 0x336b0 bytes, with 80%
  of the app partition free.
- **Agreement:** `scripts/esp/replay_scores.py` replays recorded runs and
  compares the ESP's scores and alarms with Python (see `docs/USAGE.md`).

Why distance gating: on the base track, all three ground sensors see dark track
on the wide horizontal lines as well as at junctions, so the sensor pattern
alone cannot identify a junction or the destination. See
`docs/DECISIONS.md`.

## Important limitations

- The anomaly alarms are reported, not acted on: the firmware keeps driving
  and the host logs them. What the robot should do on an alarm (slow down,
  stop, request service) is not decided yet.
- Junction distances come from the map scaled by the mission's
  `esp_odometry_scale` (0.92, measured on the base track). A track with
  different curvature may need a re-measured scale, and junctions closer than
  about 0.2 m along a route would make the +/-0.10 m windows overlap.
- Wheel slip makes odometry read short. Slip is small in the current
  simulation; a motor fault that changes the driven path may push a junction
  out of its window and stop the run with `JUNCTION_MISSED`.
- This firmware emits wheel speeds in Webots-compatible rad/s. `epuck_route`
  sends the mission's `esp_speed_rad_s` as the route speed when it is set; the
  Supervisor controller's `wheel_speed_rad_s` does not apply to the ESP.
