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
  `<action>@<metres>`. Actions are `L`, `S`, or `R` for each junction, or `l`
  / `r` for a corner (a turn the map measures at >= 85 degrees 4 cm out: a
  straight crossing line), or `<` / `>` to bear onto a Y branch (a turn of
  35-58 degrees), and the last step must be the destination `D`. Distances are expected wheel-odometry
  distances from the start of the route and must not decrease, e.g.
  `R,R@0.265,D@0.779\n`. An optional first step `U@0.000` makes the robot
  U-turn in place before following, e.g. `R,U@0.000,L@0.514,D@0.779\n` for a
  return leg. An optional leading `V@<rad/s>` sets the straight-line wheel
  speed for the route (1.0-4.5, default 3.5), e.g. `R,V@2.750,R@0.265,D@0.779\n`.
  An optional leading `F@<seconds>` (0.1-10) enables the fault stop with that
  alarm hold, e.g. `R,F@2.000,V@2.750,R@0.265,D@0.779\n`; without it alarms
  are only reported. An optional leading `E@<place>|<place>|...` gives one
  label per step: where the robot is while heading for that step, e.g.
  `E@between Junction 3 and Junction 2|between Junction 2 and Junction 1`.
  Notifications name it; labels are at most 63 characters, without `,` or
  `|`, and their count must match the steps. `epuck_route` always sends
  them. An optional leading `P@<R|L>...` gives the pull-over side per step
  (one letter per step, like `E@`; right without it), e.g. `P@RL`;
  `epuck_route` picks each from the map (see "Fault stop" below). `E@`,
  `P@`, `V@` and `F@` must come before every other step, in any order. The controller replies `A,ROUTE_OK\n`, or `E,BAD_ROUTE\n`. At
  most 32 steps.
- Supply sensor frames:
  `S,<sequence>,<dt_ms>,<left>,<center>,<right>,<enc_left_rad>,<enc_right_rad>\n`.
  Lower ground-sensor values mean darker track, matching the Webots sensors.
  Encoder values are the wheel position sensors in radians.
- Read motor frames:
  `M,<sequence>,<left_rad_s>,<right_rad_s>,<state>,<route_index>,<distance_m>,<motor_score>,<ir_score>,<motor_alarm>,<ir_alarm>,<gru_score>,<gru_alarm>,<stuck_alarm>\n`.
  `distance_m` is the ESP's odometry distance since the route started.
  `motor_score`/`ir_score` are the conv autoencoder scores of the latest
  window (`-1` until a full window exists). `motor_alarm` is the motor conv
  or motor GRU alarm, `ir_alarm` the IR conv or stuck-sensor alarm (`0`/`1`);
  these drive the fault stop. `gru_score` is the motor GRU score (`-1` for
  the first 62 samples), `gru_alarm` and `stuck_alarm` the two new detectors'
  own alarms. `epuck_route` logs them as `esp_motor_score`, `esp_ir_score`,
  `esp_motor_alarm`, `esp_ir_alarm`, `esp_gru_score`, `esp_gru_alarm` and
  `esp_stuck_alarm`.
- Inference timing: `Q\n` replies
  `I,<motor_last_us>,<motor_max_us>,<ir_last_us>,<ir_max_us>,<gru_last_us>,<gru_max_us>\n`.
- Resource usage: `U\n` replies
  `U,<heap_free>,<heap_min_free>,<largest_block>,<internal_free>,<internal_min_free>,<psram_total>,<psram_free>,<heap_total>,<tasks>,<window_us>\n`
  (bytes; `heap_min_free` is the lowest since boot), then one
  `T,<name>,<core>,<cpu_permille>,<stack_free_min>\n` per FreeRTOS task:
  its share of one core since the previous `U` (`window_us`; core `-1`: not
  pinned) and its lowest free stack in bytes. Uses FreeRTOS run-time stats
  (`sdkconfig.defaults`). `epuck_route` polls it between frames when
  `AGV_ESP_RESOURCE_LOG_S` is set; `scripts/esp/resource_usage.py`
  summarizes the logs.
- Latency benchmark (`main/bench.cpp`, not during a route):
  `B,C,<features>,<window>,<channels>,<kernel>,<bottleneck>,<hidden>,<repeats>\n`
  (conv autoencoder) or `B,G,<inputs>,<hidden>,<outputs>,<repeats>\n` (GRU
  step) runs the same esp-dsp forward pass as the detectors with those sizes
  and random weights in internal RAM, and replies
  `b,<ok>,<median_us>,<max_us>,<weight_bytes>\n`.
  `scripts/models/esp_latency.py` sweeps the architecture search space with
  it.
- WiFi status: `W\n` replies
  `W,<OFF|CONNECTING|CONNECTED>,<ip or ->,<last send result>,heap=<bytes>\n`.
  The send result is `NONE`, `SENDING_<attempt>`, `HTTP_<status>`, `NO_WIFI`
  or `<http error>/<esp-tls error>/<mbedTLS error>`.
- Test notification: `N\n` queues one and replies `A,NOTIFY_QUEUED\n`, or
  `E,NOTIFY_OFF\n` when WiFi is not configured.
- Errors are returned as `E,<reason>\n`.

States: `FOLLOWING`, `TURN_LEFT`, `STRAIGHT`, `TURN_RIGHT`, `U_TURN`,
`AT_DEST`, `LINE_LOST`, `LINK_LOST`, `JUNCTION_MISSED`, `FAULT_PULLOVER`,
`FAULT_STOP`, `NO_ROUTE`.

Wheel targets are within +/-5 rad/s. They are only negative during a spin in
place: a U-turn, a corner turn, or a fault pull-over.

## Behavior

- **Line following:** PD on the ground-sensor line offset. The base speed is
  the route speed `V` (default 3.5 rad/s) on a centered line, falling to 5/7 of
  it (2.5 rad/s by default) for sharp corrections. If the line is lost, the controller turns slowly
  towards where it was last seen for up to 750 ms, then stops with
  `LINE_LOST`. Within 5 cm of the destination it instead drives straight on at
  1 rad/s (up to 3 s): the sensors run past the end of a line or across a T's
  crossbar before the axle arrives.
- **Odometry:** distance = 0.02 m x the mean wheel rotation since the first
  sensor frame of the route.
- **Junctions:** a junction is both side sensors on dark track for two frames.
  It only counts within +/-0.10 m of the distance given for the next junction.
  Elsewhere the pattern is ignored and line following continues. The action is
  then applied until the side sensors clear for three frames. If the pattern
  was already present when the window opened (all three sensors dark along a
  line as wide as their spread, e.g. horizontal lines on the stretched track
  image), it says nothing about where the junction is: the action waits until
  3 cm before the junction distance.
- **Corners (`l`/`r`):** in the window the speed drops to 1.5 rad/s. The robot
  spins in place (2 rad/s per wheel) once its axle is on the crossing line:
  at the junction distance, 4 cm after the junction pattern appears, or 2 cm
  after all three sensors run off the line's far side, whichever is first.
  Line following takes over once it has turned at least 60 degrees and the
  center sensor sees a line, at 1.5 rad/s for the next 4 cm; past 135
  degrees it stops with `LINE_LOST`.
- **Bear (`<`/`>`):** a Y branch appears gradually beside the old line, so the
  turn waits until 3 cm before the junction distance and is held until the
  robot has turned 25 degrees (encoder heading; released at 80 at most), not
  just until the junction pattern clears.
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
- **Fault stop** (routes with `F@<s>` only): when either anomaly alarm has
  stayed on for `<s>` seconds, the controller spins right in place at 2 rad/s
  until the encoders show 90 degrees, drives 0.08 m forward off the line
  (`FAULT_PULLOVER`), then stops with `FAULT_STOP` and queues a notification.
  If that takes longer than 6 s (e.g. a stalled motor or a stuck encoder), it
  stops where it is and the notification says the pull-over was not finished.
  `FAULT_STOP` ends the route like `LINE_LOST`. On these routes a
  `LINE_LOST` or `JUNCTION_MISSED` stop also sends a notification, naming any
  anomaly alarm in the last 10 s, including one before the return leg reset
  the detectors.
- After unloading, send a new route command for the return leg (planned from
  destination back to dock, starting with `U`); it resets the route state and
  odometry. `epuck_route` does this when the mission sets `return_to_start`.

## Anomaly detection

The firmware runs four detectors on every sensor frame (`main/anomaly.cpp`):

| Detector | Catches | Defined in |
|---|---|---|
| Motor conv autoencoder (`models/esp-conv-v2`: window 10, 8 channels, bottleneck 12, hidden 16) | sudden motor and encoder faults | `scripts/models/search_architectures.py` |
| Motor GRU plant model (`models/esp-gru-v2`: 16 hidden, 31-sample score window) | weak motors (20-30% power loss), faster on all motor faults | `scripts/models/search_architectures.py` (`gru_experiment.py` model) |
| IR conv autoencoder (`models/esp-conv-v2`, unchanged from `esp-conv-v1`: window 20, 16 channels, bottleneck 8, hidden 32) | dead, noisy and drifting sensors | `scripts/models/train_autoencoder.py` |
| Stuck-sensor rule | a ground sensor stuck at one value | `scripts/models/gru_experiment.py` |

The motor alarm is either motor detector's, the IR alarm either IR
detector's; why this pairing is in `reports/esp-gru-eval-v1/FINDINGS.md`.
`main/anomaly_model.h` is generated; regenerate it after retraining. The
motor and IR conv models may use different windows and sizes: each model's
sizes are in its `ConvAutoencoder` struct and the buffers are sized for the
largest (kernel 3; channels, bottleneck and hidden multiples of 4).

```powershell
.\.venv\Scripts\python.exe -m scripts.models.export_c --model models\esp-conv-v2 --gru models\esp-gru-v2 --out firmware\main\anomaly_model.h
```

- **Samples:** one per frame, from the second frame of a route. The motor
  sample is the frame's wheel targets (rounded to the 3 decimals sent) and
  the wheel speeds from the encoder change. The IR sample is the three
  ground sensors. Each is scaled with the training scaler (the stuck rule
  uses raw readings).
- **Conv score:** reconstruction MSE of the last 20 samples (0.64 s).
- **GRU score:** the GRU (32 hidden) predicts both wheel speeds from the
  wheel targets, one step per frame from the start of the route; the score
  is the larger wheel's mean squared prediction error over the last 62
  samples (2 s). Threshold: 2x the lowest value with no alarm on the healthy
  validation runs.
- **Stuck rule:** a sensor whose reading changes by less than 0.5 between
  every pair of the last 32 frames (1 s) while the robot drives (mean
  `(|left| + |right|) / 2` encoder wheel speed over them above 1 rad/s). An
  earlier version required another sensor to move instead; that missed a
  center sensor stuck at its on-the-line value, which takes the robot off
  the track where all three sensors read flat floor.
- **Monitoring:** a window is only monitored when it starts at least 0.5 s
  after the route started and none of its samples is `AT_DEST`, `U_TURN`, a
  corner spin (a junction turn with wheel targets of opposite signs),
  `FAULT_PULLOVER`, `FAULT_STOP` or within 0.5 s after a spin. Each
  detector alarms when 3 of its last 5 windows exceed its threshold.
- **Reset:** a new route command clears the windows, the GRU state and the
  alarm history.
- **esp-dsp:** the matrix work uses Espressif's
  [esp-dsp](https://components.espressif.com/components/espressif/esp-dsp)
  component (`main/idf_component.yml`; fetched into `managed_components/`
  by the first build), whose ESP32-S3 routines use the S3's SIMD
  instructions. Each conv layer is one `dspm_mult_f32` (im2col), each
  dense/GRU row one `dsps_dotprod_f32`. The SIMD path needs 16-byte-aligned
  data and sizes that are multiples of 4, so `export_c.py` writes the
  weights pre-arranged and padded. ESP-DL was not used: its Conv and Gemm
  only run quantized (int8/int16), which would shift the scores against
  thresholds set in float32.
- **Weights in RAM:** the exported weights are in flash; each detector copies
  them into internal RAM at start-up (about 63 KB). Read through the flash
  cache, which the WiFi stack shares, they took ~4.4 ms per frame.
- **Cost** (`-O2`, 160 MHz, WiFi on): about 0.31 ms (motor conv) + 0.36 ms
  (IR conv) + 0.38 ms (GRU) per frame; the scalar C version before esp-dsp
  took 2.3 + 2.0 ms for the two conv models.
- **Agreement:** `scripts/esp/replay_scores.py` replays recorded runs and
  compares the ESP's scores and alarms with Python (see `docs/USAGE.md`).

Why distance gating: on the base track, all three ground sensors see dark track
on the wide horizontal lines as well as at junctions, so the sensor pattern
alone cannot identify a junction or the destination. See
`docs/DECISIONS.md`.

## WiFi notifications

`main/notify.{h,cpp}` joins a WiFi network and posts a notification to
[ntfy](https://ntfy.sh) on every `FAULT_STOP`, on `LINE_LOST` and
`JUNCTION_MISSED` for routes with `F@`, and for `N`. The network and
topic are read at boot from the `wificfg` NVS partition (`partitions.csv`,
12 KB at 0x310000), not compiled in. Write it from `wifi.csv` (copy
`wifi.example.csv`; `wifi.csv` is gitignored) without rebuilding:

```powershell
.\scripts\esp\flash_wifi.ps1   # -Port COM3 -Config firmware\wifi.csv
```

Keys in namespace `wifi`: `ssid`, `password` (may be empty), `ntfy_topic`,
and optionally `ntfy_server` (default `https://ntfy.sh`). Without them WiFi
stays off. Sending runs in its own task on core 0 and never blocks the
control loop, which is pinned to core 1. A message waits up to 120 s for
WiFi and is tried up to 6 times. Each retry clears lwIP's DNS cache:
ntfy.sh rotates between three servers, and on 2026-11-26 one of them
(129.212.146.227) reset every TLS handshake, so a cached address could fail
every attempt. HTTPS uses the ESP-IDF certificate bundle. With WiFi and TLS
the image is about 0xe4000 bytes, in a 3 MB app partition.
Setup and the phone side are in `docs/USAGE.md`.

## Important limitations

- The fault stop's turn side comes from the map (`P@`, chosen per route
  step), but near a junction other lines meet anyway: a pull-over there can
  end ~5 cm from another line. With a faulty drive or encoder the turn and
  drive are only approximate, which the 6 s limit bounds.
- The conv window (20) and channel count (16) are compile-time constants in
  `anomaly.cpp`; the smaller architectures found by
  `scripts/models/search_architectures.py` need them made configurable.

## ESP-DL benchmark app

`esp-dl-bench/` is a separate ESP-IDF app that runs the INT8/INT16 versions
of the detectors (exported by `scripts/models/quantize_int8.py`) with
Espressif's ESP-DL, for `scripts/esp/int8_bench.py`. Flashing it replaces
this firmware (and its partition table, with the default large single-app
one; the `wificfg` data is left in place) until this firmware is flashed
again. Results: `reports/esp-int8-v1/summary.md`.
- Junction distances come from the map scaled per edge
  (`esp_odometry_scale_by_edge`, else `esp_odometry_scale`). A track with
  different curvature may need a re-measured scale, and junctions closer than
  about 0.2 m along a route would make the +/-0.10 m windows overlap.
- Wheel slip makes odometry read short. Slip is small in the current
  simulation; a motor fault that changes the driven path may push a junction
  out of its window and stop the run with `JUNCTION_MISSED`.
- This firmware emits wheel speeds in Webots-compatible rad/s. `epuck_route`
  sends the mission's `esp_speed_rad_s` as the route speed when it is set; the
  Supervisor controller's `wheel_speed_rad_s` does not apply to the ESP.
