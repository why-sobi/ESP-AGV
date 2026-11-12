# ESP32-S3 Controller

This ESP-IDF project targets the ESP32-S3 and uses the board's CH343 USB-to-UART
connection (currently COM3) on UART0 GPIO43 (TX) and GPIO44 (RX) at 115200 baud.
It is an initial **serial/Webots controller prototype**: Webots supplies
simulated ground-sensor values, and the
ESP returns wheel targets for Webots to apply. The planned use is simulation;
this firmware does not directly drive a physical motor or sensor.

## Build

Use the VS Code ESP-IDF PowerShell terminal configured for ESP-IDF 6.0.1:

```powershell
cd esp32-s3
idf.py set-target esp32s3
idf.py build
```

Do not flash while another program (including the IDF monitor) has COM3 open.
Flashing is not required to build.

## Check the ESP over COM3

Close the ESP-IDF monitor, then run this from the repository root in an
ESP-IDF terminal (which provides the `pyserial` dependency):

```powershell
python scripts\esp_serial_smoke_test.py --port COM3
```

The test configures a short route and sends simulated sensor frames. It checks
centered-line speed, PD steering direction, one-time junction action, junction
clearing, stop-at-destination, and invalid-route rejection. It does not involve
Webots or physical sensors/motors.

## Serial protocol

Lines are UTF-8/ASCII, newline-terminated, 8-N-1, 115200 baud. Only one program
may own COM3 at a time.

- Configure one route: `R,<actions>\n`; actions are `L`, `S`, or `R` for each
  detected junction, followed by a required terminal `D` (destination), e.g.
  `R,LSRD\n`. The controller replies `A,ROUTE_OK\n`.
- Supply sensor frames: `S,<sequence>,<dt_ms>,<left>,<center>,<right>\n`.
  Lower sensor values mean darker track, matching the current Webots sensors.
- Read motor frames:
  `M,<sequence>,<left_rad_s>,<right_rad_s>,<state>,<route_index>\n`.
- Errors are returned as `E,<reason>\n`.

The controller follows the line using the existing PD gains, counts a junction
when both side sensors see the dark line for consecutive frames, and applies the
next route action until the side sensors clear. `D` stops at the next detected
junction. After unloading, send a new route command for the return leg (planned
from destination back to dock) to reset the route state and resume.

If the host stops sending sensor frames, the controller returns zero wheel
targets and latches `LINK_LOST`. If the line is lost during a commanded turn,
it searches slowly in the turn direction for a bounded time, then stops.

## Important limitations

- The Python Webots bridge and graph-route-to-turn conversion are not
  implemented yet, so the current Webots world does not use this firmware for
  route/control computation.
- The motor autoencoder needs target and measured wheel features that the
  current serial frame does not carry. ESP-side anomaly inference is not yet
  implemented.
- Junction detection with three ground sensors is a heuristic. Sensor spacing,
  track width, turn geometry, debounce timing, and route actions must be tested
  against the actual track; missed/double junctions can desynchronize a route.
- The terminal `D` action assumes the destination is a detectable junction.
  A non-junction destination needs a marker or another arrival signal.
- This firmware emits wheel speeds in Webots-compatible rad/s. The standalone
  COM3 smoke test verifies serial computation, not a closed Webots/ESP control
  loop.
