"""Exercise the ESP32-S3 line-control protocol over its USB serial port."""

import argparse
import json
import math
import time
from pathlib import Path

import serial

WHEEL_RADIUS_M = 0.02  # Must match kWheelRadiusM in the firmware.
WHEEL_BASE_M = 0.052  # Must match kWheelBaseM in the firmware.
ALL_LIGHT = (700, 700, 700)
CENTERED = (700, 300, 700)
LEFT_OF_ROBOT = (300, 700, 700)
ALL_DARK = (300, 300, 300)
STEP_M = 3.5 * 0.032 * WHEEL_RADIUS_M  # One 32 ms frame at 3.5 rad/s.


def read_line(port, timeout_seconds):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        line = port.readline()
        if line:
            return line.decode("ascii", errors="replace").strip()
    raise TimeoutError("Timed out waiting for an ESP response")


class Session:
    """Send sensor frames for a robot that has driven ``distance_m``."""

    def __init__(self, port):
        self.port = port
        self.sequence = 0

    def route(self, payload):
        self.port.write(f"R,{payload}\n".encode("ascii"))
        return read_line(self.port, 2.0)

    def frame(self, sensors, distance_m, turned_rad=0.0):
        """``turned_rad`` is a left spin in place on top of ``distance_m``."""
        self.sequence += 1
        forward = distance_m / WHEEL_RADIUS_M
        spin = turned_rad * WHEEL_BASE_M / (2.0 * WHEEL_RADIUS_M)
        left, center, right = sensors
        frame = (
            f"S,{self.sequence},32,{left},{center},{right},"
            f"{forward - spin:.6f},{forward + spin:.6f}\n"
        )
        self.port.write(frame.encode("ascii"))
        response = read_line(self.port, 2.0)
        if not response.startswith(f"M,{self.sequence},"):
            raise AssertionError(
                f"Unexpected response for sequence {self.sequence}: {response!r}"
            )

        fields = response.split(",")
        if len(fields) != 14:
            raise AssertionError(f"Malformed motor response: {response!r}")
        return {
            "left_speed": float(fields[2]),
            "right_speed": float(fields[3]),
            "state": fields[4],
            "route_index": int(fields[5]),
            "distance_m": float(fields[6]),
            "motor_score": float(fields[7]),
            "ir_score": float(fields[8]),
            "motor_alarm": int(fields[9]),
            "ir_alarm": int(fields[10]),
            "gru_score": float(fields[11]),
            "gru_alarm": int(fields[12]),
            "stuck_alarm": int(fields[13]),
            "raw": response,
        }


def check(condition, passed, failed):
    if not condition:
        raise AssertionError(failed)
    print(f"PASS {passed}")


def deployed_windows(model_dir="models/esp-conv-v2"):
    """Window of the deployed motor and IR conv models (firmware/main/anomaly_model.h)."""
    from scripts.models.train_autoencoder import MODELS, model_window

    root = Path(__file__).resolve().parents[2]
    config = json.loads((root / model_dir / "config.json").read_text(encoding="utf-8"))
    return {name: model_window(config, name) for name in MODELS}


def main():
    # Read before the port opens: the import is slow enough for the ESP to
    # report a lost link mid-test.
    windows = deployed_windows()
    parser = argparse.ArgumentParser(
        description=(
            "Check ESP serial commands, PD response, distance-gated junction "
            "routing, destination stop, and missed-junction stop."
        )
    )
    parser.add_argument("--port", default="COM3", help="ESP USB serial port (default: COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    args = parser.parse_args()

    port = serial.Serial(
        port=None,
        baudrate=args.baud,
        timeout=0.1,
        write_timeout=1.0,
        dsrdtr=False,
        rtscts=False,
    )
    port.port = args.port
    port.dtr = False
    port.rts = False
    port.open()
    try:
        time.sleep(0.5)
        port.reset_input_buffer()
        esp = Session(port)

        # Turn left at 0.30 m (accepted within +/-0.10 m), stop at 0.60 m.
        ack = esp.route("L@0.30,D@0.60")
        check(ack == "A,ROUTE_OK", f"route accepted: {ack}", f"Route was not accepted: {ack!r}")

        centered = esp.frame(CENTERED, 0.0)
        check(
            centered["state"] == "FOLLOWING"
            and abs(centered["left_speed"] - 3.5) <= 0.05
            and abs(centered["right_speed"] - 3.5) <= 0.05,
            f"centered-line PD output: {centered['raw']}",
            f"Centered-line PD output incorrect: {centered['raw']}",
        )

        left_offset = esp.frame(LEFT_OF_ROBOT, 0.02)
        check(
            left_offset["left_speed"] < left_offset["right_speed"],
            f"PD steers toward left-side line: {left_offset['raw']}",
            f"PD steering direction incorrect: {left_offset['raw']}",
        )
        check(
            abs(left_offset["distance_m"] - 0.02) <= 0.001,
            f"odometry distance reported: {left_offset['raw']}",
            f"Odometry distance incorrect: {left_offset['raw']}",
        )

        esp.frame(ALL_DARK, 0.05)
        early = esp.frame(ALL_DARK, 0.06)
        check(
            early["state"] == "FOLLOWING" and early["route_index"] == 0,
            f"junction pattern ignored outside its window: {early['raw']}",
            f"Junction taken outside its distance window: {early['raw']}",
        )

        esp.frame(CENTERED, 0.10)
        esp.frame(CENTERED, 0.21)  # The window opens on a plain line.
        esp.frame(ALL_DARK, 0.28)
        junction = esp.frame(ALL_DARK, 0.29)
        check(
            junction["state"] == "TURN_LEFT"
            and junction["route_index"] == 1
            and junction["left_speed"] < junction["right_speed"],
            f"junction action consumed inside its window: {junction['raw']}",
            f"Left-junction action incorrect: {junction['raw']}",
        )

        for distance in (0.31, 0.32, 0.33):
            cleared = esp.frame(CENTERED, distance)
        check(
            cleared["state"] == "FOLLOWING",
            f"junction clears before next action: {cleared['raw']}",
            f"Controller did not clear junction latch: {cleared['raw']}",
        )

        before = esp.frame(ALL_DARK, 0.55)
        check(
            before["state"] == "FOLLOWING",
            f"destination not reached early: {before['raw']}",
            f"Destination reported before its distance: {before['raw']}",
        )
        destination = esp.frame(CENTERED, 0.605)
        check(
            destination["state"] == "AT_DEST"
            and destination["route_index"] == 2
            and destination["left_speed"] == 0.0
            and destination["right_speed"] == 0.0,
            f"destination distance stops motors: {destination['raw']}",
            f"Destination did not stop safely: {destination['raw']}",
        )

        ack = esp.route("R@0.30,D@0.60")
        esp.frame(CENTERED, 0.0)
        esp.frame(CENTERED, 0.39)
        missed = esp.frame(CENTERED, 0.41)
        check(
            ack == "A,ROUTE_OK"
            and missed["state"] == "JUNCTION_MISSED"
            and missed["left_speed"] == 0.0
            and missed["right_speed"] == 0.0,
            f"missed junction stops motors: {missed['raw']}",
            f"Missed junction did not stop: {missed['raw']}",
        )

        # Odometry (and the turn angle) is zeroed on a route's first frame.
        ack = esp.route("U@0.000,D@0.30")
        esp.frame(CENTERED, 0.0)
        spinning = esp.frame(CENTERED, 0.0, math.radians(90))
        check(
            ack == "A,ROUTE_OK"
            and spinning["state"] == "U_TURN"
            and spinning["left_speed"] < 0.0 < spinning["right_speed"],
            f"U-turn spins left in place and ignores early lines: {spinning['raw']}",
            f"U-turn did not spin: {spinning['raw']}",
        )
        turned = esp.frame(CENTERED, 0.0, math.radians(170))
        check(
            turned["state"] == "FOLLOWING" and turned["route_index"] == 1,
            f"U-turn ends on the line after 160 degrees: {turned['raw']}",
            f"U-turn did not hand over to line following: {turned['raw']}",
        )

        esp.route("U@0.000,D@0.30")
        esp.frame(ALL_LIGHT, 0.0)
        lost = esp.frame(ALL_LIGHT, 0.0, math.radians(280))
        check(
            lost["state"] == "LINE_LOST"
            and lost["left_speed"] == 0.0
            and lost["right_speed"] == 0.0,
            f"U-turn without a line stops after 270 degrees: {lost['raw']}",
            f"U-turn did not give up: {lost['raw']}",
        )

        # A leading V@ sets the straight-line speed; corrections scale with it.
        ack = esp.route("V@2.0,U@0.000,D@0.30")
        esp.frame(CENTERED, 0.0)
        handed_over = esp.frame(CENTERED, 0.0, math.radians(170))
        slow = esp.frame(CENTERED, 0.01, math.radians(170))
        check(
            ack == "A,ROUTE_OK"
            and handed_over["route_index"] == 1
            and abs(slow["left_speed"] - 2.0) <= 0.05
            and abs(slow["right_speed"] - 2.0) <= 0.05,
            f"route speed V@2.0 sets straight-line speed: {slow['raw']}",
            f"Route speed not applied: {slow['raw']}",
        )
        esp.route("L@0.30,D@0.60")
        default = esp.frame(CENTERED, 0.0)
        check(
            abs(default["left_speed"] - 3.5) <= 0.05,
            f"speed returns to default without V@: {default['raw']}",
            f"Default speed not restored: {default['raw']}",
        )

        # A junction pattern already present when the window opens (a wide
        # line) is not where the junction is: turn only 3 cm before it.
        esp.route("L@0.30,D@0.60")
        esp.frame(CENTERED, 0.0)
        esp.frame(ALL_DARK, 0.15)
        waiting = esp.frame(ALL_DARK, 0.21)
        esp.frame(ALL_DARK, 0.272)
        turning = esp.frame(ALL_DARK, 0.275)
        check(
            waiting["state"] == "FOLLOWING" and waiting["route_index"] == 0
            and turning["state"] == "TURN_LEFT"
            and turning["left_speed"] < turning["right_speed"],
            f"pattern present at window start: turn 3 cm before the junction: "
            f"{turning['raw']}",
            f"Stale junction pattern handled wrongly: {waiting['raw']} / {turning['raw']}",
        )

        # Bear ('<', a Y branch from the map): the turn holds after the
        # junction pattern clears, until 25 degrees have turned.
        esp.route("<@0.30,D@0.60")
        esp.frame(CENTERED, 0.0)
        esp.frame(CENTERED, 0.21)
        early = esp.frame(ALL_DARK, 0.25)  # Too early for a Y branch.
        esp.frame(ALL_DARK, 0.28)
        esp.frame(ALL_DARK, 0.29)
        for distance in (0.30, 0.31, 0.32, 0.33):
            holding = esp.frame(CENTERED, distance, 0.2)
        released = [esp.frame(CENTERED, 0.34 + 0.01 * i, 0.5) for i in range(3)][-1]
        check(
            early["state"] == "FOLLOWING"
            and holding["state"] == "TURN_LEFT" and released["state"] == "FOLLOWING",
            f"bear: turn held until 25 degrees, then follow: {released['raw']}",
            f"Bear turn incorrect: {holding['raw']} / {released['raw']}",
        )

        # Corner ('l', from the map): slow down, spin in place once the axle
        # is on the crossing line, then follow it once 60 degrees have turned.
        ack = esp.route("l@0.30,D@0.60")
        esp.frame(CENTERED, 0.0)
        slow = esp.frame(CENTERED, 0.21)
        esp.frame(ALL_DARK, 0.26)
        esp.frame(ALL_DARK, 0.27)
        corner = esp.frame(ALL_DARK, 0.305)
        midway = esp.frame(ALL_LIGHT, 0.305, 0.5)
        done = esp.frame(CENTERED, 0.305, 1.2)
        check(
            ack == "A,ROUTE_OK"
            and abs(slow["left_speed"] - 1.5) <= 0.05
            and corner["state"] == "TURN_LEFT" and corner["route_index"] == 1
            and corner["left_speed"] < 0.0 < corner["right_speed"]
            and midway["state"] == "TURN_LEFT"
            and done["state"] == "FOLLOWING"
            and max(done["left_speed"], done["right_speed"]) <= 1.5 + 0.01,
            f"corner: slow, spin in place, follow the new line: {done['raw']}",
            f"Corner turn incorrect: {slow['raw']} / {corner['raw']} / {midway['raw']} / "
            f"{done['raw']}",
        )

        # The sensors leave the line in the last few cm (past a line's end or
        # across a T's crossbar): run straight on to the destination.
        esp.route("D@0.60")
        esp.frame(CENTERED, 0.0)
        esp.frame(CENTERED, 0.50)
        blind = esp.frame(ALL_LIGHT, 0.56)
        arrived = esp.frame(ALL_LIGHT, 0.61)
        check(
            blind["state"] == "FOLLOWING"
            and abs(blind["left_speed"] - blind["right_speed"]) <= 0.01
            and blind["left_speed"] > 0.0
            and arrived["state"] == "AT_DEST",
            f"line lost in the last 5 cm: straight on to the destination: {arrived['raw']}",
            f"Arrival run-in incorrect: {blind['raw']} / {arrived['raw']}",
        )

        # Anomaly scores appear once each model's window exists (the
        # deployed models' windows; motor and IR may differ). A sample needs a
        # wheel speed, so the first frame of a route adds none.
        last = max(windows.values()) + 1
        esp.route("D@5.0")
        frames = [esp.frame(CENTERED, 0.001 * i) for i in range(last + 1)]
        check(
            all(frames[w - 1][f"{name}_score"] == -1.0 and frames[w][f"{name}_score"] >= 0.0
                for name, w in windows.items())
            and frames[last]["motor_alarm"] == 0
            and frames[last]["ir_alarm"] == 0,
            f"anomaly scores start after a full window ({windows}): {frames[last]['raw']}",
            f"Anomaly scores not reported as expected for windows {windows}: "
            f"{[frames[w - 1]['raw'] for w in windows.values()]} / {frames[last]['raw']}",
        )
        port.write(b"Q\n")
        timing = read_line(port, 2.0)
        fields = timing.split(",")
        check(
            fields[0] == "I" and len(fields) == 7 and 0 < int(fields[2]) < 20000
            and 0 < int(fields[6]) < 20000,
            f"inference timing reported (us: motor, IR conv, motor GRU last,max): {timing}",
            f"Inference timing missing or too slow: {timing!r}",
        )

        # Fault stop: without F@ an alarm is only reported; with F@<s> an alarm
        # lasting that long makes the robot spin right about 90 degrees, drive
        # forward off the line and stop. Alternating IR readings trip the IR
        # model once the 0.5 s warm-up has passed.
        def drive_with_ir_noise(payload):
            ack = esp.route(payload)
            if ack != "A,ROUTE_OK":
                raise AssertionError(f"Route {payload!r} was not accepted: {ack!r}")
            frames, distance = [], 0.0
            for i in range(120):
                if i < 40:
                    sensors = CENTERED
                else:
                    sensors = (100, 900, 100) if i % 2 else (900, 100, 900)
                frames.append(esp.frame(sensors, distance))
                distance += STEP_M
                if frames[-1]["state"] == "FAULT_PULLOVER":
                    break
            return frames, distance

        frames, _ = drive_with_ir_noise("D@5.0")
        check(
            any(f["ir_alarm"] or f["motor_alarm"] for f in frames)
            and all(f["state"] == "FOLLOWING" for f in frames),
            f"without F@ an alarm is only reported: {frames[-1]['raw']}",
            f"Alarm without F@ changed the behaviour: {frames[-1]['raw']}",
        )

        # E@ names the robot's place per step for the notification.
        frames, distance = drive_with_ir_noise(
            "E@between Junction 1 and Junction 2,F@0.3,D@5.0"
        )
        first_alarm = next(
            (i for i, f in enumerate(frames) if f["ir_alarm"] or f["motor_alarm"]), None
        )
        pulling = frames[-1]
        check(
            first_alarm is not None
            and pulling["state"] == "FAULT_PULLOVER"
            and (len(frames) - 1 - first_alarm) * 0.032 >= 0.3
            and pulling["left_speed"] > 0.0 > pulling["right_speed"],
            f"alarm held 0.3 s starts the pull-over (right spin): {pulling['raw']}",
            f"Fault stop did not start as expected: {pulling['raw']}",
        )
        turned = esp.frame(ALL_LIGHT, distance, math.radians(-95))
        forward = esp.frame(ALL_LIGHT, distance + 0.04, math.radians(-95))
        check(
            turned["state"] == "FAULT_PULLOVER"
            and forward["left_speed"] > 0.0
            and forward["left_speed"] == forward["right_speed"],
            f"after 90 degrees it drives forward off the line: {forward['raw']}",
            f"Pull-over did not drive forward: {forward['raw']}",
        )
        stopped = esp.frame(ALL_LIGHT, distance + 0.09, math.radians(-95))
        after = esp.frame(CENTERED, distance + 0.09, math.radians(-95))
        check(
            stopped["state"] == "FAULT_STOP"
            and stopped["left_speed"] == stopped["right_speed"] == 0.0
            and after["state"] == "FAULT_STOP"
            and after["left_speed"] == after["right_speed"] == 0.0,
            f"pull-over ends in FAULT_STOP with the wheels stopped: {after['raw']}",
            f"Pull-over did not stop: {stopped['raw']} / {after['raw']}",
        )

        # P@ gives the pull-over side per step: 'L' spins left instead.
        frames, distance = drive_with_ir_noise("P@L,F@0.3,D@5.0")
        pulling = frames[-1]
        check(
            pulling["state"] == "FAULT_PULLOVER"
            and pulling["left_speed"] < 0.0 < pulling["right_speed"],
            f"P@L pulls over to the left (left spin): {pulling['raw']}",
            f"P@L pull-over did not spin left: {pulling['raw']}",
        )
        still_turning = esp.frame(ALL_LIGHT, distance, math.radians(-95))
        esp.frame(ALL_LIGHT, distance, math.radians(95))
        forward = esp.frame(ALL_LIGHT, distance + 0.04, math.radians(95))
        stopped = esp.frame(ALL_LIGHT, distance + 0.09, math.radians(95))
        check(
            still_turning["state"] == "FAULT_PULLOVER"
            and still_turning["left_speed"] < 0.0
            and forward["left_speed"] > 0.0
            and forward["left_speed"] == forward["right_speed"]
            and stopped["state"] == "FAULT_STOP",
            f"left pull-over turns 90 degrees left, drives off and stops: {stopped['raw']}",
            f"Left pull-over went wrong: {still_turning['raw']} / {forward['raw']} / "
            f"{stopped['raw']}",
        )

        # A drive that cannot turn (e.g. stalled) still stops after 6 s.
        frames, distance = drive_with_ir_noise("F@0.3,D@5.0")
        for _ in range(195):
            stuck = esp.frame(ALL_LIGHT, distance)
            if stuck["state"] != "FAULT_PULLOVER":
                break
        check(
            stuck["state"] == "FAULT_STOP" and stuck["left_speed"] == 0.0,
            f"pull-over that cannot turn stops after its time limit: {stuck['raw']}",
            f"Stuck pull-over did not stop: {stuck['raw']}",
        )

        # WiFi is optional; each FAULT_STOP above queued a notification.
        port.write(b"W\n")
        wifi = read_line(port, 2.0)
        check(
            wifi.startswith("W,"),
            f"WiFi status (state, IP, last notification): {wifi}",
            f"WiFi status missing: {wifi!r}",
        )

        for payload in (
            "E@edge-001|edge-002,D@0.3",
            "F@0.0,D@0.3",
            "F@20,D@0.3",
            "R@0.1,F@1.0,D@0.3",
            "XD",
            "V@9.0,D@0.3",
            "V@0.5,D@0.3",
            "R@0.1,V@2.0,D@0.3",
            "V@2.0",
            "RD",
            "R@0.3",
            "R@0.5,D@0.4",
            "D@0.2,D@0.4",
            "R@0.1,U@0.2,D@0.3",
            "P@X,D@0.3",
            "P@,D@0.3",
            "P@RL,D@0.3",
            "P@R,R@0.1,D@0.3",
            "R@0.1,P@RR,D@0.3",
        ):
            error = esp.route(payload)
            check(
                error == "E,BAD_ROUTE",
                f"malformed route {payload!r} rejected: {error}",
                f"Invalid route {payload!r} was not rejected: {error!r}",
            )
    finally:
        port.close()

    print("ESP serial smoke test passed.")


if __name__ == "__main__":
    main()
