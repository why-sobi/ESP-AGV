"""Exercise the ESP32-S3 line-control protocol over its USB serial port."""

import argparse
import time

import serial


def read_line(port, timeout_seconds):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        line = port.readline()
        if line:
            return line.decode("ascii", errors="replace").strip()
    raise TimeoutError("Timed out waiting for an ESP response")


def send_sensor_frame(port, sequence, left, center, right):
    frame = f"S,{sequence},32,{left},{center},{right}\n"
    port.write(frame.encode("ascii"))
    response = read_line(port, 2.0)
    if not response.startswith(f"M,{sequence},"):
        raise AssertionError(f"Unexpected response for sequence {sequence}: {response!r}")

    fields = response.split(",")
    if len(fields) != 6:
        raise AssertionError(f"Malformed motor response: {response!r}")
    return {
        "left_speed": float(fields[2]),
        "right_speed": float(fields[3]),
        "state": fields[4],
        "route_index": int(fields[5]),
        "raw": response,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Check ESP serial commands, PD response, junction routing, and destination stop."
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

        port.write(b"R,LD\n")
        ack = read_line(port, 2.0)
        if ack != "A,ROUTE_OK":
            raise AssertionError(f"Route was not accepted: {ack!r}")
        print(f"PASS route accepted: {ack}")

        sequence = 1
        centered = send_sensor_frame(port, sequence, 700, 300, 700)
        sequence += 1
        if (
            centered["state"] != "FOLLOWING"
            or abs(centered["left_speed"] - 3.5) > 0.05
            or abs(centered["right_speed"] - 3.5) > 0.05
        ):
            raise AssertionError(f"Centered-line PD output incorrect: {centered['raw']}")
        print(f"PASS centered-line PD output: {centered['raw']}")

        left_offset = send_sensor_frame(port, sequence, 300, 700, 700)
        sequence += 1
        if not left_offset["left_speed"] < left_offset["right_speed"]:
            raise AssertionError(f"PD steering direction incorrect: {left_offset['raw']}")
        print(f"PASS PD steers toward left-side line: {left_offset['raw']}")

        send_sensor_frame(port, sequence, 300, 300, 300)
        sequence += 1
        junction = send_sensor_frame(port, sequence, 300, 300, 300)
        sequence += 1
        if (
            junction["state"] != "TURN_LEFT"
            or junction["route_index"] != 1
            or not junction["left_speed"] < junction["right_speed"]
        ):
            raise AssertionError(f"Left-junction action incorrect: {junction['raw']}")
        print(f"PASS junction action consumed once: {junction['raw']}")

        for _ in range(3):
            cleared = send_sensor_frame(port, sequence, 700, 300, 700)
            sequence += 1
        if cleared["state"] != "FOLLOWING":
            raise AssertionError(f"Controller did not clear junction latch: {cleared['raw']}")
        print(f"PASS junction clears before next action: {cleared['raw']}")

        send_sensor_frame(port, sequence, 300, 300, 300)
        sequence += 1
        destination = send_sensor_frame(port, sequence, 300, 300, 300)
        if (
            destination["state"] != "AT_DEST"
            or destination["route_index"] != 2
            or destination["left_speed"] != 0.0
            or destination["right_speed"] != 0.0
        ):
            raise AssertionError(f"Destination did not stop safely: {destination['raw']}")
        print(f"PASS destination action stops motors: {destination['raw']}")

        port.write(b"R,XD\n")
        error = read_line(port, 2.0)
        if error != "E,BAD_ROUTE":
            raise AssertionError(f"Invalid route was not rejected: {error!r}")
        print(f"PASS malformed route rejected: {error}")
    finally:
        port.close()

    print("ESP serial smoke test passed.")


if __name__ == "__main__":
    main()
