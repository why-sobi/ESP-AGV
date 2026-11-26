"""Show the ESP's WiFi status and optionally send a test notification."""

import argparse
import time

import serial


def ask(port, command, timeout_seconds=2.0):
    port.write(f"{command}\n".encode("ascii"))
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        line = port.readline()
        if line:
            return line.decode("ascii", errors="replace").strip()
    raise TimeoutError(f"ESP did not answer {command!r}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM3", help="ESP USB serial port (default: COM3)")
    parser.add_argument("--test", action="store_true", help="send a test notification")
    parser.add_argument(
        "--wait", type=float, default=60.0, help="seconds to wait for WiFi and the send"
    )
    args = parser.parse_args()

    port = serial.Serial(
        port=None, baudrate=115200, timeout=0.1, write_timeout=1.0, dsrdtr=False, rtscts=False
    )
    port.port = args.port
    port.dtr = False
    port.rts = False
    port.open()
    try:
        time.sleep(0.5)
        port.reset_input_buffer()
        deadline = time.monotonic() + args.wait
        status = ask(port, "W")
        print(status)
        if status.startswith("W,OFF"):
            print("WiFi is not configured: run scripts\\esp\\flash_wifi.ps1.")
            return
        while not status.startswith("W,CONNECTED") and time.monotonic() < deadline:
            time.sleep(1.0)
            status = ask(port, "W")
            print(status)
        if not args.test:
            return
        print(ask(port, "N"))
        # The last-result field goes SENDING_<attempt>, then HTTP_<status> or
        # an error; up to 3 attempts.
        sending_seen = False
        previous = None
        while time.monotonic() < deadline:
            time.sleep(0.5)
            status = ask(port, "W")
            result = status.split(",")[3]
            if result != previous:
                print(status)
                previous = result
            if result.startswith("SENDING"):
                sending_seen = True
            elif sending_seen:
                print("Sent." if result == "HTTP_200" else "Send failed.")
                return
        print("Timed out waiting for the send.")
    finally:
        port.close()


if __name__ == "__main__":
    main()
