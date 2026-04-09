#!/usr/bin/env python3
"""
serial_dump.py — Run this on the Raspberry Pi.

Reads structured CSV lines from the ESP32 over USB Serial and appends them
to a dump file.  Only lines that begin with "DATA," are written; all other
Serial output (debug text) is printed to the console so you can watch it live.

Usage:
    python3 serial_dump.py [--port /dev/ttyUSB0] [--baud 115200] [--out dump.csv]

Dependencies:
    pip install pyserial
"""

import argparse
import csv
import sys
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError:
    sys.exit("pyserial not found. Install it with:  pip install pyserial")


HEADER = ["wall_time", "esp_ms", "d1_m", "d2_m", "d3_m", "pos_x_m", "pos_y_m"]


def parse_args():
    p = argparse.ArgumentParser(description="ESP32 serial data logger")
    p.add_argument("--port", default="/dev/ttyUSB0",
                   help="Serial port the ESP32 is on (default: /dev/ttyUSB0). "
                        "Try /dev/ttyACM0 if USB0 does not exist.")
    p.add_argument("--baud", type=int, default=115200,
                   help="Baud rate (default: 115200)")
    p.add_argument("--out", default="dump.csv",
                   help="Output CSV file (default: dump.csv)")
    return p.parse_args()


def main():
    args = parse_args()
    out_path = Path(args.out)
    write_header = not out_path.exists()

    print(f"Opening {args.port} @ {args.baud} baud")
    print(f"Logging DATA lines to: {out_path.resolve()}")
    print("Press Ctrl+C to stop.\n")

    with serial.Serial(args.port, args.baud, timeout=2) as ser, \
         open(out_path, "a", newline="") as fh:

        writer = csv.writer(fh)
        if write_header:
            writer.writerow(HEADER)
            fh.flush()

        while True:
            try:
                raw = ser.readline()
            except serial.SerialException as e:
                sys.exit(f"\nSerial error: {e}")

            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()

            if line.startswith("DATA,"):
                # Expected format: DATA,<esp_ms>,<d1>,<d2>,<d3>,<posX>,<posY>
                parts = line.split(",")
                if len(parts) == 7:
                    wall_time = datetime.now().isoformat(timespec="milliseconds")
                    row = [wall_time] + parts[1:]   # drop the "DATA" tag
                    writer.writerow(row)
                    fh.flush()                      # ensure data reaches disk
                    print(f"[LOGGED] {row}")
                else:
                    print(f"[MALFORMED] {line}")
            else:
                # Pass debug / human-readable output straight to console
                print(f"[ESP32]  {line}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nLogger stopped.")