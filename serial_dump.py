#!/usr/bin/env python3
"""
serial_dump.py — Run this on the Raspberry Pi.
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

HEADER = ["wall_time", "esp_ms", "d1_m", "d6_m", "d11_m", "pos_x_m", "pos_y_m"]

def parse_args():
    p = argparse.ArgumentParser(description="ESP32 serial data logger")
    p.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate")
    p.add_argument("--out", default="dump.csv", help="Output CSV file")
    return p.parse_args()

def calculate_checksum(payload_str):
    """Calculates XOR checksum of a string"""
    chk = 0
    for char in payload_str:
        chk ^= ord(char)
    return chk

def main():
    args = parse_args()
    out_path = Path(args.out)
    write_header = not out_path.exists()

    print(f"Opening {args.port} @ {args.baud} baud")
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

            # Check for our Sync character '$' and 'DATA' tag
            if line.startswith("$DATA,"):
                try:
                    # Split into Payload and Checksum (e.g. "$DATA,123,1,2,3,4,5" and "A1")
                    packet_str, hex_crc_str = line.split('*')
                    
                    # The payload to check excludes the '$'
                    payload_str = packet_str[1:] 
                    
                    # Verify Checksum
                    calc_crc = calculate_checksum(payload_str)
                    recv_crc = int(hex_crc_str, 16)
                    
                    if calc_crc != recv_crc:
                        print(f"[CRC ERROR] Dropped corrupted packet. Calc: {calc_crc:02X}, Recv: {recv_crc:02X}")
                        continue
                    
                    # Parse valid data
                    parts = payload_str.split(",")
                    if len(parts) == 7:
                        wall_time = datetime.now().isoformat(timespec="milliseconds")
                        row = [wall_time] + parts[1:]   # drop the "DATA" tag
                        writer.writerow(row)
                        fh.flush()                      
                        print(f"[LOGGED] {row}")
                    else:
                        print(f"[MALFORMED] Incorrect number of fields: {line}")
                        
                except ValueError:
                    print(f"[PARSE ERROR] Missing checksum or bad format: {line}")
            else:
                # Pass normal debug output straight to console
                print(f"[ESP32]  {line}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nLogger stopped.")