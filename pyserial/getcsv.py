#!/usr/bin/env python3
import argparse
import csv
import sys
import time
from datetime import datetime

import serial

HEADER = ["meanRSSI", "varRSSI", "meanSNR", "varSNR", "CFO", "PLR",
          "CRC", "SF", "CR", "meanToA", "label"]


def parse_args():
    p = argparse.ArgumentParser(description="LoRa feature-vector dataset logger")
    p.add_argument("--port", default=None, help="e.g. COM8 or /dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--label", type=int, default=None, choices=[0, 1, 2, 3],
                   help="Class label for THIS run: 0=clear 1=jammed 2=fading 3=excellent")
    p.add_argument("--outfile", default=None)
    p.add_argument("--reconnect-delay", type=float, default=2.0)
    args = p.parse_args()

    if args.port is None:
        args.port = input("Serial port (e.g. COM8): ").strip()

    if args.label is None:
        while True:
            raw = input("Label for this run [0=clear 1=jammed 2=fading 3=excellent]: ").strip()
            if raw in ("0", "1", "2", "3"):
                args.label = int(raw)
                break
            print("  enter 0, 1, 2, or 3")

    return args


def open_serial(port, baud):
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=1)
            print(f"[PC] Connected to {port} at {baud} baud")
            return ser
        except serial.SerialException as e:
            print(f"[PC] Could not open {port}: {e} -- retrying in 2s")
            time.sleep(2)


def main():
    args = parse_args()

    outfile = args.outfile or f"dataset_class{args.label}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"[PC] Label for this run : {args.label}")
    print(f"[PC] Writing to         : {outfile}")

    ser = open_serial(args.port, args.baud)

    with open(outfile, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(HEADER)
        f.flush()
        row_count = 0

        try:
            while True:
                try:
                    raw = ser.readline()
                except serial.SerialException as e:
                    print(f"[PC] Serial error: {e} -- attempting reconnect")
                    ser.close()
                    ser = open_serial(args.port, args.baud)
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                if line.startswith("CSVROW,"):
                    fields = line[len("CSVROW,"):].split(",")

                    if len(fields) != len(HEADER) - 1:
                        print(f"    ! malformed row ({len(fields)} fields, "
                              f"expected {len(HEADER) - 1}): {line}")
                        continue

                    fields.append(str(args.label))
                    writer.writerow(fields)
                    f.flush()
                    row_count += 1
                    print(f"[{row_count}] {','.join(fields)}")
                else:
                    print(f"    . {line}")

        except KeyboardInterrupt:
            print(f"\n[PC] Stopped. {row_count} rows saved to {outfile}")
        finally:
            ser.close()


if __name__ == "__main__":
    main()