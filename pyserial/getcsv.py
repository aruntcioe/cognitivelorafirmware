#!/usr/bin/env python3
import serial
import sys
from datetime import datetime

PORT = "COM8"          # change to your port, e.g. "/dev/ttyUSB0" on Linux/Mac
BAUD = 115200
OUTFILE = f"dataset_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

HEADER = "meanRSSI,varRSSI,meanSNR,varSNR,CFO,PLR,CRC,SF,CR,meanToA,label"

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"[PC] Listening on {PORT} at {BAUD} baud")
    print(f"[PC] Writing to {OUTFILE}")

    with open(OUTFILE, "w", newline="") as f:
        f.write(HEADER + "\n")
        f.flush()
        row_count = 0

        try:
            while True:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                if line.startswith("CSVROW,"):
                    row = line[len("CSVROW,"):]
                    f.write(row + "\n")
                    f.flush()          # commit immediately, row-by-row
                    row_count += 1
                    print(f"[{row_count}] {row}")
                else:
                    # everything else (diagnostics) just echoed to your terminal, not saved
                    print(f"    . {line}")
        except KeyboardInterrupt:
            print(f"\n[PC] Stopped. {row_count} rows saved to {OUTFILE}")

if __name__ == "__main__":
    main()