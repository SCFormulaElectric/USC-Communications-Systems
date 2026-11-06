#!/usr/bin/env python3
"""Print what the PC-side LoRa module receives, as text (for the setup tool's radio report).

    py listen.py COM8            # Ctrl+C to stop

The setup tool sends two lines every 3 s once the board's module is set up:
    LORA MODULE SETUP OK. BEFORE: baud 9600 | MODE 1 | LEVEL 7 >> ... | SF 12 | MAC 0a,02 | ...
    AFTER: baud 115200 | MODE 1 | LEVEL 7 >> 13020.833984bps | SF 5 | MAC 0a,02 | ...
Telemetry packets (after you flash the telemetry firmware again) show up as binary; use
telemetry/pc/telemetry_cli.py for those.
"""
import sys
import time

import serial


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    ser = serial.Serial(sys.argv[1], 115200, timeout=0.2)
    print(f"listening on {sys.argv[1]} at 115200 (Ctrl+C to stop)")
    line = bytearray()
    try:
        while True:
            data = ser.read(ser.in_waiting or 1)
            for b in data:
                if b == 0x0A:
                    text = line.decode("ascii", "replace").rstrip("\r")
                    printable = sum(32 <= c < 127 for c in line) >= 0.9 * max(1, len(line))
                    print(time.strftime("%H:%M:%S"), text if printable else f"[{len(line)} binary bytes]")
                    line.clear()
                else:
                    line.append(b)
                    if len(line) > 400:
                        print(time.strftime("%H:%M:%S"), f"[{len(line)} bytes without a newline: binary]")
                        line.clear()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
