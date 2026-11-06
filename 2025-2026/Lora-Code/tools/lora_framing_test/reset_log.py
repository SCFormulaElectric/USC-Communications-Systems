"""T0 substitute for a power cycle: log both ports while the TX module soft-reboots.
Writes '+++' (enter AT mode), then 'AT+RESET', and logs 12 s. No settings are changed.
Usage: py reset_log.py COM8 COM9
"""
import os
import sys
import time

import serial

from lora_common import BAUD, Recorder

tx = serial.Serial(sys.argv[1], BAUD, timeout=0.001)
rx = serial.Serial(sys.argv[2], BAUD, timeout=0.001)
rec = Recorder([("TX", tx), ("RX", rx)])
rec.start()
time.sleep(2.0)
rec.write(tx, b"+++\r\n", {"test": "T0", "cmd": "+++"})
time.sleep(1.0)
rec.write(tx, b"AT+RESET\r\n", {"test": "T0", "cmd": "AT+RESET"})
time.sleep(12.0)
rec.stop()
rec.save(os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "t0_reset_raw.csv"))
for r in sorted(rec.rows, key=lambda r: r[0]):
    print(f"t={r[0] - rec.t0:7.3f}s {r[1]:9s} {len(r[2]):4d} B  {r[2].decode('ascii', 'backslashreplace')!r}")
