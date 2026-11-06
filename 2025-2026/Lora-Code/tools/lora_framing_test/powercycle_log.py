"""T0 power-cycle logger: logs RX port the whole time and opens the TX port as soon as it
(re)appears, logging everything it prints. Writes logs/t0_powercycle_raw.csv.
Usage: py powercycle_log.py COM8 COM9 seconds
"""
import os
import sys
import time

import serial

from lora_common import BAUD, PortLogger, Recorder, now

tx_port, rx_port, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
rx = serial.Serial(rx_port, BAUD, timeout=0.001)
rec = Recorder([("RX", rx)], reopen=True)
rec.start()
t_end = now() + secs
tx = None
while now() < t_end:
    try:
        tx = serial.Serial(tx_port, BAUD, timeout=0.001)
        break
    except serial.SerialException:
        time.sleep(0.02)
if tx:
    with rec.lock:
        rec.rows.append((now(), "TX_event", b"PORT OPENED", None, ""))
    lg = PortLogger(tx, "TX", rec.rows, rec.lock, reopen=True)
    lg.start()
    rec.loggers.append(lg)
    print(f"{tx_port} opened at t={now() - rec.t0:.2f}s", flush=True)
time.sleep(max(0.0, t_end - now()))
rec.stop()
rec.save(os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "t0_powercycle_raw.csv"))
print("saved")
