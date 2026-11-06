#!/usr/bin/env python3
"""Bench test 2 helper: a one-shot Bamocar read every N seconds for M minutes, with logging.

    py read_loop.py --port COM9 [--reg 0x25] [--every 2] [--minutes 10] [--log-dir logs]

Prints a summary at the end: commands acknowledged / failed, lockups, telemetry received / missing.
commands.csv in the log folder has every attempt.
"""
import argparse
import collections
import time

from decoder import Schema
from logger import TelemetryLogger
from lora_link import LoraLink


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--reg", default="0x25")
    ap.add_argument("--every", type=float, default=2.0, help="seconds between reads")
    ap.add_argument("--minutes", type=float, default=10.0)
    ap.add_argument("--log-dir", default="logs")
    a = ap.parse_args()
    s = Schema()
    lg = TelemetryLogger(a.log_dir, s)
    link = LoraLink(a.port, s, logger=lg)
    results, states = collections.Counter(), []
    link.listeners.append(lambda ev: results.update([s.status_name(ev["status"]) if ev["status"] is not None
                                                     else "GAVE_UP"]) if ev["kind"] == "result" else None)
    link.listeners.append(lambda ev: states.append((time.time(), ev["text"])) if ev["kind"] == "state" else None)
    link.start()
    reg = s.parse_reg(a.reg)
    print(f"logging to {lg.raw_path}; reading {s.reg_name(reg)} every {a.every} s for {a.minutes} min")
    t_end = time.time() + a.minutes * 60
    try:
        link.submit("PING")
        while time.time() < t_end:
            link.submit("BAMO_READ", {"reg": reg, "period": 0})
            time.sleep(a.every)
            st = link.stats()
            print(f"\r{st['state']:<8} {st['pps']:4.1f} pkt/s loss {st['loss_pct']:4.1f}% "
                  f"results {dict(results)}   ", end="", flush=True)
        link.submit("STOP_ALL")
        time.sleep(3)
    finally:
        st = link.stats()
        link.stop()
        lg.close()
    print(f"\ncommand results: {dict(results)}")
    print(f"link state changes: {states or 'none'}")
    print(f"telemetry: received {st['received']}, missing {st['missing']}, bad CRC {st['bad_crc']}, "
          f"PC 'Power on' {st['power_on']}")


if __name__ == "__main__":
    main()
