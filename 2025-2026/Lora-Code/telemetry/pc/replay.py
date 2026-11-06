#!/usr/bin/env python3
"""Rebuild the per-type CSVs from a raw log (BUILD_INSTRUCTIONS.md 7.3).

    py replay.py logs/raw_20261105_201500.bin [--out logs/replay_20261105_201500]

Writes fast.csv, therm.csv, diag.csv, events.csv, diag_resp.csv (commands.csv is not in the raw
log). Prints packet counts and sequence gaps.
"""
import argparse
import collections
import os

from decoder import Schema
from logger import TelemetryLogger, read_raw


def replay(raw_path, out_dir=None, schema=None):
    schema = schema or Schema()
    base = os.path.splitext(os.path.basename(raw_path))[0].replace("raw_", "")
    out_dir = out_dir or os.path.join(os.path.dirname(raw_path) or ".", f"replay_{base}")
    lg = TelemetryLogger(os.path.dirname(out_dir) or ".", schema, raw=False, csv_dir=out_dir)
    lg.t_offset = 0.0                     # raw times are written as recorded
    counts, last, missing = collections.Counter(), None, 0
    for t, pkt in read_raw(raw_path):
        d = schema.decode_packet(pkt, lg.ctx)
        counts[d["type_name"]] += 1
        if last is not None:
            diff = (d["seq"] - last) & 0xFF
            if 1 < diff < 128:
                missing += diff - 1
        last = d["seq"]
        lg.write_packet(t, pkt, d, raw=False)
    lg.close()
    return out_dir, counts, missing


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("raw")
    ap.add_argument("--out")
    a = ap.parse_args()
    out, counts, missing = replay(a.raw, a.out)
    print(f"wrote CSVs to {out}")
    print("packets: " + ", ".join(f"{k} {v}" for k, v in sorted(counts.items())) + f"; missing (seq gaps) {missing}")


if __name__ == "__main__":
    main()
