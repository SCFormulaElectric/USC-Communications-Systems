"""Check T9 results against an on-air collision model.

Model: a module goes on air WINDOW s after a write starts (T2: ~101 ms), then stays on air
CAR_AIR (3 x 64 B batch) or CMD_AIR (24 B command). A command or telemetry batch is
predicted lost when its on-air time overlaps the other direction's.

Usage: py t9_collision_check.py [car_air_ms cmd_air_ms]   (default 130 25)
"""
import os
import sys

from lora_common import load_csv
from framing_tests import LOGS, parse_stream

WINDOW = 0.101
CAR_AIR = float(sys.argv[1]) / 1000 if len(sys.argv) > 1 else 0.130
CMD_AIR = float(sys.argv[2]) / 1000 if len(sys.argv) > 2 else 0.025


def overlaps(s, e, intervals):
    return any(s < b and a < e for a, b in intervals)


for t in ["t9b", "t9c", "t9d"]:
    path = os.path.join(LOGS, f"{t}_raw.csv")
    if not os.path.exists(path):
        continue
    writes, reads, _ = load_csv(path)
    car = [w for w in writes if w["tag"]["dir"] == "car"]
    cmd = [w for w in writes if w["tag"]["dir"] == "cmd"]
    car_iv = [(w["t_start"] + WINDOW, w["t_start"] + WINDOW + CAR_AIR) for w in car]
    cmd_iv = [(w["t_start"] + WINDOW, w["t_start"] + WINDOW + CMD_AIR) for w in cmd]

    got_cmd = {f for f, _, _ in parse_stream(reads["TX"], 24)[0]}
    got_tel = {f for f, _, _ in parse_stream(reads["RX"], 64)[0]}

    c = {(o, d): 0 for o in (0, 1) for d in (0, 1)}
    for w, (s, e) in zip(cmd, cmd_iv):
        c[(int(overlaps(s, e, car_iv)), int(w["data"][3:] in got_cmd))] += 1
    m = {(o, d): 0 for o in (0, 1) for d in (0, 1)}
    for w, (s, e) in zip(car, car_iv):
        body = w["data"][3:]
        ok = all(body[j:j + 64] in got_tel for j in range(0, len(body), 64))
        m[(int(overlaps(s, e, cmd_iv)), int(ok))] += 1

    def line(name, tab, n):
        agree = 100 * (tab[(1, 0)] + tab[(0, 1)]) / n
        return (f"  {name}: overlap -> lost {tab[(1, 0)]}, ok {tab[(1, 1)]} | "
                f"no overlap -> ok {tab[(0, 1)]}, lost {tab[(0, 0)]} | model matches {agree:.0f}%")

    print(f"{t} (car air {CAR_AIR * 1000:.0f} ms, command air {CMD_AIR * 1000:.0f} ms)")
    print(line("commands ", c, len(cmd)))
    print(line("telemetry", m, len(car)))
