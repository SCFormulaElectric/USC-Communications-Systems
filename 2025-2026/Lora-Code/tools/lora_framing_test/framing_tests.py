#!/usr/bin/env python3
"""DX-LR02 UART->radio framing tests (T0-T6). See lora_framing_results.md.

Run a test (logs raw CSV to logs/<test>_raw.csv, then analyzes it):
    py framing_tests.py run t1 --tx COM8 --rx COM9
Re-analyze a saved log with a different arrival-group gap:
    py framing_tests.py analyze t1 --gap-ms 5

Modules are in MODE1 (fixed-point): every UART write that starts a packet carries the
3-byte header [RX addr hi][RX addr lo][channel] = 0A 01 03, which the RX module strips.
"""

import argparse
import difflib
import json
import os
import random
import struct
import sys
import time

from lora_common import (LoRaModule, Recorder, busy_wait_until, crc16_ccitt, group_reads,
                         in_window, load_csv, now, pattern, pct, wire_end, wire_time)

sys.setswitchinterval(0.0002)  # let reader threads grab the GIL quickly during busy-waits

HDR = bytes([0x0A, 0x01, 0x03])
HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.join(HERE, "logs")

T1_SIZES = [8, 32, 64, 100, 128, 200, 239, 240, 241, 255, 256, 300, 512]
T2_PAUSES_MS = [0, 0.5, 1, 2, 3, 5, 8, 10, 15, 20, 30, 50]
T5_PAUSES_MS = [0, 5, 20, 50]


# --------------------------------------------------------------------------- helpers
def describe_match(expected, got):
    expected, got = bytes(expected), bytes(got)
    if got == expected:
        return "exact"
    if not got:
        return "nothing received"
    i = got.find(expected) if expected else -1
    if i >= 0:
        return f"exact + {i} B prefix {got[:i].hex()} + {len(got) - i - len(expected)} B suffix " \
               f"{got[i + len(expected):].hex()}"
    if expected.startswith(got):
        return f"truncated: first {len(got)} of {len(expected)} B"
    sm = difflib.SequenceMatcher(None, expected, got, autojunk=False)
    blocks = [(a, b, n) for a, b, n in sm.get_matching_blocks() if n]
    parts = ", ".join(f"exp[{a}:{a + n}]->got[{b}:{b + n}]" for a, b, n in blocks)
    return f"mismatch ({len(got)} B vs {len(expected)} B expected); matching runs: {parts}"


def ms(x):
    return None if x is None else round(x * 1000, 2)


def write_summary(name, summary, text):
    with open(os.path.join(LOGS, f"{name}_summary.json"), "w") as f:
        json.dump(summary, f, indent=1)
    with open(os.path.join(LOGS, f"{name}_summary.txt"), "w") as f:
        f.write(text + "\n")
    print(text)


def open_pair(args):
    tx = LoRaModule("TX", args.tx)
    rx = LoRaModule("RX", args.rx)
    for m in (tx, rx):
        m.ser.reset_input_buffer()
    rec = Recorder([("TX", tx.ser), ("RX", rx.ser)])
    rec.start()
    return tx, rx, rec


def close_pair(name, tx, rx, rec, tail=2.0):
    time.sleep(tail)  # catch stragglers
    rec.stop()
    path = os.path.join(LOGS, f"{name}_raw.csv")
    rec.save(path)
    tx.close()
    rx.close()
    print(f"raw log: {path}")


def windows(writes, key, max_window=3.0):
    """Group writes into units (by key fn) and give each unit a read window that ends
    at the next unit's first write."""
    units = []
    for w in writes:
        k = key(w)
        if units and units[-1][0] == k:
            units[-1][1].append(w)
        else:
            units.append((k, [w]))
    out = []
    for i, (k, ws) in enumerate(units):
        t_a = ws[0]["t_start"]
        t_b = units[i + 1][1][0]["t_start"] if i + 1 < len(units) else t_a + max_window
        out.append((k, ws, t_a, min(t_b, t_a + max_window)))
    return out


# --------------------------------------------------------------------------- T0
def run_t0(args):
    tx = LoRaModule("TX", args.tx)
    rx = LoRaModule("RX", args.rx)
    rec = Recorder([("TX", tx.ser), ("RX", rx.ser)], reopen=True)
    rec.start()
    print(f"logging both ports for {args.duration:.0f} s ...", flush=True)
    time.sleep(args.duration)
    rec.stop()
    name = args.name or "t0"
    rec.save(os.path.join(LOGS, f"{name}_raw.csv"))
    tx.close()
    rx.close()
    analyze_t0(name, args)


def analyze_t0(name, args):
    writes, reads, events = load_csv(os.path.join(LOGS, f"{name}_raw.csv"))
    lines = [f"{name}: anything printed by either port"]
    s = {}
    for port in ("TX", "RX"):
        data = b"".join(d for _, d in reads[port])
        s[port] = {"reads": len(reads[port]), "bytes": len(data), "hex": data.hex(),
                   "ascii": data.decode("ascii", "backslashreplace")}
        lines.append(f"  {port}: {len(reads[port])} reads, {len(data)} bytes")
        for t, d in reads[port]:
            lines.append(f"    t={t:8.3f}s  {len(d):4d} B  {d.decode('ascii', 'backslashreplace')!r}")
    s["events"] = [(t, p, d.decode()) for t, p, d in events]
    for t, p, d in events:
        lines.append(f"  event t={t:8.3f}s {p}: {d.decode()}")
    write_summary(name, s, "\n".join(lines))


# --------------------------------------------------------------------------- T1
def run_t1(args):
    tx, rx, rec = open_pair(args)
    w = 0
    t_next = now() + 0.5
    for size in T1_SIZES:
        for rep in range(args.reps):
            busy_wait_until(t_next)
            rec.write(tx.ser, HDR + pattern(w, size), {"test": "T1", "size": size, "rep": rep, "w": w})
            w += 1
            t_next = now() + (1.0 if size <= 200 else 2.0)
    close_pair(args.name or "t1", tx, rx, rec, tail=3.0)
    analyze_t1(args)


def analyze_t1(args):
    gap = args.gap_ms / 1000
    writes, reads, _ = load_csv(os.path.join(LOGS, f"{args.name or 't1'}_raw.csv"))
    units = windows(writes, key=lambda w: w["tag"]["w"])
    rows, gaps64 = [], []
    for _, ws, t_a, t_b in units:
        wr = ws[0]
        rx_reads = in_window(reads["RX"], t_a, t_b)
        tx_reads = in_window(reads["TX"], t_a, t_b)
        groups = group_reads(rx_reads, gap)
        got = b"".join(bytes(g["data"]) for g in groups)
        expected = wr["data"][len(HDR):]
        t_we = wire_end(wr)
        if wr["tag"]["size"] == 64:
            ts = [t for t, _ in rx_reads]
            gaps64 += [b - a for a, b in zip(ts, ts[1:])]
        rows.append({
            "size": wr["tag"]["size"], "rep": wr["tag"]["rep"],
            "uart_write_bytes": len(wr["data"]),
            "groups": [len(g["data"]) for g in groups],
            "group_reads": [g["nreads"] for g in groups],
            "group_start_ms": [ms(g["t_first"] - t_we) for g in groups],
            "group_end_ms": [ms(g["t_last"] - t_we) for g in groups],
            "rx_total": len(got),
            "match": describe_match(expected, got),
            "group_match": [describe_match(expected, bytes(g["data"])) for g in groups],
            "latency_last_ms": ms(groups[-1]["t_last"] - t_we) if groups else None,
            "latency_first_ms": ms(groups[0]["t_first"] - t_we) if groups else None,
            "tx_echo": b"".join(d for _, d in tx_reads).decode("ascii", "backslashreplace"),
        })
    lines = [f"T1 single writes (arrival-group gap {args.gap_ms} ms; latency from estimated UART wire end)"]
    lines.append(f"  read-to-read gaps inside 64 B arrivals: n={len(gaps64)} "
                 f"max={ms(max(gaps64)) if gaps64 else None} ms "
                 f"p95={ms(pct(gaps64, 95))} ms p50={ms(pct(gaps64, 50))} ms")
    lines.append(f"  {'size':>4} {'rep':>3}  groups(B)            total  latency first/last ms   match")
    for r in rows:
        lines.append(f"  {r['size']:4d} {r['rep']:3d}  {str(r['groups']):20s} {r['rx_total']:5d}  "
                     f"{str(r['latency_first_ms']):>8}/{str(r['latency_last_ms']):<8}      {r['match'][:90]}"
                     + (f"  TX-echo={r['tx_echo']!r}" if r["tx_echo"] else ""))
    write_summary(args.name or "t1", {"gap_ms": args.gap_ms, "gaps64_ms": [ms(g) for g in gaps64], "rows": rows},
                  "\n".join(lines))


# --------------------------------------------------------------------------- T2 / T5 (pairs of writes)
def run_pairs(args, test):
    tx, rx, rec = open_pair(args)
    pauses = T2_PAUSES_MS if test == "T2" else T5_PAUSES_MS
    reps = args.reps
    w = 0
    t_next = now() + 0.5
    for pause in pauses:
        for rep in range(reps):
            if test == "T2":
                a, b = HDR + pattern(w, 32), pattern(w, 64)[32:]
            else:
                a, b = HDR + pattern(w, 64), HDR + pattern(w + 1, 64)
            busy_wait_until(t_next)
            tag = {"test": test, "pause_ms": pause, "rep": rep, "w": w}
            t_s, t_e = rec.write(tx.ser, a, dict(tag, part=0))
            # pause is measured from when the last byte of part 0 should have left the wire
            busy_wait_until(max(t_e, t_s + wire_time(len(a))) + pause / 1000)
            rec.write(tx.ser, b, dict(tag, part=1))
            w += 2
            t_next = now() + 1.0
    name = args.name or test.lower()
    close_pair(name, tx, rx, rec, tail=3.0)
    analyze_pairs(args, test)


def analyze_pairs(args, test):
    gap = args.gap_ms / 1000
    name = args.name or test.lower()
    writes, reads, _ = load_csv(os.path.join(LOGS, f"{name}_raw.csv"))
    units = windows(writes, key=lambda w: w["tag"]["w"])
    rows = []
    for _, ws, t_a, t_b in units:
        if len(ws) != 2:
            continue
        w0, w1 = ws
        groups = group_reads(in_window(reads["RX"], t_a, t_b), gap)
        tx_echo = b"".join(d for _, d in in_window(reads["TX"], t_a, t_b))
        if test == "T2":
            expected = w0["data"][len(HDR):] + w1["data"]
        else:
            expected = w0["data"][len(HDR):] + w1["data"][len(HDR):]
        got = b"".join(bytes(g["data"]) for g in groups)
        t_we = wire_end(w1)
        rows.append({
            "pause_ms": w0["tag"]["pause_ms"], "rep": w0["tag"]["rep"],
            "actual_gap_ms": ms(w1["t_start"] - wire_end(w0)),
            "groups": [len(g["data"]) for g in groups],
            "rx_total": len(got),
            "match": describe_match(expected, got),
            "group_hex": [bytes(g["data"]).hex() for g in groups],
            "latency_last_ms": ms(groups[-1]["t_last"] - t_we) if groups else None,
            "group_end_ms": [ms(g["t_last"] - t_we) for g in groups],
            "tx_echo": tx_echo.decode("ascii", "backslashreplace"),
        })
    by_pause = {}
    for r in rows:
        by_pause.setdefault(r["pause_ms"], []).append(r)
    lines = [f"{test} pairs (arrival-group gap {args.gap_ms} ms; latency from wire end of 2nd write)"]
    lines.append(f"  {'pause':>5}  n  1grp 2grp other  bytes-rcvd(set)       exact  lat p50 ms  "
                 f"group-size patterns")
    summary = {}
    for p, rs in by_pause.items():
        n1 = sum(len(r["groups"]) == 1 for r in rs)
        n2 = sum(len(r["groups"]) == 2 for r in rs)
        exact = sum(r["match"] == "exact" for r in rs)
        pats = {}
        for r in rs:
            pats[str(r["groups"])] = pats.get(str(r["groups"]), 0) + 1
        lat = [r["latency_last_ms"] for r in rs if r["latency_last_ms"] is not None]
        totals = sorted({r["rx_total"] for r in rs})
        summary[str(p)] = {"n": len(rs), "one_group": n1, "two_groups": n2, "exact": exact,
                           "patterns": pats, "lat_p50_ms": pct(lat, 50), "rx_totals": totals}
        lines.append(f"  {p:5}  {len(rs):2d} {n1:4d} {n2:4d} {len(rs) - n1 - n2:5d}  {str(totals):20s}  "
                     f"{exact:5d}  {str(pct(lat, 50)):>10}  {pats}")
    odd = [r for r in rows if r["match"] != "exact"]
    if odd:
        lines.append("  non-exact pairs:")
        for r in odd[:40]:
            lines.append(f"    pause {r['pause_ms']} rep {r['rep']}: groups {r['groups']} {r['match'][:150]}")
    echoes = [r["tx_echo"] for r in rows if r["tx_echo"]]
    lines.append(f"  TX-port output during test: {echoes[:5] if echoes else 'none'}")
    write_summary(name, {"gap_ms": args.gap_ms, "by_pause": summary, "rows": rows}, "\n".join(lines))


# --------------------------------------------------------------------------- T3 / T4 production pattern
def build_packet(seq, uptime_ms, rnd):
    body = bytes([0xA5, 0x5A, 0x01, seq & 0xFF]) + struct.pack("<I", uptime_ms & 0xFFFFFFFF) \
        + rnd.randbytes(54)
    return body + struct.pack("<H", crc16_ccitt(body))


def run_prod(args, test):
    odd = {1, 11} if test == "T3" else {1, 5, 11, 15}
    slots = sorted(set(range(0, 20, 2)) | odd)
    duration = args.duration or (300 if test == "T3" else 120)
    rnd = random.Random(1234 if test == "T3" else 5678)
    tx, rx, rec = open_pair(args)
    t_base = now() + 0.5
    seq = 0
    n_slots = int(duration / 0.05)
    late = 0
    for k in range(n_slots):
        if k % 20 not in slots:
            continue
        t_slot = t_base + k * 0.05
        busy_wait_until(t_slot)
        if now() - t_slot > 0.005:
            late += 1
        pkt = build_packet(seq, int((t_slot - t_base) * 1000), rnd)
        rec.write(tx.ser, HDR + pkt, {"test": test, "seq": seq, "slot": k})
        seq += 1
    print(f"sent {seq} packets, {late} writes started >5 ms after their slot")
    name = args.name or test.lower()
    close_pair(name, tx, rx, rec, tail=3.0)
    analyze_prod(args, test)


def parse_stream(reads):
    """Byte-stream parser: find A5 5A, take 64 B, check CRC; on failure advance 1 byte."""
    buf = bytearray()
    tbyte = []
    for t, d in reads:
        buf += d
        tbyte += [t] * len(d)
    good, bad_crc, skipped = [], 0, 0
    i = 0
    n = len(buf)
    while i + 64 <= n:
        if buf[i] == 0xA5 and buf[i + 1] == 0x5A:
            frame = bytes(buf[i:i + 64])
            if crc16_ccitt(frame[:62]) == struct.unpack_from("<H", frame, 62)[0]:
                good.append((frame, tbyte[i + 63], i))
                i += 64
                continue
            bad_crc += 1
        skipped += 1
        i += 1
    skipped += n - i  # trailing partial
    return good, bad_crc, skipped, n


def analyze_prod(args, test):
    gap = args.gap_ms / 1000
    name = args.name or test.lower()
    writes, reads, _ = load_csv(os.path.join(LOGS, f"{name}_raw.csv"))
    sent = {w["data"][len(HDR):]: w for w in writes}
    good, bad_crc, skipped, nbytes = parse_stream(reads["RX"])
    seen, dup, unknown, lat = {}, 0, 0, []
    for frame, t_rx, _ in good:
        if frame not in sent:
            unknown += 1
            continue
        if frame in seen:
            dup += 1
            continue
        seen[frame] = t_rx
        lat.append(t_rx - wire_end(sent[frame]))
    lost = [w["tag"]["seq"] for f, w in sent.items() if f not in seen]
    # seq-gap based loss count (u8 seq, wraps)
    seqs = [f[3] for f, _, _ in good]
    seq_gaps = sum(((b - a) & 0xFF) - 1 for a, b in zip(seqs, seqs[1:]) if ((b - a) & 0xFF) != 1)
    groups = group_reads(reads["RX"], gap)
    sizes = {}
    for g in groups:
        sizes[len(g["data"])] = sizes.get(len(g["data"]), 0) + 1
    odd_groups = [(round(g["t_first"], 3), len(g["data"])) for g in groups if len(g["data"]) != 64]
    tx_out = b"".join(d for _, d in reads["TX"])
    # spacing actually achieved between consecutive writes
    starts = [w["t_start"] for w in writes]
    spac = [b - a for a, b in zip(starts, starts[1:])]
    lost_detail = sorted(lost)
    # lost by position in the 1 s slot pattern
    lost_slots = {}
    for w in writes:
        if w["data"][len(HDR):] not in seen:
            s = w["tag"]["slot"] % 20
            lost_slots[s] = lost_slots.get(s, 0) + 1
    duration = writes[-1]["t_start"] - writes[0]["t_start"] if writes else 0
    s = {
        "sent": len(writes), "duration_s": round(duration, 1),
        "received_good_crc": len(seen), "bad_crc_candidates": bad_crc, "lost": len(lost),
        "lost_seq_list": lost_detail[:200], "lost_by_slot": lost_slots,
        "lost_by_seq_gaps": seq_gaps, "duplicates": dup, "good_crc_not_sent": unknown,
        "rx_bytes": nbytes, "bytes_skipped_resync": skipped,
        "arrival_group_sizes": sizes, "groups_not_64": len(odd_groups), "odd_groups": odd_groups[:100],
        "latency_ms": {"p50": ms(pct(lat, 50)), "p95": ms(pct(lat, 95)), "max": ms(max(lat) if lat else None),
                       "min": ms(min(lat) if lat else None)},
        "write_spacing_ms": {"min": ms(min(spac) if spac else None), "p50": ms(pct(spac, 50))},
        "tx_port_output": tx_out.decode("ascii", "backslashreplace"),
        "gap_ms": args.gap_ms,
    }
    lines = [f"{test} production pattern ({len(writes)} packets over {s['duration_s']} s)"]
    for k in ("received_good_crc", "bad_crc_candidates", "lost", "lost_by_seq_gaps", "duplicates",
              "good_crc_not_sent", "rx_bytes", "bytes_skipped_resync", "arrival_group_sizes",
              "groups_not_64", "latency_ms", "write_spacing_ms", "lost_by_slot"):
        lines.append(f"  {k}: {s[k]}")
    if odd_groups:
        lines.append(f"  first odd groups (t, size): {odd_groups[:20]}")
    if lost_detail:
        lines.append(f"  lost seqs (first 50): {lost_detail[:50]}")
    lines.append(f"  TX port output: {s['tx_port_output'][:200]!r}")
    write_summary(name, s, "\n".join(lines))


# --------------------------------------------------------------------------- T7 extra: trickle (idle timer vs fixed window)
def run_t7(args):
    """15 writes of 8 B every 20 ms (first one carries the header). An idle timer longer than
    ~20 ms would merge them into one 120 B packet; a fixed window from the first byte splits them."""
    tx, rx, rec = open_pair(args)
    period = (args.duration or 20) / 1000
    w = 0
    for rep in range(args.reps):
        time.sleep(1.5)
        t0 = now()
        for k in range(15):
            busy_wait_until(t0 + k * period)
            data = pattern(w, 120)[k * 8:(k + 1) * 8]
            rec.write(tx.ser, (HDR if k == 0 else b"") + data, {"test": "T7", "rep": rep, "k": k, "w": w})
        w += 1
    name = args.name or "t7"
    close_pair(name, tx, rx, rec, tail=3.0)
    analyze_t7(args)


def analyze_t7(args):
    gap = args.gap_ms / 1000
    name = args.name or "t7"
    writes, reads, _ = load_csv(os.path.join(LOGS, f"{name}_raw.csv"))
    units = windows(writes, key=lambda w: w["tag"]["w"])
    lines = [f"T7 trickle: 15 x 8 B writes (gap {args.gap_ms} ms grouping)"]
    rows = []
    for _, ws, t_a, t_b in units:
        expected = b"".join(w["data"] for w in ws)[len(HDR):]
        groups = group_reads(in_window(reads["RX"], t_a, t_b), gap)
        got = b"".join(bytes(g["data"]) for g in groups)
        echo = b"".join(d for _, d in in_window(reads["TX"], t_a, t_b))
        # which write index does each received group start/end in?
        spans = []
        for g in groups:
            gd = bytes(g["data"])
            i = expected.find(gd)
            spans.append((i // 8, (i + len(gd) - 1) // 8) if i >= 0 else None)
        r = {"rep": ws[0]["tag"]["rep"], "groups": [len(g["data"]) for g in groups],
             "group_t_ms_from_first_write": [ms(g["t_last"] - ws[0]["t_start"]) for g in groups],
             "write_spans": spans, "rx_total": len(got), "match": describe_match(expected, got),
             "tx_echo": echo.decode("ascii", "backslashreplace")}
        rows.append(r)
        lines.append(f"  rep {r['rep']}: groups {r['groups']} from writes {spans} "
                     f"at {r['group_t_ms_from_first_write']} ms  TX={r['tx_echo']!r}")
    write_summary(name, {"rows": rows}, "\n".join(lines))


# --------------------------------------------------------------------------- T8 extra: what rate works?
T8_CONFIGS = [(1, 100), (1, 120), (1, 150), (1, 200), (1, 250), (3, 250), (3, 300), (3, 400)]


def run_t8(args):
    """Production-format 64 B packets: k packets per UART write (one header), one write every P ms.
    Each config runs --duration seconds (default 20)."""
    secs = args.duration or 20
    rnd = random.Random(42)
    tx, rx, rec = open_pair(args)
    seq = 0
    t_base = now()
    for k, period in T8_CONFIGS:
        time.sleep(1.5)  # let the previous config drain
        t0 = now()
        n_writes = int(secs * 1000 / period)
        for i in range(n_writes):
            busy_wait_until(t0 + i * period / 1000)
            pkts = [build_packet(seq + j, int((now() - t_base) * 1000), rnd) for j in range(k)]
            rec.write(tx.ser, HDR + b"".join(pkts),
                      {"test": "T8", "k": k, "period_ms": period, "seqs": list(range(seq, seq + k))})
            seq += k
    name = args.name or "t8"
    close_pair(name, tx, rx, rec, tail=3.0)
    analyze_t8(args)


def analyze_t8(args):
    name = args.name or "t8"
    writes, reads, _ = load_csv(os.path.join(LOGS, f"{name}_raw.csv"))
    good, bad_crc, skipped, nbytes = parse_stream(reads["RX"])
    rx_t = {}
    for frame, t, _ in good:
        rx_t.setdefault(frame, t)
    cfgs = {}
    for w in writes:
        key = (w["tag"]["k"], w["tag"]["period_ms"])
        c = cfgs.setdefault(key, {"sent": 0, "ok": 0, "lat": [], "t_first": w["t_start"], "t_last": w["t_start"]})
        c["t_last"] = w["t_start"]
        body = w["data"][len(HDR):]
        for j in range(0, len(body), 64):
            c["sent"] += 1
            f = body[j:j + 64]
            if f in rx_t:
                c["ok"] += 1
                c["lat"].append(rx_t[f] - wire_end(w))
    errs = {}
    for t, d in reads["TX"]:
        n = d.count(b"channel Error")
        for key, c in cfgs.items():
            if c["t_first"] <= t < c["t_last"] + 1.0:
                errs[key] = errs.get(key, 0) + n
    lines = ["T8 what rate works (64 B production packets; k per write, one write every P ms)",
             "    k  P(ms)  pkts/s  sent   ok   loss%   lat p50/p95/max ms   TX 'channel Error' lines"]
    out = []
    for (k, p), c in cfgs.items():
        loss = 100 * (c["sent"] - c["ok"]) / c["sent"]
        lat = c["lat"]
        out.append({"k": k, "period_ms": p, "pkts_per_s": round(k * 1000 / p, 2), "sent": c["sent"], "ok": c["ok"],
                    "loss_pct": round(loss, 1), "lat_p50_ms": ms(pct(lat, 50)), "lat_p95_ms": ms(pct(lat, 95)),
                    "lat_max_ms": ms(max(lat) if lat else None), "channel_errors": errs.get((k, p), 0)})
        o = out[-1]
        lines.append(f"  {k:3d} {p:6d} {o['pkts_per_s']:7.2f} {c['sent']:5d} {c['ok']:4d} {loss:7.1f}   "
                     f"{o['lat_p50_ms']}/{o['lat_p95_ms']}/{o['lat_max_ms']}   {o['channel_errors']}")
    write_summary(name, {"configs": out, "bad_crc_candidates": bad_crc, "bytes_skipped": skipped}, "\n".join(lines))


# --------------------------------------------------------------------------- T6 extra: '+++' inside data
def run_t6(args):
    """Extra: does '+++' inside packet data flip the TX module into AT mode?"""
    tx, rx, rec = open_pair(args)
    cases = [("plus3_mid", 30, b"+++"), ("plus3crlf_mid", 30, b"+++\r\n"), ("plus3_end", 61, b"+++")]
    w = 0
    results = []
    for name, pos, ins in cases:
        for rep in range(3):
            base = bytearray(pattern(1000 + w, 64))
            base[pos:pos + len(ins)] = ins
            payload = bytes(base[:64])
            time.sleep(1.0)
            rec.write(tx.ser, HDR + payload, {"test": "T6", "case": name, "rep": rep, "w": w})
            w += 1
    time.sleep(2.0)
    rec.stop()
    rec.save(os.path.join(LOGS, "t6_raw.csv"))
    # make sure TX is back in data mode
    tx.ser.timeout = 0.05
    try:
        if tx.toggle_at():  # 'Entry AT' -> it was in data mode (good); go back
            tx.toggle_at()
            state = "TX was still in data mode after the test (+++ -> Entry AT; toggled back)"
        else:               # 'Exit AT' -> the data had put it in AT mode
            state = "TX had been switched INTO AT mode by the data (+++ -> Exit AT); now back in data mode"
    except RuntimeError as e:
        state = f"no reply to +++ ({e})"
    tx.close()
    rx.close()
    print("post-test TX state:", state)
    analyze_t6(args, state)


def analyze_t6(args, state=None):
    gap = args.gap_ms / 1000
    writes, reads, _ = load_csv(os.path.join(LOGS, "t6_raw.csv"))
    units = windows(writes, key=lambda w: w["tag"]["w"])
    lines = ["T6 '+++' inside packet data"]
    rows = []
    for _, ws, t_a, t_b in units:
        wr = ws[0]
        groups = group_reads(in_window(reads["RX"], t_a, t_b), gap)
        got = b"".join(bytes(g["data"]) for g in groups)
        echo = b"".join(d for _, d in in_window(reads["TX"], t_a, t_b))
        r = {"case": wr["tag"]["case"], "rep": wr["tag"]["rep"], "groups": [len(g["data"]) for g in groups],
             "match": describe_match(wr["data"][len(HDR):], got),
             "tx_echo": echo.decode("ascii", "backslashreplace")}
        rows.append(r)
        lines.append(f"  {r['case']:14s} rep {r['rep']}: groups {r['groups']} {r['match'][:80]}  TX={r['tx_echo']!r}")
    lines.append(f"  post-test TX state: {state}")
    write_summary("t6", {"rows": rows, "post_state": state}, "\n".join(lines))


# --------------------------------------------------------------------------- CLI
def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("action", choices=["run", "analyze"])
    p.add_argument("test", choices=["t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7", "t8"])
    p.add_argument("--tx", default="COM8")
    p.add_argument("--rx", default="COM9")
    p.add_argument("--gap-ms", type=float, default=5.0, help="arrival-group gap threshold (default 5 ms)")
    p.add_argument("--reps", type=int, default=None)
    p.add_argument("--duration", type=float, default=None)
    p.add_argument("--name", default=None, help="log name (t0: default t0; t1/t2: suffix, e.g. t1_edge)")
    p.add_argument("--sizes", default=None, help="t1: comma-separated data sizes (overrides default list)")
    p.add_argument("--pauses", default=None, help="t2/t5: comma-separated pauses in ms (overrides default)")
    p.add_argument("--configs", default=None, help="t8: e.g. 3x214,2x143 (packets per write x period ms)")
    args = p.parse_args()
    global T1_SIZES, T2_PAUSES_MS, T5_PAUSES_MS, T8_CONFIGS
    if args.configs:
        T8_CONFIGS = [tuple(int(v) for v in c.split("x")) for c in args.configs.split(",")]
    if args.sizes:
        T1_SIZES = [int(x) for x in args.sizes.split(",")]
    if args.pauses:
        ps = [float(x) for x in args.pauses.split(",")]
        T2_PAUSES_MS = T5_PAUSES_MS = ps
    os.makedirs(LOGS, exist_ok=True)
    t = args.test
    if args.reps is None:
        args.reps = {"t1": 5, "t2": 10, "t5": 20, "t7": 5}.get(t, 1)
    if t == "t0" and args.duration is None:
        args.duration = 10
    if args.action == "run":
        {"t0": run_t0, "t1": run_t1, "t2": lambda a: run_pairs(a, "T2"), "t3": lambda a: run_prod(a, "T3"),
         "t4": lambda a: run_prod(a, "T4"), "t5": lambda a: run_pairs(a, "T5"), "t6": run_t6, "t7": run_t7, "t8": run_t8}[t](args)
    else:
        {"t0": lambda a: analyze_t0(a.name or "t0", a), "t1": analyze_t1, "t2": lambda a: analyze_pairs(a, "T2"),
         "t3": lambda a: analyze_prod(a, "T3"), "t4": lambda a: analyze_prod(a, "T4"),
         "t5": lambda a: analyze_pairs(a, "T5"), "t6": analyze_t6, "t7": analyze_t7, "t8": analyze_t8}[t](args)


if __name__ == "__main__":
    main()
