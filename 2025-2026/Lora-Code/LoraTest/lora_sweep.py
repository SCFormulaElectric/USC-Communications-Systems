#!/usr/bin/env python3
"""
lora_sweep.py - find the fastest LoRa send rate with zero packet loss.

Bench setup (two COM ports on one laptop):

  laptop --USB--> USB-TTL --UART 9600--> LoRa TX module  ~~~RF~~~>  LoRa RX board --USB--> laptop
          (--tx COMx)                                                            (--rx COMy)

What it does:
  1. Configures both LoRa modules with the same AT sequence as LoRa_board/Core/Src/main.c
     (skip with --no-config if they're already set up).
  2. Link check: sends a few packets and makes sure at least one arrives.
  3. Sweep: for each packet size, sends N numbered packets at each rate, then counts
     what arrived and classifies every received line:
        OK            valid packet, first time seen
        CORRUPT       looks like a test packet but checksum/length/type is wrong
        DUPLICATE     valid, but this seq already arrived
        OUT_OF_ORDER  valid, but arrived after a later seq
        STRAY         valid, but belongs to an earlier step (arrived late)
        UNKNOWN       anything else (AT replies, boot text, noise)
        LOST          sent but never arrived (computed at the end of the step)
     LOST counts every packet that never arrived intact, so corrupted packets count as lost.
     A lossy step is re-run (--retries) before it counts: random RF noise drops the odd
     packet at any rate, but loss from sending too fast repeats. Once loss is confirmed,
     the script bisects between the last clean rate and the first lossy one.
  4. Writes runs/<timestamp>/results.csv, rx_raw.log, and loss_vs_rate.png.

Test payload (printable ASCII, so '\\n' framing and printf("%s") can never break it):

     T  A  002F  bcdefg...  9C
     |  |  |     |          '- checksum: sum of all previous bytes & 0xFF, 2 hex chars
     |  |  |     '- padding up to --sizes (deterministic letters, covered by checksum)
     |  |  '- sequence number, 4 hex chars
     |  '- message type A/B/C (stand-ins for different CAN messages)
     '- marker

On air (fixed-point mode, same as the firmware):  [0x0A 0x01 0x03] + payload + "\\r\\n"

Usage:
  pip install pyserial matplotlib
  python lora_sweep.py --tx COM5 --rx COM7
  python lora_sweep.py --tx COM5 --rx COM7 --sizes 8,16,32 --rates 0.5,1,2,3,5 --n 50
  python lora_sweep.py --mock          # no hardware: fake radio, for checking the script
"""

import argparse
import csv
import math
import os
import queue
import random
import re
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field

# ----------------------------------------------------------------------------
# AT configuration (mirrors at_steps[] in LoRa_board/Core/Src/main.c)
# ----------------------------------------------------------------------------

AT_REPLY_TIMEOUT_S = 0.3
AT_MAX_RETRIES = 3


def at_sequence(mac, channel, sf, level, power):
    """(command bytes, settle seconds after the reply) - same order as the firmware."""
    return [
        (b"+++\r\n", 0.02),
        (b"AT+MODE1\r\n", 0.02),                         # fixed-point transmission
        (f"AT+LEVEL{level}\r\n".encode(), 0.02),         # air data rate level
        (f"AT+CHANNEL{channel:02X}\r\n".encode(), 0.02),
        (f"AT+MAC{mac}\r\n".encode(), 0.02),
        (f"AT+POWE{power}\r\n".encode(), 0.02),
        (f"AT+SF{sf}\r\n".encode(), 0.02),
        (b"AT+RESET\r\n", 1.0),                          # module reboots; wait longer
        (b"+++\r\n", 0.02),                              # firmware ends with two "+++";
        (b"+++\r\n", 0.02),                              # kept so behaviour matches
    ]


def read_reply(ser, timeout):
    """Collect bytes until a full line arrives or the timeout expires."""
    deadline = time.monotonic() + timeout
    buf = b""
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            if b"\n" in buf:
                time.sleep(0.02)                 # let the rest of the reply land
                buf += ser.read(ser.in_waiting)
                return buf
    return buf


def at_configure(ser, steps, name, verbose):
    print(f"[config] {name}: sending {len(steps)} AT commands")
    for cmd, settle in steps:
        for attempt in range(AT_MAX_RETRIES + 1):
            ser.reset_input_buffer()             # throw away stale replies
            ser.write(cmd)
            reply = read_reply(ser, AT_REPLY_TIMEOUT_S)
            if reply:
                if verbose:
                    print(f"  {cmd.strip().decode():<16} -> {reply.strip()!r}")
                break
        else:
            # for/else: runs only if the loop never hit `break` (no reply on any attempt)
            raise RuntimeError(
                f"{name}: no reply to {cmd.strip().decode()} after {AT_MAX_RETRIES + 1} tries. "
                "Check wiring (TX<->RX crossed, 3.3 V logic, common GND) and baud rate.")
        time.sleep(settle)
    ser.reset_input_buffer()


# ----------------------------------------------------------------------------
# Test packets
# ----------------------------------------------------------------------------

TYPES = "ABC"
MIN_SIZE = 8
PKT_RE = re.compile(rb"T([A-Z])([0-9A-F]{4})([a-z]*)([0-9A-F]{2})")


def build_payload(seq, size, mtype):
    pad = bytes(97 + (seq + i) % 26 for i in range(size - MIN_SIZE))   # 97 = 'a'
    body = b"T" + mtype.encode() + f"{seq:04X}".encode() + pad
    return body + f"{sum(body) & 0xFF:02X}".encode()



def classify_line(line):
    """Return a list of (kind, info) for one received line.
    kind is 'PKT' (info = (seq, type, raw, checksum_ok)), 'FRAGMENT' or 'UNKNOWN'."""
    matches = list(PKT_RE.finditer(line))
    if not matches:
        # A 'T' with garbage around it is most likely a mangled test packet.
        return [("FRAGMENT" if b"T" in line else "UNKNOWN", line)]
    out = []
    for m in matches:
        raw = m.group(0)
        chk_ok = int(m.group(4), 16) == (sum(raw[:-2]) & 0xFF)
        out.append(("PKT", (int(m.group(2), 16), m.group(1).decode(), raw, chk_ok)))
    return out


# ----------------------------------------------------------------------------
# Receiver thread
# ----------------------------------------------------------------------------

class Receiver(threading.Thread):
    """Reads the RX COM port nonstop so slow analysis never causes dropped bytes.
    Every line is timestamped on arrival with the same clock the sender uses."""

    def __init__(self, ser, raw_log=None):
        super().__init__(daemon=True)
        self.ser = ser
        self.lines = []                  # (perf_counter time, bytes)
        self.lock = threading.Lock()
        self.stop_evt = threading.Event()
        self.raw_log = raw_log

    def run(self):
        buf = b""
        while not self.stop_evt.is_set():
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if not chunk:
                continue
            t = time.perf_counter()
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.rstrip(b"\r")
                if not line:
                    continue
                with self.lock:
                    self.lines.append((t, line))
                if self.raw_log:
                    self.raw_log.write(f"{t:.6f} {line!r}\n")

    def mark(self):
        with self.lock:
            return len(self.lines)

    def since(self, idx):
        with self.lock:
            return self.lines[idx:]

    def stop(self):
        self.stop_evt.set()
        self.join(timeout=1.0)


# ----------------------------------------------------------------------------
# One step: N packets at one size and one rate
# ----------------------------------------------------------------------------

@dataclass
class StepResult:
    size: int
    target_rate: float
    actual_rate: float
    sent: int
    sf: int = 0
    ok: int = 0
    lost: int = 0
    corrupt: int = 0
    duplicate: int = 0
    out_of_order: int = 0
    stray: int = 0
    unknown: int = 0
    lat_p50_ms: float = float("nan")
    lat_p95_ms: float = float("nan")
    lat_max_ms: float = float("nan")
    lost_by_type: dict = field(default_factory=dict)

    @property
    def loss_pct(self):
        return 100.0 * self.lost / self.sent if self.sent else 0.0


def pace_until(target):
    """Sleep until `target` (perf_counter seconds) without drifting."""
    while True:
        d = target - time.perf_counter()
        if d <= 0:
            return
        time.sleep(min(d, 0.002) if d < 0.01 else d - 0.005)


def run_step(tx, rx, header, size, rate, n, seq0, drain_s, sf=0):
    sent = {}                                    # seq -> (send time, type)
    start_idx = rx.mark()
    t0 = time.perf_counter()
    for i in range(n):
        pace_until(t0 + i / rate)                # deadline-based pacing
        seq = (seq0 + i) & 0xFFFF
        mtype = TYPES[i % len(TYPES)]
        frame = header + build_payload(seq, size, mtype) + b"\r\n"
        t = time.perf_counter()
        tx.write(frame)
        sent[seq] = (t, mtype)

    times = sorted(t for t, _ in sent.values())
    actual = (n - 1) / (times[-1] - times[0]) if n > 1 and times[-1] > times[0] else rate
    time.sleep(drain_s)                          # let in-flight packets arrive

    r = StepResult(size=size, target_rate=rate, actual_rate=actual, sent=n, sf=sf)
    got = {}                                     # seq -> latency (s)
    highest_idx = -1
    for t, line in rx.since(start_idx):
        for kind, info in classify_line(line):
            if kind == "UNKNOWN":
                r.unknown += 1
                continue
            if kind == "FRAGMENT":
                r.corrupt += 1
                continue
            seq, mtype, raw, chk_ok = info
            if seq not in sent:
                if chk_ok:
                    r.stray += 1                 # late packet from an earlier step
                else:
                    r.corrupt += 1
                continue
            if not chk_ok or len(raw) != size or sent[seq][1] != mtype:
                r.corrupt += 1
                continue
            if seq in got:
                r.duplicate += 1
                continue
            idx = (seq - seq0) & 0xFFFF
            if idx < highest_idx:
                r.out_of_order += 1
            highest_idx = max(highest_idx, idx)
            got[seq] = t - sent[seq][0]

    r.ok = len(got)
    r.lost = n - r.ok
    for seq, (_, mtype) in sent.items():
        if seq not in got:
            r.lost_by_type[mtype] = r.lost_by_type.get(mtype, 0) + 1
    if got:
        lat = sorted(v * 1000 for v in got.values())
        r.lat_p50_ms = statistics.median(lat)
        r.lat_p95_ms = lat[min(len(lat) - 1, math.ceil(0.95 * len(lat)) - 1)]
        r.lat_max_ms = lat[-1]
    return r


# ----------------------------------------------------------------------------
# Sweep
# ----------------------------------------------------------------------------

class SeqCounter:
    def __init__(self):
        self.next = 0

    def take(self, n):
        s = self.next
        self.next = (self.next + n) & 0xFFFF
        return s


def print_step(r):
    lost_types = ",".join(f"{k}:{v}" for k, v in sorted(r.lost_by_type.items())) or "-"
    print(f"  SF{r.sf:<2} {r.size:>3} B @ {r.target_rate:6.2f} Hz (actual {r.actual_rate:6.2f})  "
          f"ok {r.ok:>4}/{r.sent:<4} lost {r.lost:>4} ({r.loss_pct:5.1f}%)  "
          f"corrupt {r.corrupt:>3} dup {r.duplicate:>2} ooo {r.out_of_order:>2} "
          f"stray {r.stray:>2}  p50 {r.lat_p50_ms:7.1f} ms  lost by type {lost_types}")


def sweep_size(tx, rx, header, sf, size, rates, drain, args, seqs, results):
    """Return the highest rate where every step at or below it was confirmed clean."""
    frame_bytes = len(header) + size + 2
    uart_max = args.baud / (10 * frame_bytes)     # 8N1 = 10 bits per byte

    def step(rate):
        r = run_step(tx, rx, header, size, rate, args.n, seqs.take(args.n), drain, sf)
        results.append(r)
        print_step(r)
        return r.lost == 0

    def confirmed_clean(rate):
        """Random RF noise can drop a packet at any rate. Only call a rate lossy if the
        loss repeats: re-run a lossy step up to --retries times and accept it if any rerun
        is clean. Rate-caused loss (module buffer overflowing) repeats every time."""
        if step(rate):
            return True
        for _ in range(args.retries):
            print(f"  retry {rate:.2f} Hz (could be random noise)")
            if step(rate):
                return True
        return False

    print(f"\n== SF{sf} size {size} B  (frame {frame_bytes} B on the UART, "
          f"UART limit ~{uart_max:.0f} pkt/s)")
    last_clean, first_lossy = None, None
    for rate in rates:
        if rate > 0.9 * uart_max:
            print(f"  {rate} Hz is past the UART limit; stopping this size")
            break
        if confirmed_clean(rate):
            last_clean = rate
        else:
            first_lossy = rate
            break

    if first_lossy is None:
        if last_clean is not None:
            print(f"  no loss up to {last_clean} Hz - add higher --rates to find the limit")
        return last_clean
    if last_clean is None:
        print("  lossy even at the lowest rate - lower --rates or check the link")
        return None

    # One step past the knee, so the plot shows how fast loss climbs.
    above = [r for r in rates if r > first_lossy and r <= 0.9 * uart_max]
    if above:
        step(above[0])

    # Narrow the gap between the last clean rate and the first lossy one.
    lo, hi = last_clean, first_lossy
    for _ in range(args.refine):
        if hi / lo < 1.1:
            break
        mid = math.sqrt(lo * hi)                   # geometric midpoint: rates are spaced ~log
        print(f"  refine: {lo:.2f} .. {hi:.2f} Hz -> trying {mid:.2f}")
        if confirmed_clean(mid):
            lo = mid
        else:
            hi = mid
    return lo


def link_check(tx, rx, header, sf, args, seqs):
    """Send 3 slow packets. Returns the step result, or None if nothing arrived."""
    print(f"\n[link check] SF{sf}: sending 3 packets at 0.5 Hz ...")
    r = run_step(tx, rx, header, MIN_SIZE, 0.5, 3, seqs.take(3), args.drain, sf)
    print_step(r)
    if r.ok == 0:
        sample = [line for _, line in rx.since(0)[-5:]]
        print("\nNothing arrived. Last raw lines from RX:", sample or "(none)")
        print("Check: both modules configured (same channel/SF/level), RX MAC matches --dest,\n"
              "antennas attached, and the right COM ports. Raw RX output is in rx_raw.log.")
        return None
    return r


# ----------------------------------------------------------------------------
# Output
# ----------------------------------------------------------------------------

CSV_FIELDS = ["sf", "size", "target_rate", "actual_rate", "sent", "ok", "lost", "loss_pct",
              "corrupt", "duplicate", "out_of_order", "stray", "unknown",
              "lat_p50_ms", "lat_p95_ms", "lat_max_ms", "lost_by_type"]


def write_csv(path, results):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_FIELDS)
        for r in results:
            w.writerow([r.sf, r.size, f"{r.target_rate:.3f}", f"{r.actual_rate:.3f}", r.sent, r.ok,
                        r.lost, f"{r.loss_pct:.2f}", r.corrupt, r.duplicate, r.out_of_order,
                        r.stray, r.unknown, f"{r.lat_p50_ms:.1f}", f"{r.lat_p95_ms:.1f}",
                        f"{r.lat_max_ms:.1f}",
                        " ".join(f"{k}:{v}" for k, v in sorted(r.lost_by_type.items()))])


def plot(path, results):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib not installed - skipping plot)")
        return
    fig, ax = plt.subplots(figsize=(8, 5))
    from matplotlib.ticker import ScalarFormatter
    for size in sorted({r.size for r in results}):
        by_rate = {}                               # retries at one rate -> pooled loss
        for r in results:
            if r.size == size:
                s, l = by_rate.get(r.target_rate, (0, 0))
                by_rate[r.target_rate] = (s + r.sent, l + r.lost)
        pts = sorted((rate, 100.0 * l / s) for rate, (s, l) in by_rate.items())
        ax.plot([p[0] for p in pts], [p[1] for p in pts], "o-", label=f"{size} B")
    ax.set_xscale("log")
    ax.xaxis.set_major_formatter(ScalarFormatter())
    ax.xaxis.set_minor_formatter(ScalarFormatter())
    ax.set_xlabel("send rate (packets/s, log scale)")
    ax.set_ylabel("packet loss (%)")
    ax.set_title("LoRa loss vs send rate")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(title="payload")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"plot  -> {path}")


def print_summary(best, n, results):
    bound = 300.0 / n                              # rule of three: 95% upper bound on loss
    print("\n================ SUMMARY ================")
    print(f"{'size':>6} {'max lossless rate':>18} {'throughput':>12}")
    for size, rate in best.items():
        if rate is None:
            print(f"{size:>4} B {'lossy at every rate':>18}")
        else:
            print(f"{size:>4} B {rate:>14.2f} pkt/s {rate * size:>8.1f} B/s")
    print(f"\n'Lossless' = 0 lost out of {n} at that step, i.e. true loss is below "
          f"~{bound:.1f}% (95% confidence). Raise --n for a tighter bound.")
    # Noise floor: loss in every step at or below each size's limit (includes retried steps).
    sent = sum(r.sent for r in results if best.get(r.size) and r.target_rate <= best[r.size])
    lost = sum(r.lost for r in results if best.get(r.size) and r.target_rate <= best[r.size])
    if sent:
        print(f"Random loss below the limit: {lost}/{sent} = {100 * lost / sent:.2f}% "
              "(the link's noise floor; retries absorbed this)")
    valid = {s: r for s, r in best.items() if r}
    if valid:
        s = max(valid, key=lambda k: valid[k] * k)
        print(f"Best payload throughput: {s} B packets at {valid[s]:.2f} pkt/s "
              f"= {valid[s] * s:.1f} B/s")


# ----------------------------------------------------------------------------
# Mock hardware (--mock): fake radio with airtime, a small module buffer and noise
# ----------------------------------------------------------------------------

class MockRadio:
    """Stands in for TX module + RF + RX module. Numbers are made up; this only exists
    to check the script's logic without hardware."""

    def __init__(self, header_len, overhead_s=0.05, per_byte_s=0.002, buffer_pkts=2,
                 p_corrupt=0.01, p_drop=0.0):
        self.header_len = header_len
        self.overhead_s, self.per_byte_s = overhead_s, per_byte_s
        self.p_corrupt, self.p_drop = p_corrupt, p_drop
        self.tx_q = queue.Queue(maxsize=buffer_pkts)
        self.rx_buf = bytearray()
        self.lock = threading.Lock()
        threading.Thread(target=self._air, daemon=True).start()

    def send(self, frame):
        try:
            self.tx_q.put_nowait(frame)          # module buffer full -> packet silently dropped
        except queue.Full:
            pass

    def _air(self):
        while True:
            frame = self.tx_q.get()
            payload = frame[self.header_len:]    # fixed mode: RX module strips the header
            time.sleep(self.overhead_s + self.per_byte_s * len(payload))
            if random.random() < self.p_drop:
                continue
            if random.random() < self.p_corrupt:
                b = bytearray(payload)
                b[random.randrange(len(b) - 2)] ^= 0x04
                payload = bytes(b)
            with self.lock:
                self.rx_buf += payload


class MockTxPort:
    def __init__(self, radio):
        self.radio = radio
        self.in_waiting = 0

    def write(self, b):
        self.radio.send(bytes(b))

    def read(self, n=1):
        return b""

    def reset_input_buffer(self):
        pass

    def close(self):
        pass


class MockRxPort:
    def __init__(self, radio):
        self.radio = radio

    @property
    def in_waiting(self):
        with self.radio.lock:
            return len(self.radio.rx_buf)

    def read(self, n=1):
        with self.radio.lock:
            out = bytes(self.radio.rx_buf[:n])
            del self.radio.rx_buf[:n]
        if not out:
            time.sleep(0.01)                     # behave like a serial read timeout
        return out

    def reset_input_buffer(self):
        with self.radio.lock:
            self.radio.rx_buf.clear()

    def close(self):
        pass


# ----------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description="Sweep LoRa send rate to find the max lossless rate.")
    p.add_argument("--tx", help="COM port of the USB-TTL adapter on the TX module (e.g. COM5)")
    p.add_argument("--rx", help="COM port of the RX board (e.g. COM7)")
    p.add_argument("--baud", type=int, default=9600, help="module UART baud (firmware uses 9600)")
    p.add_argument("--rx-baud", type=int, help="RX board baud if different from --baud")
    p.add_argument("--sizes", default="8,16,32", help="payload sizes in bytes (min 8)")
    p.add_argument("--rates", default="0.5,1,1.5,2,3,4,6,8,12,16,24",
                   help="send rates to try, packets/s, ascending")
    p.add_argument("--n", type=int, default=100, help="packets per step")
    p.add_argument("--drain", type=float, default=3.0, help="seconds to wait after each step")
    p.add_argument("--refine", type=int, default=3, help="bisection steps after the first loss")
    p.add_argument("--retries", type=int, default=1,
                   help="re-runs of a lossy step before calling it lossy (separates noise from overload)")
    p.add_argument("--dest", default="0A01", help="RX module address (header bytes 1-2)")
    p.add_argument("--channel", type=int, default=3)
    p.add_argument("--tx-mac", default="0a,02")
    p.add_argument("--rx-mac", default="0a,01")
    p.add_argument("--sf", type=int, default=12, help="spreading factor")
    p.add_argument("--level", type=int, default=7, help="air data rate level")
    p.add_argument("--power", type=int, default=22)
    p.add_argument("--no-config", action="store_true", help="skip AT config (modules already set)")
    p.add_argument("--verbose", action="store_true", help="print AT replies")
    p.add_argument("--mock", action="store_true", help="simulated radio, no hardware")
    a = p.parse_args()
    a.sizes = [int(s) for s in a.sizes.split(",")]
    a.rates = sorted(float(r) for r in a.rates.split(","))
    if min(a.sizes) < MIN_SIZE:
        p.error(f"sizes must be >= {MIN_SIZE}")
    if not a.mock and not (a.tx and a.rx):
        p.error("--tx and --rx are required (or use --mock)")
    return a


def main():
    args = parse_args()
    dest = int(args.dest, 16)
    header = bytes([dest >> 8, dest & 0xFF, args.channel])

    run_dir = os.path.join("runs", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    raw_log = open(os.path.join(run_dir, "rx_raw.log"), "w")

    if args.mock:
        radio = MockRadio(len(header))
        tx, rx_port = MockTxPort(radio), MockRxPort(radio)
        print("[mock] simulated radio - numbers are fake")
    else:
        import serial                            # pyserial
        tx = serial.Serial(args.tx, args.baud, timeout=0.05)
        rx_port = serial.Serial(args.rx, args.rx_baud or args.baud, timeout=0.05)
        if not args.no_config:
            at_configure(tx, at_sequence(args.tx_mac, args.channel, args.sf, args.level, args.power),
                         f"TX ({args.tx})", args.verbose)
            at_configure(rx_port, at_sequence(args.rx_mac, args.channel, args.sf, args.level, args.power),
                         f"RX ({args.rx})", args.verbose)

    est = sum(args.n / r + args.drain for r in args.rates) * len(args.sizes) / 60
    print(f"Run folder: {run_dir}   (worst case ~{est:.0f} min if no size ever loses packets)")

    rx = Receiver(rx_port, raw_log)
    rx.start()
    seqs = SeqCounter()
    results, best = [], {}
    try:
        if not link_check(tx, rx, header, args.sf, args, seqs):
            sys.exit(1)
        for size in args.sizes:
            best[size] = sweep_size(tx, rx, header, args.sf, size, args.rates, args.drain,
                                    args, seqs, results)
    except KeyboardInterrupt:
        print("\nInterrupted - saving what we have.")
    finally:
        rx.stop()
        raw_log.close()
        tx.close()
        rx_port.close()
        if results:
            write_csv(os.path.join(run_dir, "results.csv"), results)
            print(f"\ncsv   -> {os.path.join(run_dir, 'results.csv')}")
            plot(os.path.join(run_dir, "loss_vs_rate.png"), results)
        if best:
            print_summary(best, args.n, results)


if __name__ == "__main__":
    main()
