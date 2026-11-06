"""Shared helpers for the DX-LR02 UART->radio framing tests.

- LoRaModule: AT-mode control (+++ toggles AT/data mode, AT+HELP dumps config).
- Recorder: reader threads on both ports; logs every read as (perf_counter, port, bytes),
  plus every TX write (start/return time, bytes, tag) into one raw CSV.
- group_reads: merge reads into "arrival groups" using a gap threshold.
"""

import binascii
import csv
import json
import threading
import time

import serial

now = time.perf_counter

BAUD = 115200
BAUD_SCAN = [115200, 9600, 57600, 38400, 19200, 4800, 2400, 1200]


def busy_wait_until(t):
    """Precise wait: sleep coarsely while far away, then spin on perf_counter."""
    while True:
        rem = t - now()
        if rem <= 0:
            return
        if rem > 0.025:
            time.sleep(rem - 0.02)


def wire_time(n, baud=BAUD):
    return n * 10 / baud  # 8N1 = 10 bits per byte


class LoRaModule:
    def __init__(self, name, port, baud=BAUD):
        self.name = name
        self.port = port
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
        try:
            self.ser.set_buffer_size(rx_size=65536, tx_size=65536)
        except Exception:
            pass

    def close(self):
        self.ser.close()

    def _read_response(self, timeout, quiet=0.3):
        buf = bytearray()
        deadline = now() + timeout
        last = None
        while now() < deadline:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                buf += chunk
                last = now()
            elif last is not None and now() - last >= quiet:
                break
        return bytes(buf)

    def send(self, line, timeout=1.5):
        self.ser.reset_input_buffer()
        self.ser.write(line.encode("ascii") + b"\r\n")
        return self._read_response(timeout).decode("ascii", errors="backslashreplace")

    def toggle_at(self):
        reply = self.send("+++").lower()
        if "entry" in reply:
            return True
        if "exit" in reply:
            return False
        raise RuntimeError(f"{self.name}: no Entry/Exit reply to +++ (got {reply.strip()!r})")

    def set_at(self, want):
        for _ in range(2):
            if self.toggle_at() == want:
                return
        raise RuntimeError(f"{self.name}: could not {'enter' if want else 'leave'} AT mode")

    def find_baud(self):
        for baud in BAUD_SCAN:
            self.ser.baudrate = baud
            try:
                self.toggle_at()
            except RuntimeError:
                continue
            self.toggle_at()  # put it back in the mode it was found in
            return baud
        return None


class PortLogger(threading.Thread):
    """Logs every read from a port as (t, label, bytes). Survives the port disappearing."""

    def __init__(self, ser, label, rows, lock, reopen=False):
        super().__init__(daemon=True)
        self.ser = ser
        self.label = label
        self.rows = rows
        self.lock = lock
        self.reopen = reopen
        self._halt = threading.Event()

    def _log(self, data, label=None):
        with self.lock:
            self.rows.append((now(), label or self.label, data, None, ""))

    def run(self):
        self.ser.timeout = 0.001
        while not self._halt.is_set():
            try:
                n = self.ser.in_waiting
                data = self.ser.read(n if n else 1)
                if data:
                    t = now()
                    extra = self.ser.in_waiting
                    if extra:
                        data += self.ser.read(extra)
                    with self.lock:
                        self.rows.append((t, self.label, data, None, ""))
            except (serial.SerialException, OSError) as e:
                if not self.reopen:
                    raise
                self._log(f"PORT ERROR: {e}".encode(), self.label + "_event")
                try:
                    self.ser.close()
                except Exception:
                    pass
                while not self._halt.is_set():
                    time.sleep(0.2)
                    try:
                        self.ser.open()
                        self.ser.timeout = 0.001
                        self._log(b"PORT REOPENED", self.label + "_event")
                        break
                    except Exception:
                        continue

    def stop(self):
        self._halt.set()
        self.join()


class Recorder:
    def __init__(self, ports, reopen=False):
        self.rows = []
        self.lock = threading.Lock()
        self.t0 = now()
        self.loggers = [PortLogger(ser, label, self.rows, self.lock, reopen) for label, ser in ports]

    def start(self):
        for lg in self.loggers:
            lg.start()

    def stop(self):
        for lg in self.loggers:
            lg.stop()

    def write(self, ser, data, tag):
        """One UART write. Records start time, return time and a JSON tag."""
        t_start = now()
        ser.write(data)
        t_end = now()
        with self.lock:
            self.rows.append((t_end, "TX_write", bytes(data), t_start, json.dumps(tag)))
        return t_start, t_end

    def save(self, path):
        t0 = self.t0
        with self.lock:
            rows = sorted(self.rows, key=lambda r: r[0])
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "port", "nbytes", "hex", "t_write_start_s", "tag"])
            for t, port, data, t_start, tag in rows:
                w.writerow([f"{t - t0:.6f}", port, len(data), data.hex(),
                            "" if t_start is None else f"{t_start - t0:.6f}", tag])


def load_csv(path):
    """Returns (writes, reads, events). writes: dicts with t_start, t_end, data, tag.
    reads: {"TX": [(t, bytes)], "RX": [...]}. Times are relative to recorder start."""
    writes, reads, events = [], {"TX": [], "RX": []}, []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            data = bytes.fromhex(row["hex"])
            t = float(row["t_s"])
            if row["port"] == "TX_write":
                writes.append({"t_start": float(row["t_write_start_s"]), "t_end": t,
                               "data": data, "tag": json.loads(row["tag"])})
            elif row["port"] in reads:
                reads[row["port"]].append((t, data))
            else:
                events.append((t, row["port"], data))
    writes.sort(key=lambda w: w["t_start"])
    return writes, reads, events


def wire_end(w):
    """Best estimate of when the last byte of a write left the adapter on the UART wire."""
    return max(w["t_end"], w["t_start"] + wire_time(len(w["data"])))


def group_reads(reads, gap):
    """reads: list of (t, bytes). Returns arrival groups: dicts t_first, t_last, data, nreads."""
    groups = []
    for t, d in sorted(reads, key=lambda x: x[0]):
        if groups and t - groups[-1]["t_last"] <= gap:
            groups[-1]["t_last"] = t
            groups[-1]["data"] += d
            groups[-1]["nreads"] += 1
        else:
            groups.append({"t_first": t, "t_last": t, "data": bytearray(d), "nreads": 1})
    return groups


def in_window(reads, t_a, t_b):
    return [(t, d) for t, d in reads if t_a <= t < t_b]


def pct(xs, p):
    xs = sorted(xs)
    if not xs:
        return None
    return xs[min(len(xs) - 1, round(p / 100 * (len(xs) - 1)))]


def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF (same as the earlier bench script)."""
    return binascii.crc_hqx(bytes(data), 0xFFFF)


def pattern(w, n):
    """Distinct bytes per write: no repeated neighbours (so never '+++'), and offsets
    i and i+256 differ, so a fragment can be located inside the original."""
    return bytes((w * 31 + i * 7 + (i >> 8) * 13) & 0xFF for i in range(n))
