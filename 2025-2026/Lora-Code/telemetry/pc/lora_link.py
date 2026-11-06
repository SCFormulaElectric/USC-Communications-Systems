"""Owns the PC-side LoRa module's serial port (BUILD_INSTRUCTIONS.md 7.1, 7.4).

- PacketParser   : byte stream -> 64 B packets (sync, CRC; on failure move ahead one byte),
                   plus the module's "Power on" / "channel Error:NN" text.
- BatchDetector  : a telemetry batch has arrived after >= 64 B and then 10 ms with no bytes.
- CommandSender  : one command waiting for acknowledgement; sends at batch arrival + 110 ms,
                   >= 150 ms between PC writes, resends after 5 batches, gives up after 10 tries,
                   PING every 3 s while a session is open, pauses during a PC-module lockup.
- LoraLink       : the threads and serial port that tie these together.

PacketParser, BatchDetector and CommandSender take explicit timestamps, so the tests drive them
with a fake clock and a fake port.
"""
import collections
import random
import threading
import time

from decoder import Schema

SYNC = b"\xA5\x5A"
POWER_ON = b"Power on"
CHANNEL_ERROR = b"channel Error:"
TOKENS = (SYNC, POWER_ON, CHANNEL_ERROR)


class PacketParser:
    def __init__(self, schema):
        self.schema = schema
        self.size = schema.pkt["size"]
        self.buf = bytearray()
        self.buf_t = None
        self.good = 0
        self.bad_crc = 0
        self.skipped = 0
        self.power_on = 0
        self.channel_errors = 0

    def feed(self, data, t):
        """Returns a list of events: ('packet', t, bytes), ('power_on', t), ('channel_error', t, text)."""
        self.buf += data
        self.buf_t = t
        out = []
        b = self.buf
        while b:
            hits = [(b.find(tok), tok) for tok in TOKENS]
            hits = [(i, tok) for i, tok in hits if i >= 0]
            if not hits:
                keep = self._partial_tail(b)
                self.skipped += len(b) - keep
                del b[:len(b) - keep]
                break
            i, tok = min(hits)
            if i:
                self.skipped += i
                del b[:i]
            if tok == SYNC:
                if len(b) < self.size:
                    break
                pkt = bytes(b[:self.size])
                if self.schema.check_packet(pkt):
                    self.good += 1
                    out.append(("packet", t, pkt))
                    del b[:self.size]
                else:
                    self.bad_crc += 1
                    self.skipped += 1
                    del b[:1]
            elif tok == POWER_ON:
                self.power_on += 1
                out.append(("power_on", t))
                del b[:len(POWER_ON)]
                self._eat_crlf(b)
            else:
                end = b.find(b"\r\n")
                if end < 0:
                    if len(b) < 40:
                        break               # wait for the rest of the line
                    end = len(CHANNEL_ERROR)
                self.channel_errors += 1
                out.append(("channel_error", t, bytes(b[:end]).decode("ascii", "replace")))
                del b[:end]
                self._eat_crlf(b)
        return out

    @staticmethod
    def _eat_crlf(b):
        while b[:1] in (b"\r", b"\n"):
            del b[:1]

    @staticmethod
    def _partial_tail(b):
        """Bytes at the end of b that could be the start of a token."""
        for n in range(min(len(b), len(CHANNEL_ERROR) - 1), 0, -1):
            tail = bytes(b[-n:])
            if any(tok.startswith(tail) for tok in TOKENS):
                return n
        return 0


class BatchDetector:
    def __init__(self, min_bytes=64, quiet_s=0.010):
        self.min_bytes, self.quiet_s = min_bytes, quiet_s
        self.n = 0
        self.last_t = None

    def feed(self, nbytes, t):
        self.n += nbytes
        self.last_t = t

    def poll(self, now):
        """Arrival time (timestamp of the last read) once a batch is complete, else None."""
        if self.n >= self.min_bytes and now - self.last_t >= self.quiet_s:
            self.n = 0
            return self.last_t
        if self.n and self.n < self.min_bytes and now - self.last_t >= 1.0:
            self.n = 0                       # a stray fragment, not a batch
        return None


class CommandSender:
    """Command state machine. All methods take the current time; `write(bytes)` sends one
    24 B command (the 3-byte header is added here)."""

    PAUSED_TEXT = "PC radio restarting, commands paused"

    def __init__(self, schema, write, delay_s=0.110, min_gap_s=0.150, ack_batches=5, max_tries=10,
                 ping_every_s=3.0, lockup_resends=3, lockup_resume_s=90.0, on_result=None, on_state=None,
                 on_attempt=None):
        self.schema = schema
        self.write = write
        self.delay_s, self.min_gap_s = delay_s, min_gap_s
        self.ack_batches, self.max_tries = ack_batches, max_tries
        self.ping_every_s = ping_every_s
        self.lockup_resends, self.lockup_resume_s = lockup_resends, lockup_resume_s
        self.on_result = on_result or (lambda cmd, status: None)
        self.on_state = on_state or (lambda text: None)
        self.on_attempt = on_attempt or (lambda cmd, t, kind: None)
        self.queue = collections.deque()
        self.inflight = None
        self.next_seq = random.randint(1, 255)
        self.seq_known = False
        self.due = None
        self.last_write = None
        self.session = False
        self.last_cmd_t = None
        self.paused = False
        self.paused_since = None
        self.gap_seen = False
        self.state = "OK"

    # ------------------------------------------------------------------ inputs
    def submit(self, code_name, args=None, note=""):
        cmd = {"code_name": code_name, "args": args or {}, "note": note, "tries": 0, "batches": 0,
               "seq": None, "status": None, "done": False}
        self.queue.append(cmd)
        self.session = True
        return cmd

    def note_car_ack_seq(self, ack_seq):
        """Start our numbering after the car's last acknowledged sequence, so a restarted PC
        tool isn't taken for a repeat (the car doesn't run a repeated sequence number)."""
        if not self.seq_known and self.inflight is None:
            self.next_seq = (ack_seq + 1) & 0xFF or 1
            self.seq_known = True

    def on_batch(self, arrival):
        """A telemetry batch finished arriving at `arrival`."""
        if self.paused:
            self._maybe_resume(arrival, by_timeout=True)
            if self.paused:
                return
        c = self.inflight
        if c is not None and c["tries"]:
            c["batches"] += 1
            if c["batches"] < self.ack_batches:
                return
            if c["tries"] >= self.max_tries:
                self._finish(c, None)                      # gave up
                c = None
            elif c["tries"] - 1 >= self.lockup_resends and self.gap_seen:
                self._pause(arrival)                       # 3 resends unanswered + packets missing
                return
            else:
                self._schedule(arrival, c, "resend")
                return
        if self.inflight is None:
            if not self.queue and self.session and self.last_cmd_t is not None and \
                    arrival - self.last_cmd_t >= self.ping_every_s:
                self.queue.append({"code_name": "PING", "args": {}, "note": "keepalive", "tries": 0,
                                   "batches": 0, "seq": None, "status": None, "done": False})
            if self.queue:
                c = self.queue.popleft()
                c["seq"] = self.next_seq
                self.next_seq = (self.next_seq + 1) & 0xFF or 1
                self.inflight = c
                self.gap_seen = False
        if self.inflight is not None and self.due is None and self.inflight["tries"] == 0:
            self._schedule(arrival, self.inflight, "send")

    def on_ack(self, seq, status, t=None):
        c = self.inflight
        if c is None or not c["tries"] or seq != c["seq"] or status == 0xFF:
            return
        self._finish(c, status)
        if c["code_name"] == "STOP_ALL" and status == 0:
            self.session = False

    def on_gap(self, missing):
        """Packets were missing (sequence gap)."""
        if missing and self.inflight is not None and self.inflight["tries"] >= 2:
            self.gap_seen = True

    def on_power_on(self, t):
        if self.paused:
            self._maybe_resume(t, by_timeout=False)

    def poll(self, now):
        if self.paused:
            self._maybe_resume(now, by_timeout=True)

    def fire(self, now):
        """Write the scheduled command if its time has come. Returns True if it wrote."""
        if self.due is None or now < self.due or self.paused or self.inflight is None:
            return False
        if self.last_write is not None and now - self.last_write < self.min_gap_s:
            return False
        c = self.inflight
        frame = self.schema.encode_command(c["code_name"], c["seq"], c["args"])
        self.write(self.schema.header_to_car + frame)
        kind = "send" if c["tries"] == 0 else "resend"
        c["tries"] += 1
        c["batches"] = 0
        self.last_write = now
        self.last_cmd_t = now
        self.due = None
        self.on_attempt(c, now, kind)
        return True

    # ------------------------------------------------------------------ internals
    def _schedule(self, arrival, c, kind):
        t = arrival + self.delay_s
        if self.last_write is not None:
            t = max(t, self.last_write + self.min_gap_s)
        self.due = t

    def _finish(self, c, status):
        c["status"] = status
        c["done"] = True
        self.inflight = None
        self.due = None
        self.on_result(c, status)

    def _pause(self, t):
        self.paused = True
        self.paused_since = t
        self.due = None
        self.state = self.PAUSED_TEXT
        self.on_state(self.state)

    def _maybe_resume(self, t, by_timeout):
        if by_timeout and t - self.paused_since < self.lockup_resume_s:
            return
        self.paused = False
        self.gap_seen = False
        if self.inflight is not None:
            self.inflight["tries"] = 0          # tries during the lockup don't count
            self.inflight["batches"] = 0
        self.state = "OK"
        self.on_state(self.state)


class LoraLink:
    """Serial port + reader thread + precise command timing thread."""

    def __init__(self, port, schema=None, delay_ms=110, logger=None, baud=115200, serial_factory=None):
        import serial  # imported here so tests can run without a port
        self.schema = schema or Schema()
        self.logger = logger
        factory = serial_factory or serial.Serial
        self.ser = factory(port, baudrate=baud, timeout=0.001)
        self.lock = threading.RLock()
        self.clock = time.perf_counter
        self.parser = PacketParser(self.schema)
        self.batch = BatchDetector()
        self.sender = CommandSender(self.schema, self._write, delay_s=delay_ms / 1000.0,
                                    on_result=self._on_result, on_state=self._on_state,
                                    on_attempt=self._on_attempt)
        self.listeners = []
        self.ctx = {}
        self.last_seq = None
        self.received = 0
        self.missing = 0
        self.recent = collections.deque()          # (t, n_received, n_missing)
        self.last_packet_t = None
        self.latest = {}                           # type name -> decoded packet
        self._run = False

    # ------------------------------------------------------------------ public
    def start(self):
        self._run = True
        for fn in (self._reader, self._timer):
            threading.Thread(target=fn, daemon=True).start()

    def stop(self):
        self._run = False
        time.sleep(0.05)
        self.ser.close()

    def submit(self, code_name, args=None, note=""):
        with self.lock:
            return self.sender.submit(code_name, args, note)

    def stats(self, window_s=5.0):
        now = self.clock()
        with self.lock:
            while self.recent and now - self.recent[0][0] > 30.0:
                self.recent.popleft()
            rx5 = sum(1 for t, r, m in self.recent if now - t <= window_s and r)
            rec = sum(r for _, r, _ in self.recent)
            mis = sum(m for _, _, m in self.recent)
            state = self.sender.state
            if self.last_packet_t is None or now - self.last_packet_t > 2.0:
                state = "no data" if state == "OK" else state
            return {"pps": rx5 / window_s, "loss_pct": 100.0 * mis / (rec + mis) if rec + mis else 0.0,
                    "state": state, "received": self.received, "missing": self.missing,
                    "bad_crc": self.parser.bad_crc, "skipped": self.parser.skipped,
                    "power_on": self.parser.power_on, "session": self.sender.session,
                    "inflight": self.sender.inflight, "queued": len(self.sender.queue)}

    # ------------------------------------------------------------------ threads
    def _write(self, data):
        self.ser.write(data)

    def _reader(self):
        while self._run:
            try:
                n = self.ser.in_waiting
                data = self.ser.read(n if n else 1)
            except Exception as e:                  # port gone
                self._emit({"kind": "error", "text": str(e)})
                time.sleep(0.5)
                continue
            now = self.clock()
            with self.lock:
                if data:
                    self.batch.feed(len(data), now)
                    for ev in self.parser.feed(data, now):
                        self._handle(ev)
                arrival = self.batch.poll(now)
                if arrival is not None:
                    self.sender.on_batch(arrival)
                    self._emit({"kind": "batch", "t": arrival})
                self.sender.poll(now)

    def _timer(self):
        while self._run:
            with self.lock:
                due = self.sender.due
            if due is None:
                time.sleep(0.005)
                continue
            rem = due - self.clock()
            if rem > 0.025:
                time.sleep(min(rem - 0.020, 0.050))   # time.sleep is coarse on Windows
                continue
            while self.clock() < due:
                pass                                   # busy-wait the last ~20 ms
            with self.lock:
                self.sender.fire(self.clock())

    # ------------------------------------------------------------------ packet handling
    def _handle(self, ev):
        kind, t = ev[0], ev[1]
        if kind == "power_on":
            self.sender.on_power_on(t)
            self._emit({"kind": "power_on", "t": t})
            return
        if kind == "channel_error":
            self._emit({"kind": "channel_error", "t": t, "text": ev[2]})
            return
        pkt = ev[2]
        d = self.schema.decode_packet(pkt, self.ctx)
        self.last_packet_t = t
        self.received += 1
        missing = 0
        if self.last_seq is not None:
            diff = (d["seq"] - self.last_seq) & 0xFF
            if 1 < diff < 128:
                missing = diff - 1
        self.last_seq = d["seq"]
        self.missing += missing
        self.recent.append((t, 1, missing))
        if missing:
            self.sender.on_gap(missing)
        if d["type_name"] == "FAST":
            f = d["fields"]
            if f["ack_status"] != 0xFF:
                self.sender.note_car_ack_seq(f["ack_seq"])
            self.sender.on_ack(f["ack_seq"], f["ack_status"], t)
        elif d["type_name"] == "DIAG_RESP":
            f = d["fields"]
            for a in f["acks"][:f["n_acks"]]:
                self.sender.on_ack(a["seq"], a["status"], t)
        self.latest[d["type_name"]] = d
        if self.logger:
            self.logger.write_packet(t, pkt, d)
        self._emit({"kind": "packet", "t": t, "packet": d, "missing": missing})

    def _on_result(self, cmd, status):
        if self.logger:
            self.logger.write_command(self.clock(), cmd, "acked" if status is not None else "gave_up", status)
        self._emit({"kind": "result", "cmd": cmd, "status": status})

    def _on_attempt(self, cmd, t, kind):
        if self.logger:
            self.logger.write_command(t, cmd, kind, None)

    def _on_state(self, text):
        if self.logger:
            self.logger.write_command(self.clock(), None, "link_state", None, text)
        self._emit({"kind": "state", "text": text})

    def _emit(self, ev):
        for fn in list(self.listeners):
            try:
                fn(ev)
            except Exception:
                pass
