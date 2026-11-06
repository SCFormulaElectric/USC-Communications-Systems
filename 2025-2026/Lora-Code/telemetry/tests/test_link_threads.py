"""LoraLink end to end in real time with a fake serial port: a simulated car sends a batch every
200 ms and acknowledges commands in FAST bytes 52-53. Checks the threads, the batch-arrival
detection, the +110 ms send time (busy-wait) and the acknowledgement path."""
import threading
import time

from lora_link import LoraLink


class FakeCar:
    """Looks like a pyserial port to LoraLink."""

    def __init__(self, schema):
        self.s = schema
        self.lock = threading.Lock()
        self.t0 = time.perf_counter()
        self.next_batch = self.t0 + 0.05
        self.buf = bytearray()
        self.seq = 0
        self.ack = (0, 0xFF)
        self.batch_times = []
        self.writes = []                    # (t, bytes)

    def _pump(self):
        now = time.perf_counter()
        if now >= self.next_batch:
            self.batch_times.append(now)
            for typ in ("FAST", "FAST", "DIAG"):
                fields = {"ack_seq": self.ack[0], "ack_status": self.ack[1]} if typ == "FAST" else {"nmax": 6000, "reg_seen": 1}
                self.buf += self.s.encode_packet(typ, fields, seq=self.seq, uptime_ms=int((now - self.t0) * 1000))
                self.seq = (self.seq + 1) & 0xFF
            self.next_batch += 0.200

    @property
    def in_waiting(self):
        with self.lock:
            self._pump()
            return len(self.buf)

    def read(self, n=1):
        with self.lock:
            self._pump()
            if not self.buf:
                time.sleep(0.001)
                return b""
            out = bytes(self.buf[:n])
            del self.buf[:n]
            return out

    def write(self, data):
        t = time.perf_counter()
        with self.lock:
            self.writes.append((t, bytes(data)))
            c = self.s.decode_command(data[3:])
            if data[:3] == b"\x0A\x02\x03" and c["crc_ok"]:
                self.ack = (c["seq"], 0)       # executed OK; reported in the next FAST
        return len(data)

    def close(self):
        pass


def test_link_sends_110ms_after_arrival_and_gets_ack(schema):
    car = FakeCar(schema)
    link = LoraLink("FAKE", schema, serial_factory=lambda *a, **k: car)
    results = []
    link.listeners.append(lambda ev: results.append(ev) if ev["kind"] == "result" else None)
    link.start()
    try:
        time.sleep(0.5)
        link.submit("BAMO_READ", {"reg": 0xC8, "period": 0})
        t_end = time.time() + 3.0
        while not results and time.time() < t_end:
            time.sleep(0.01)
        assert results and results[0]["status"] == 0
        t_w = car.writes[0][0]
        t_b = max(t for t in car.batch_times if t < t_w)
        delay = t_w - t_b
        assert 0.108 <= delay <= 0.130, f"command written {delay * 1000:.1f} ms after the batch"
        st = link.stats()
        assert st["received"] >= 6 and st["missing"] == 0 and st["bad_crc"] == 0
        assert link.latest["FAST"]["fields"]["ack_seq"] == results[0]["cmd"]["seq"]
    finally:
        link.stop()
