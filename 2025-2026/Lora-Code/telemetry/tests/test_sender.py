"""Command sender: acknowledgement, resend, give-up and lockup states with a fake clock and a
fake port (8.2). Batches 'arrive' every 200 ms; the test calls fire() at the scheduled time."""
import pytest

from lora_link import CommandSender

PERIOD = 0.200


class Rig:
    def __init__(self, schema, **kw):
        self.schema = schema
        self.writes = []                 # (t, bytes)
        self.results = []
        self.states = []
        self.t = 10.0
        self.s = CommandSender(schema, self._write, on_result=lambda c, st: self.results.append((c, st)),
                               on_state=self.states.append, **kw)
        self.s.next_seq = 5
        self._now = None

    def _write(self, data):
        self.writes.append((self._now, data))

    def batch(self, ack=None, gap=0):
        """One batch arrives at self.t; optionally carries an ack (seq, status) and a seq gap."""
        if gap:
            self.s.on_gap(gap)
        if ack is not None:
            self.s.on_ack(*ack, self.t)
        self.s.on_batch(self.t)
        self.s.poll(self.t)
        due = self.s.due
        if due is not None:
            self._now = due
            assert self.s.fire(due)
        self.t += PERIOD
        return due

    def cmd(self, i=-1):
        return self.schema.decode_command(self.writes[i][1][3:])


def test_send_at_arrival_plus_110ms_with_header(schema):
    r = Rig(schema)
    r.s.submit("BAMO_READ", {"reg": 0xC8, "period": 0})
    due = r.batch()
    assert due == pytest.approx(10.0 + 0.110)
    t, data = r.writes[0]
    assert data[:3] == b"\x0A\x02\x03" and len(data) == 27
    c = r.cmd()
    assert c["code_name"] == "BAMO_READ" and c["seq"] == 5 and c["args"] == {"reg": 0xC8, "period": 0}


def test_ack_completes(schema):
    r = Rig(schema)
    r.s.submit("PING")
    r.batch()
    r.batch(ack=(4, 0))                  # someone else's ack: ignored
    r.batch(ack=(5, 0xFF))               # 'no command yet': ignored
    assert r.s.inflight is not None
    r.batch(ack=(5, 0))
    assert r.results[-1][1] == 0 and r.s.inflight is None
    assert len(r.writes) == 1


def test_resend_after_5_batches_same_seq(schema):
    r = Rig(schema)
    r.s.submit("PING")
    r.batch()                            # send at 10.110
    dues = [r.batch() for _ in range(5)]
    assert dues[:4] == [None] * 4        # waiting
    assert dues[4] == pytest.approx(11.0 + 0.110)   # 5th batch without ack: resend
    assert len(r.writes) == 2 and r.cmd(0)["seq"] == r.cmd(1)["seq"] == 5
    r.batch(ack=(5, 0))
    assert r.results[-1][1] == 0


def test_give_up_after_10_tries_then_next(schema):
    r = Rig(schema)
    r.s.submit("PING")
    r.s.submit("STOP_ALL")
    for _ in range(1 + 10 * 5):
        r.batch()
    assert len([w for w in r.writes if r.schema.decode_command(w[1][3:])["code_name"] == "PING"]) == 10
    assert r.results[0][1] is None and r.results[0][0]["tries"] == 10   # gave up
    assert r.cmd()["code_name"] == "STOP_ALL" and r.cmd()["seq"] == 6


def test_lockup_pauses_and_power_on_resumes(schema):
    r = Rig(schema)
    r.s.submit("PING")
    r.batch()                                        # try 1
    for k in range(15):                              # 3 resends, missing packets in between
        r.batch(gap=1 if k == 7 else 0)
    assert len(r.writes) == 4                        # 1 send + 3 resends
    for _ in range(5):
        r.batch()                                    # 5 more batches unanswered -> lockup
    assert r.s.paused and r.states[-1] == CommandSender.PAUSED_TEXT
    n = len(r.writes)
    for _ in range(20):
        r.batch()
    assert len(r.writes) == n                        # nothing sent while paused
    r.s.on_power_on(r.t)
    assert not r.s.paused and r.states[-1] == "OK"
    r.batch()                                        # resent at the next batch
    assert len(r.writes) == n + 1 and r.cmd()["seq"] == 5
    assert r.s.inflight["tries"] == 1                # tries during the lockup don't count
    r.batch(ack=(5, 0))
    assert r.results[-1][1] == 0


def test_lockup_resumes_after_90s(schema):
    r = Rig(schema)
    r.s.submit("PING")
    r.batch()
    for k in range(20):
        r.batch(gap=1 if k == 6 else 0)
    assert r.s.paused
    t_pause = r.s.paused_since
    while r.t < t_pause + 89.9:
        r.batch()
    assert r.s.paused
    while r.s.paused:
        r.batch()
    assert r.t - PERIOD >= t_pause + 90.0 - 1e-9


def test_no_lockup_without_missing_packets(schema):
    r = Rig(schema)
    r.s.submit("PING")
    for _ in range(1 + 6 * 5):
        r.batch()
    assert not r.s.paused and len(r.writes) == 7     # keeps resending


def test_min_gap_150ms_between_pc_writes(schema):
    r = Rig(schema)
    r.s.submit("PING")
    r.batch()                                        # wrote at 10.110
    r.s.on_ack(5, 0)
    r.s.submit("PING")
    r.s.on_batch(10.130)                             # an early batch: 10.240 is < 150 ms later
    assert r.s.due == pytest.approx(10.110 + 0.150)
    assert not r.s.fire(10.255)
    assert r.s.fire(10.260)


def test_keepalive_ping_every_3s_until_stopall(schema):
    r = Rig(schema)
    r.s.submit("BAMO_READ", {"reg": 0x25, "period": 0})
    r.batch()
    r.batch(ack=(5, 0))
    while r.t < 20.0:
        due = r.batch(ack=(r.s.inflight["seq"], 0) if r.s.inflight and r.s.inflight["tries"] else None)
    pings = [w for w in r.writes if r.schema.decode_command(w[1][3:])["code_name"] == "PING"]
    assert 2 <= len(pings) <= 4
    gaps = [b[0] - a[0] for a, b in zip(r.writes, r.writes[1:])]
    assert all(g >= 3.0 - 1e-9 for g in gaps[1:])
    r.s.submit("STOP_ALL")
    r.batch()
    r.batch(ack=(r.s.inflight["seq"], 0))
    assert not r.s.session
    n = len(r.writes)
    for _ in range(30):
        r.batch()
    assert len(r.writes) == n                        # session closed: no more pings


def test_seq_starts_after_car_ack(schema):
    r = Rig(schema)
    r.s.seq_known = False
    r.s.note_car_ack_seq(41)
    r.s.submit("PING")
    r.batch()
    assert r.cmd()["seq"] == 42
    r.s.note_car_ack_seq(99)                         # only the first one counts
    assert r.s.next_seq == 43
