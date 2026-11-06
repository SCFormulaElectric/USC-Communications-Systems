"""Stream parser on garbage, merged packets, cut packets and module text (8.2)."""
from lora_link import BatchDetector, PacketParser


def pkts(schema, n, start=0):
    return [schema.encode_packet("FAST", {"speed": 100 + i}, seq=start + i) for i in range(n)]


def packets(events):
    return [e[2] for e in events if e[0] == "packet"]


def test_garbage_then_packet(schema):
    p = PacketParser(schema)
    junk = bytes([0x00, 0xA5, 0x11, 0xA5, 0x5A, 0x01, 0x02] + [0x5A] * 20)
    good = pkts(schema, 1)[0]
    ev = p.feed(junk + good, 1.0)
    assert packets(ev) == [good]
    assert p.bad_crc >= 1                       # the fake A5 5A candidate failed its CRC
    assert p.skipped >= len(junk)


def test_merged_packets_with_header_between(schema):
    # the module merges writes that arrive within ~101 ms; the 2nd write's 0A 01 03 header
    # comes through as data
    a, b, c = pkts(schema, 3)
    p = PacketParser(schema)
    ev = p.feed(a + b"\x0A\x01\x03" + b + c, 2.0)
    assert packets(ev) == [a, b, c]
    assert p.skipped == 3


def test_cut_packet_across_reads_and_a_truly_cut_packet(schema):
    a, b, c = pkts(schema, 3)
    p = PacketParser(schema)
    assert packets(p.feed(a[:30], 1.0)) == []
    assert packets(p.feed(a[30:] + b[:10], 1.001)) == [a]
    assert packets(p.feed(b[10:], 1.002)) == [b]
    # a packet cut by the radio (only its first 20 B arrive), then a good one
    ev = p.feed(c[:20] + a, 2.0)
    assert packets(ev) == [a]
    assert p.good == 3


def test_power_on_and_channel_error_text(schema):
    a, b = pkts(schema, 2)
    p = PacketParser(schema)
    ev = p.feed(b"Power on\r\n" + a + b"channel Error:66\r\n", 1.0)
    ev += p.feed(b"chan", 1.1)                  # split across reads
    ev += p.feed(b"nel Error:31\r\n" + b, 1.2)
    kinds = [e[0] for e in ev]
    assert kinds == ["power_on", "packet", "channel_error", "channel_error", "packet"]
    assert [e[2] for e in ev if e[0] == "channel_error"] == ["channel Error:66", "channel Error:31"]
    assert p.power_on == 1 and p.channel_errors == 2
    assert packets(ev) == [a, b]


def test_power_on_split(schema):
    p = PacketParser(schema)
    ev = p.feed(b"Pow", 1.0) + p.feed(b"er on\r\n", 1.1)
    assert [e[0] for e in ev] == ["power_on"]


def test_batch_detector():
    bd = BatchDetector()
    bd.feed(100, 1.000)
    bd.feed(95, 1.003)
    assert bd.poll(1.010) is None               # only 7 ms quiet
    assert bd.poll(1.013) == 1.003              # arrival = last read time
    assert bd.poll(1.100) is None
    bd.feed(30, 2.0)
    assert bd.poll(2.05) is None                # < 64 B is not a batch
