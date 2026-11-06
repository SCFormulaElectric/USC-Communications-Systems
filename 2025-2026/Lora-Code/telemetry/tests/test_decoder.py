"""CRC and encode -> decode round trips for every packet type in the YAML (8.2)."""
import struct

import pytest

from decoder import ARRAY, FMT, crc16, field_size

RANGES = {"u8": (0, 0xFF), "i8": (-128, 127), "u16": (0, 0xFFFF), "i16": (-32768, 32767),
          "u32": (0, 0xFFFFFFFF), "i32": (-2**31, 2**31 - 1)}


def sample_values(fields, salt):
    """A distinctive in-range value for every field (incl. arrays and struct arrays)."""
    out = {}
    for i, f in enumerate(fields):
        t = f["type"]
        k = salt * 97 + i * 31 + 7
        if t == "struct":
            out[f["name"]] = [sample_values(f["fields"], k + j) for j in range(f["count"])]
        elif ARRAY.match(t):
            out[f["name"]] = bytes((k + j * 13) & 0xFF for j in range(field_size(f)))
        else:
            lo, hi = RANGES[t]
            out[f["name"]] = lo + (k * 2654435761) % (hi - lo + 1)
    return out


def test_crc_vector():
    assert crc16(b"123456789") == 0x29B1


def test_yaml_check_value(schema):
    assert crc16(schema.y["crc"]["check"]["input"].encode()) == schema.y["crc"]["check"]["value"]


@pytest.mark.parametrize("name", ["FAST", "THERM", "DIAG", "EVENT", "DIAG_RESP"])
def test_packet_round_trip(schema, name):
    pl = schema.payloads[schema.types[name]]
    vals = sample_values(pl["fields"], schema.types[name])
    pkt = schema.encode_packet(name, vals, seq=0xA7, uptime_ms=0xDEADBEEF)
    assert len(pkt) == 64
    assert schema.check_packet(pkt)
    d = schema.decode_packet(pkt)
    assert d["type_name"] == name and d["seq"] == 0xA7 and d["uptime_ms"] == 0xDEADBEEF and d["crc_ok"]
    assert d["fields"] == vals
    bad = bytearray(pkt)
    bad[20] ^= 0x01
    assert not schema.check_packet(bytes(bad))


@pytest.mark.parametrize("code", ["PING", "BAMO_READ", "BAMO_WRITE", "SEND_FRAME", "CAPTURE_SET",
                                  "CAPTURE_CLEAR", "STOP_ALL"])
def test_command_round_trip(schema, code):
    spec = schema.commands[code]
    args = sample_values(spec["args"], spec["code"])
    b = schema.encode_command(code, seq=0x42, args=args)
    assert len(b) == 24
    assert b[:2] == b"\xA5\x5A" and b[2] == 0x18 and b[4:6] == b"\x5A\xC3" and b[6] == spec["code"]
    assert crc16(b[:22]) == struct.unpack_from("<H", b, 22)[0]
    d = schema.decode_command(b)
    assert d["code_name"] == code and d["seq"] == 0x42 and d["crc_ok"] and d["args"] == args


def test_display_scaling(schema):
    pkt = schema.encode_packet("FAST", {"speed": 16384, "motor_temp": 9369 + 5521, "torque_cmd": 32767,
                                        "torque_req": -125, "valid": 0b101, "status": 0b11})
    ctx = {}
    d = schema.decode_packet(pkt, ctx)
    assert d["values"]["speed"] is None                    # Nmax not known yet: no rpm
    diag = schema.encode_packet("DIAG", {"nmax": 6000, "reg_seen": 1})
    schema.decode_packet(diag, ctx)
    assert ctx["nmax"] == 6000
    v = schema.decode_packet(pkt, ctx)["values"]
    assert v["speed"] == pytest.approx(16384 / 32767 * 6000)
    assert v["motor_temp"] == pytest.approx(100.0)          # (raw - 9369) / 55.21
    assert v["torque_cmd"] == pytest.approx(100.0)
    assert v["torque_req"] == pytest.approx(-12.5)
    assert v["valid"] == ["bms_301", "bms_303"]
    assert v["status"] == ["red_light", "bspd_ok"]
    assert v["dc_bus"] == 0                                 # TODO scaling: raw shown


def test_bamo_frame(schema):
    assert schema.bamo_frame({"id": 0x181, "dlc": 4, "data": bytes([0xC8, 0x70, 0x17, 0, 0, 0, 0, 0])}) == \
        {"reg": 0xC8, "name": "N_MAX (0xC8)", "value": 6000}
    assert schema.bamo_frame({"id": 0x181, "dlc": 6, "data": bytes([0x8F, 1, 2, 3, 0x80, 0, 0, 0])})["value"] == \
        struct.unpack("<i", bytes([1, 2, 3, 0x80]))[0]
    assert schema.bamo_frame({"id": 0x182, "dlc": 4, "data": bytes(8)}) is None


def test_register_names(schema):
    assert schema.parse_reg("N_MAX") == 0xC8
    assert schema.parse_reg("0x35") == 0x35
    for reg in (0x90, 0x31, 0x6E, 0x51, 0x68, 0x84, 0xDC):
        assert schema.registers[reg].get("blocked")
    for reg in (0x30, 0x49, 0x4A):
        assert schema.registers[reg].get("read_blocked")
    assert FMT["i16"] == "<h"
