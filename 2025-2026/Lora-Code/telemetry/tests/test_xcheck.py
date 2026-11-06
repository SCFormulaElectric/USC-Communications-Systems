"""Cross-check (8.3): a C program writes one packet of each type (every field filled from the
YAML via the generated xcheck_fill.c, compiled against the generated header) plus a real
telem_core write built from CAN frames; Python decodes them with the YAML and checks every field."""
import os
import subprocess
import sys

import pytest

from decoder import ARRAY, field_size

HERE = os.path.dirname(os.path.abspath(__file__))
M = 0xFFFFFFFF


def xv(a, b):
    return ((a * 2654435761) & M) ^ ((b * 40503 + 0x9E37) & M)


def cast(v, t):
    bits = {"u8": 8, "i8": 8, "u16": 16, "i16": 16, "u32": 32, "i32": 32}[t]
    v &= (1 << bits) - 1
    if t.startswith("i") and v >> (bits - 1):
        v -= 1 << bits
    return v


def expected(fields, a):
    out = {}
    for fi, f in enumerate(fields):
        key = a * 64 + fi
        if f["type"] == "struct":
            out[f["name"]] = [{sf["name"]: one(sf, key * 32 + j * 8 + sfi) for sfi, sf in enumerate(f["fields"])}
                              for j in range(f["count"])]
        else:
            out[f["name"]] = one(f, key)
    return out


def one(f, key):
    if ARRAY.match(f["type"]):
        return bytes(xv(key, 1000 + k) & 0xFF for k in range(field_size(f)))
    return cast(xv(key, 0), f["type"])


@pytest.fixture(scope="module")
def xcheck_files():
    try:
        import run_host_tests
        return run_host_tests.build_xcheck()
    except SystemExit as e:
        pytest.fail(f"could not build the C cross-check program: {e}")
    except FileNotFoundError as e:
        pytest.skip(f"no C compiler: {e}")


def test_every_field_of_every_packet(schema, xcheck_files):
    data = open(xcheck_files[0], "rb").read()
    order = ["FAST", "THERM", "DIAG", "EVENT", "DIAG_RESP"]
    assert len(data) == 64 * len(order) + 24 * len(schema.commands)
    checked = 0
    for i, name in enumerate(order):
        pkt = data[64 * i:64 * (i + 1)]
        typ = schema.types[name]
        assert schema.check_packet(pkt), name
        d = schema.decode_packet(pkt)
        assert d["type_name"] == name and d["seq"] == typ and d["uptime_ms"] == (0x01020304 * typ) & M
        exp = expected(schema.payloads[typ]["fields"], typ)
        for k, v in exp.items():
            assert d["fields"][k] == v, f"{name}.{k}"
            checked += 1
    off = 64 * len(order)
    for j, code in enumerate(["PING", "BAMO_READ", "BAMO_WRITE", "SEND_FRAME", "CAPTURE_SET",
                              "CAPTURE_CLEAR", "STOP_ALL"]):
        c = schema.decode_command(data[off + 24 * j: off + 24 * (j + 1)])
        spec = schema.commands[code]
        assert c["crc_ok"] and c["code_name"] == code and c["seq"] == spec["code"] and c["key"] == b"\x5A\xC3"
        assert c["args"] == expected(spec["args"], 100 + spec["code"]), code
        checked += 1
    assert checked > 60


def test_real_firmware_write(schema, xcheck_files):
    data = open(xcheck_files[1], "rb").read()
    assert len(data) == 3 * 64
    p = [data[i * 64:(i + 1) * 64] for i in range(3)]
    assert all(schema.check_packet(x) for x in p)
    ctx = {}
    diag = schema.decode_packet(p[2], ctx)                 # write index 1: packet 3 is DIAG
    assert diag["type_name"] == "DIAG" and diag["fields"]["nmax"] == 6000 and ctx["nmax"] == 6000
    f1, f2 = (schema.decode_packet(x, ctx) for x in p[:2])
    assert f1["type_name"] == f2["type_name"] == "FAST"
    assert f1["uptime_ms"] == 1100 and f2["uptime_ms"] == 1200          # FAST from 100 ms earlier, FAST now
    assert (f2["seq"] - f1["seq"]) & 0xFF == 1 and (diag["seq"] - f2["seq"]) & 0xFF == 1
    f, v = f2["fields"], f2["values"]
    assert f["speed"] == 32767 and v["speed"] == pytest.approx(6000.0)  # rpm = raw / 32767 x Nmax
    assert f["motor_temp"] == 9369 and v["motor_temp"] == pytest.approx(0.0, abs=1e-9)
    assert f["torque_cmd"] == 32767 and v["torque_cmd"] == pytest.approx(100.0)
    assert f["torque_req"] == -200 and v["torque_req"] == pytest.approx(-20.0)
    assert f["brake1"] == 0x1234 and f["brake1_prev"] == 0x1234
    assert f1["fields"]["brake1"] == 0x1234 and f1["fields"]["brake1_prev"] == 0
    assert f["pack_current"] == 0x0102 and f["pack_voltage"] == 0x0304 and f["pack_soc"] == 0x55
    assert f["pack_ocv"] == 0x0607 and f["bms_fault_flags"] == 0xDEADBEEF and f["max_cell_temp"] == 45
    assert set(v["status"]) == {"red_light", "bspd_ok"}
    assert set(v["valid"]) == {"bms_301", "bms_303", "therm_380", "bamo_speed", "bamo_temps",
                               "brake1", "torque_req", "torque_cmd", "gevcu_471"}
    assert f["ack_status"] == 0xFF and f["ack_seq"] == 0


def test_gen_schema_check():
    gen = os.path.normpath(os.path.join(HERE, "..", "schema", "gen_schema.py"))
    r = subprocess.run([sys.executable, gen, "--check"], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
