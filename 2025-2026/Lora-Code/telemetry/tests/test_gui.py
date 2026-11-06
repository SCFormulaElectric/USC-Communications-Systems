"""GUI: the row model's change detection, the simulated car, and a real Tk smoke test in demo mode."""
import time
import types

import pytest

from lora_link import PacketParser
from sim_port import SimPort
from telemetry_gui import NEW_S, RowModel


def test_row_model_highlights_only_changes(schema):
    m = RowModel(schema, "FAST")
    ids = [r[0] for r in m.rows]
    assert "FAST.speed" in ids and "FAST.valid.bms_301" in ids and "FAST.status.r2d" in ids
    assert not any(i.startswith("FAST.reserved") for i in ids)
    d1 = schema.decode_packet(schema.encode_packet("FAST", {"speed": 100, "valid": 1, "status": 0x10}), {})
    first = {rid: hl for rid, _v, _r, hl in m.update(d1, 1.0)}
    assert all(first.values())                                   # everything is new the first time
    d2 = schema.decode_packet(schema.encode_packet("FAST", {"speed": 101, "valid": 1, "status": 0x10}), {})
    second = {rid: (v, hl) for rid, v, _r, hl in m.update(d2, 1.1)}
    assert second["FAST.speed"][1] is True                       # changed -> highlight
    assert second["FAST.valid.bms_301"] == ("OK", False)         # same value -> no highlight
    assert second["FAST.status.r2d"] == ("yes", False)
    assert second["FAST.valid.bms_302"] == ("timeout", False)
    assert m.shown["FAST.ack_status"][0] == "OK"                 # enum shown by name (0 = OK)
    every = {rid: hl for rid, _v, _r, hl in m.update(d2, 1.2, every=True)}
    assert all(every.values())                                   # "highlight every update"


def test_row_model_therm_motor_temp(schema):
    m = RowModel(schema, "THERM")
    d = schema.decode_packet(schema.encode_packet("THERM", {"motor_temp_vcu": bytes([0, 40]), "seen": 0x40}), {})
    rows = {rid: (v, raw) for rid, v, raw, _ in m.update(d, 1.0)}
    assert rows["THERM.motor_temp_vcu"] == ("40", "00 28")


def test_sim_port_makes_valid_batches(schema):
    sim = SimPort(schema)
    p = PacketParser(schema)
    t_end = time.perf_counter() + 1.3
    got = []
    while time.perf_counter() < t_end:
        n = sim.in_waiting
        got += [e[2] for e in p.feed(sim.read(n or 1), time.perf_counter()) if e[0] == "packet"]
    assert len(got) >= 15 and p.bad_crc == 0
    names = {schema.decode_packet(x)["type_name"] for x in got}
    assert {"FAST", "THERM", "DIAG"} <= names


def test_gui_smoke_demo(schema):
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError as e:
        pytest.skip(f"no display: {e}")
    import telemetry_gui
    args = types.SimpleNamespace(port=None, demo=True, delay_ms=110, log_dir="logs", no_log=True)
    app = telemetry_gui.App(root, schema, args)
    try:
        t_end = time.time() + 3.0
        saw_new = False
        while time.time() < t_end:
            root.update()
            if "new" in app.trees["FAST"].item("FAST.brake1", "tags"):   # brake1 varies; speed is 0 while idle
                saw_new = True
            time.sleep(0.02)
        tree = app.trees["FAST"]
        assert tree.item("FAST.pack_voltage", "values")[0] != ""      # values arrived
        assert tree.item("FAST.status.bspd_ok", "values")[0] == "yes"
        assert app.trees["THERM"].item("THERM.motor_temp_vcu", "values")[0] != ""
        assert saw_new
        # a value that stopped changing goes back to normal after NEW_S
        assert "new" not in tree.item("FAST.pack_soc", "tags")
        assert app.models["FAST"].shown["FAST.pack_soc"][0] == "85"
        # diagnostics through the CLI parser, answered by the simulated car
        app.cmd.insert(0, "read N_MAX")
        app.send_cmd()
        t_end = time.time() + 3.0
        while time.time() < t_end and "N_MAX (0xC8) = 6000" not in app.out.get("1.0", "end"):
            root.update()
            time.sleep(0.02)
        assert "N_MAX (0xC8) = 6000" in app.out.get("1.0", "end")
        assert "-> OK" in app.out.get("1.0", "end")
        assert NEW_S == 0.5
    finally:
        app.on_close()
