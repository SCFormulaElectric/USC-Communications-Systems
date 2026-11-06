"""telemetry_cli command parsing -> queued commands (no serial port, no timing)."""
import builtins

import pytest

from decoder import EXT_FLAG
from telemetry_cli import Cli


class FakeLink:
    def __init__(self, schema):
        from lora_link import CommandSender
        self.sender = CommandSender(schema, lambda b: None)
        self.listeners = []
        self.latest = {}

    def submit(self, code, args=None, note=""):
        return self.sender.submit(code, args, note)

    def stats(self):
        return {"pps": 15.0, "loss_pct": 0.0, "state": "OK", "received": 1, "missing": 0, "bad_crc": 0,
                "skipped": 0, "power_on": 0, "session": self.sender.session, "inflight": None, "queued": 0}


@pytest.fixture
def cli(schema, capsys):
    c = Cli(FakeLink(schema), schema)
    c.vt = False
    return c


def queued(cli):
    return [(c["code_name"], c["args"]) for c in cli.link.sender.queue]


def test_commands_encode(cli, schema, monkeypatch):
    cli.handle("read N_MAX --every 100")
    q = queued(cli)
    assert q[0] == ("CAPTURE_SET", {"slot": 0, "id": 0x181, "mask": 0x7FF, "rate": 20})   # new session
    assert q[1] == ("BAMO_READ", {"reg": 0xC8, "period": 100})
    cli.handle("stop 0xC8")
    cli.handle("write 0x35 -2")
    cli.handle("write 0x35 70000 --32")
    cli.handle("raw 0x201 3d c8 00")
    cli.handle("raw 0x18EEFF80 01 02")
    cli.handle("capture 0x1839F380 0x1FFFFFF0 --slot 2 --rate 5")
    cli.handle("uncapture 2")
    cli.handle("uncapture")
    cli.handle("ping")
    cli.handle("stopall")
    q = queued(cli)[2:]
    assert q == [
        ("BAMO_READ", {"reg": 0xC8, "period": 0xFF}),
        ("BAMO_WRITE", {"reg": 0x35, "size": 2, "value": 0xFFFE}),
        ("BAMO_WRITE", {"reg": 0x35, "size": 4, "value": 70000}),
        ("SEND_FRAME", {"id": 0x201, "dlc": 3, "data": bytes([0x3D, 0xC8, 0, 0, 0, 0, 0, 0])}),
        ("SEND_FRAME", {"id": 0x18EEFF80 | EXT_FLAG, "dlc": 2, "data": bytes([1, 2, 0, 0, 0, 0, 0, 0])}),
        ("CAPTURE_SET", {"slot": 2, "id": 0x1839F380 | EXT_FLAG, "mask": 0x1FFFFFF0, "rate": 5}),
        ("CAPTURE_CLEAR", {"slot": 2}),
        ("CAPTURE_CLEAR", {"slot": 0xFF}),
        ("PING", {}),
        ("STOP_ALL", {}),
    ]
    for code, args in queued(cli):                 # every queued command encodes
        assert len(schema.encode_command(code, 1, args)) == 24


def test_read_0x8f_warns(cli, monkeypatch):
    asked = []
    monkeypatch.setattr(builtins, "input", lambda prompt="": asked.append(prompt) or "n")
    cli.handle("read 0x8F")
    assert asked and "error clear" in asked[0]
    assert not any(c == "BAMO_READ" for c, _ in queued(cli))
    monkeypatch.setattr(builtins, "input", lambda prompt="": "y")
    cli.handle("read 0x8F")
    assert ("BAMO_READ", {"reg": 0x8F, "period": 0}) in queued(cli)


def test_bad_period_and_status_line(cli, schema, capsys):
    cli.handle("read 0xC8 --every 10")
    assert "20-254" in capsys.readouterr().out
    cli.link.latest["FAST"] = schema.decode_packet(schema.encode_packet("FAST", {"speed": 100, "status": 0x11}), {})
    line = cli.status_line()
    assert "15.0 pkt/s" in line and "red_light" in line and "r2d" in line
    cli.link.latest["DIAG"] = schema.decode_packet(schema.encode_packet("DIAG", {"fw_version": 0x100}), {})
    cli.print_status()
    assert "fw 0x0100" in capsys.readouterr().out


def test_result_messages(cli, schema, capsys):
    cmd = {"code_name": "BAMO_WRITE", "args": {"reg": 0x90, "size": 2, "value": 1}, "tries": 1, "note": ""}
    cli.on_event({"kind": "result", "cmd": cmd, "status": 4})
    cli.on_event({"kind": "result", "cmd": dict(cmd, tries=10), "status": None})
    out = capsys.readouterr().out
    assert "TORQUE_SETPOINT (0x90)" in out and "register blocked" in out
    assert "no acknowledgement after 10 tries" in out
