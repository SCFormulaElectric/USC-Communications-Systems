"""Raw log -> replay rebuilds the per-type CSVs (7.3)."""
import csv
import os

from logger import TelemetryLogger, read_raw
from replay import replay


def test_log_and_replay(schema, tmp_path):
    lg = TelemetryLogger(str(tmp_path), schema, stamp="test")
    pk = [
        schema.encode_packet("FAST", {"speed": 1000, "valid": 0x10}, seq=1, uptime_ms=1000),
        schema.encode_packet("THERM", {"f380": bytes(range(8)), "seen": 1}, seq=2, uptime_ms=1000),
        schema.encode_packet("DIAG", {"nmax": 6000, "reg_seen": 1}, seq=3, uptime_ms=1000),
        schema.encode_packet("EVENT", {"count": 2, "records": [{"age_ms": 5, "code": 2, "value": 0},
                                                               {"age_ms": 9, "code": 10, "value": 3}]},
                             seq=5, uptime_ms=1200),                        # seq 4 missing
        schema.encode_packet("DIAG_RESP", {"n_acks": 1, "acks": [{"seq": 7, "status": 4}], "n_frames": 1,
                                           "frames": [{"age_ms": 3, "id": 0x181, "dlc": 4,
                                                       "data": bytes([0xC8, 0x70, 0x17, 0, 0, 0, 0, 0])}]},
                             seq=6, uptime_ms=1200),
    ]
    for i, p in enumerate(pk):
        lg.write_packet(100.0 + i * 0.01, p)
    lg.close()
    recs = list(read_raw(lg.raw_path))
    assert [r[1] for r in recs] == pk
    out, counts, missing = replay(lg.raw_path, str(tmp_path / "replayed"), schema)
    assert counts == {"FAST": 1, "THERM": 1, "DIAG": 1, "EVENT": 1, "DIAG_RESP": 1}
    assert missing == 1
    rows = lambda name: list(csv.DictReader(open(os.path.join(out, name), newline="")))
    ev = rows("events.csv")
    assert [(r["name"], r["event_time_ms"]) for r in ev] == [("R2D_ON", "1195"), ("BMS_FAULT_CHANGED", "1191")]
    dr = rows("diag_resp.csv")
    assert dr[0]["kind"] == "ack" and dr[0]["status_name"] == "REJ_REGID"
    assert dr[1]["bamo_reg"] == "N_MAX (0xC8)" and dr[1]["bamo_value"] == "6000"
    fast = rows("fast.csv")
    assert fast[0]["speed"] == "1000" and fast[0]["valid_names"] == "bamo_speed"
    # replay rebuilds the same CSV content as live logging
    for name in ("fast.csv", "events.csv", "diag_resp.csv", "therm.csv", "diag.csv"):
        assert open(os.path.join(out, name)).read() == open(os.path.join(lg.dir, name)).read(), name
