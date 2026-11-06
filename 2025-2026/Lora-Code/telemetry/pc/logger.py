"""Telemetry logging (BUILD_INSTRUCTIONS.md 7.3).

    logs/raw_<date>_<time>.bin   master record: [f64 PC time, epoch seconds][64 B packet]
    logs/<date>_<time>/fast.csv, therm.csv, diag.csv, events.csv, diag_resp.csv, commands.csv

Everything except commands.csv can be rebuilt from the raw file with replay.py.
"""
import csv
import datetime
import os
import struct
import time

RAW_REC = struct.Struct("<d64s")


class TelemetryLogger:
    def __init__(self, log_dir, schema, raw=True, stamp=None, csv_dir=None):
        self.schema = schema
        stamp = stamp or datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs(log_dir, exist_ok=True)
        self.raw_path = os.path.join(log_dir, f"raw_{stamp}.bin") if raw else None
        self.raw = open(self.raw_path, "ab") if raw else None
        self.dir = csv_dir or os.path.join(log_dir, stamp)
        os.makedirs(self.dir, exist_ok=True)
        self.files, self.writers = {}, {}
        # perf_counter -> wall clock, so CSV times are readable
        self.t_offset = time.time() - time.perf_counter()
        self.ctx = {}

    # ------------------------------------------------------------------ helpers
    def _w(self, name, header):
        if name not in self.writers:
            path = os.path.join(self.dir, name)
            new = not os.path.exists(path) or os.path.getsize(path) == 0
            f = open(path, "a", newline="")
            self.files[name] = f
            self.writers[name] = csv.writer(f)
            if new:
                self.writers[name].writerow(header)
        return self.writers[name]

    def _wall(self, t):
        return f"{t + self.t_offset:.3f}"

    def _payload_columns(self, type_name):
        pl = self.schema.payloads[self.schema.types[type_name]]
        cols, scaled = [], []
        for f in pl["fields"]:
            if f["type"] == "struct":
                continue
            cols.append(f["name"])
            if "scale" in f or "offset" in f or "scale_ref" in f:
                unit = f.get("unit", "")
                scaled.append(f"{f['name']}_{unit}" if unit else f"{f['name']}_scaled")
            if f.get("bits"):
                scaled.append(f"{f['name']}_names")
        return pl, cols, scaled

    # ------------------------------------------------------------------ packets
    def write_packet(self, t, pkt, d=None, raw=True):
        if raw and self.raw:
            self.raw.write(RAW_REC.pack(t + self.t_offset, bytes(pkt)))   # epoch seconds
            self.raw.flush()
        d = d or self.schema.decode_packet(pkt, self.ctx)
        name = d["type_name"]
        base = [self._wall(t), d["seq"], d["uptime_ms"]]
        if name in ("FAST", "THERM", "DIAG"):
            pl, cols, scaled = self._payload_columns(name)
            w = self._w(f"{name.lower()}.csv", ["pc_time", "seq", "uptime_ms"] + cols + scaled)
            row = base[:]
            for c in cols:
                v = d["fields"][c]
                row.append(v.hex() if isinstance(v, bytes) else (f"0x{v:04X}" if c in ("valid", "status") else v))
            for f in pl["fields"]:
                if f["type"] == "struct":
                    continue
                if "scale" in f or "offset" in f or "scale_ref" in f:
                    v = d["values"].get(f["name"])
                    row.append("" if v is None else round(v, 4))
                if f.get("bits"):
                    row.append(" ".join(d["values"][f["name"]]))
            w.writerow(row)
        elif name == "EVENT":
            w = self._w("events.csv", ["pc_time", "seq", "uptime_ms", "event_time_ms", "age_ms", "code", "name", "value"])
            f = d["fields"]
            for r in f["records"][:f["count"]]:
                w.writerow(base + [d["uptime_ms"] - r["age_ms"], r["age_ms"], r["code"],
                                   self.schema.event_name(r["code"]), r["value"]])
        elif name == "DIAG_RESP":
            w = self._w("diag_resp.csv", ["pc_time", "seq", "uptime_ms", "kind", "ack_seq", "status", "status_name",
                                          "age_ms", "id", "ext", "dlc", "data", "bamo_reg", "bamo_value",
                                          "cap_dropped"])
            f = d["fields"]
            for a in f["acks"][:f["n_acks"]]:
                w.writerow(base + ["ack", a["seq"], a["status"], self.schema.status_name(a["status"])] + [""] * 8)
            for fr in f["frames"][:f["n_frames"]]:
                ext = bool(fr["id"] & 0x80000000)
                fid = fr["id"] & 0x1FFFFFFF
                b = self.schema.bamo_frame({"id": fid if not ext else -1, "dlc": fr["dlc"], "data": fr["data"]})
                w.writerow(base + ["frame", "", "", "", fr["age_ms"], f"0x{fid:X}", int(ext), fr["dlc"],
                                   fr["data"][:fr["dlc"]].hex(), b["name"] if b else "", b["value"] if b else "",
                                   ""])
            if f["cap_dropped"]:
                w.writerow(base + ["dropped"] + [""] * 10 + [f["cap_dropped"]])
        self.flush()

    # ------------------------------------------------------------------ commands
    def write_command(self, t, cmd, event, status, text=""):
        w = self._w("commands.csv", ["pc_time", "event", "cmd_seq", "code", "args", "attempt", "status",
                                     "status_name", "note"])
        if cmd is None:
            w.writerow([self._wall(t), event, "", "", "", "", "", "", text])
        else:
            args = " ".join(f"{k}={v.hex() if isinstance(v, bytes) else v}" for k, v in cmd["args"].items())
            w.writerow([self._wall(t), event, cmd["seq"], cmd["code_name"], args, cmd["tries"],
                        "" if status is None else status,
                        "" if status is None else self.schema.status_name(status), cmd.get("note", "")])
        self.flush()

    def flush(self):
        for f in self.files.values():
            f.flush()

    def close(self):
        if self.raw:
            self.raw.close()
        for f in self.files.values():
            f.close()


def read_raw(path):
    """Yield (pc_time, packet) from a raw log."""
    with open(path, "rb") as f:
        while True:
            b = f.read(RAW_REC.size)
            if len(b) < RAW_REC.size:
                return
            yield RAW_REC.unpack(b)
