"""Packet decoder/encoder driven by telemetry/schema/telemetry_schema.yaml (BUILD_INSTRUCTIONS 7.2).

    s = Schema()                       # loads the YAML next to this repo
    d = s.decode_packet(pkt64)         # {'type_name': 'FAST', 'seq': 7, 'fields': {...}, 'values': {...}}
    s.encode_packet('FAST', {...})     # 64 B, for tests and simulators
    s.encode_command('BAMO_READ', seq=3, args={'reg': 0xC8, 'period': 0})   # 24 B
"""
import binascii
import os
import re
import struct

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SCHEMA = os.path.normpath(os.path.join(HERE, "..", "schema", "telemetry_schema.yaml"))
FMT = {"u8": "<B", "i8": "<b", "u16": "<H", "i16": "<h", "u32": "<I", "i32": "<i"}
ARRAY = re.compile(r"^u8\[(\d+)\]$")
EXT_FLAG = 0x80000000


def crc16(data):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    return binascii.crc_hqx(bytes(data), 0xFFFF)


def field_size(f):
    if f["type"] == "struct":
        return f["count"] * f["size"]
    m = ARRAY.match(f["type"])
    return int(m.group(1)) if m else struct.calcsize(FMT[f["type"]])


class Schema:
    def __init__(self, path=DEFAULT_SCHEMA):
        with open(path, encoding="utf-8") as fh:
            self.y = yaml.safe_load(fh)
        y = self.y
        self.pkt = y["packet"]
        self.cmd = y["command"]
        self.transport = y["transport"]
        self.types = dict(y["types"])                         # name -> number
        self.type_names = {v: k for k, v in self.types.items()}
        self.payloads = {p["type"]: dict(p, name=n) for n, p in y["payloads"].items()}
        self.status_codes = {int(k): v for k, v in y["status_codes"].items()}
        self.status_by_name = {v: k for k, v in self.status_codes.items()}
        self.event_codes = {int(k): v for k, v in y["event_codes"].items()}
        self.registers = {int(k): v for k, v in y["bamocar_registers"].items()}
        self.reg_by_name = {v["name"].upper(): k for k, v in self.registers.items()}
        self.commands = {n: c for n, c in self.cmd["codes"].items()}
        self.command_names = {c["code"]: n for n, c in self.commands.items()}
        self.bit_tables = {k: y[k] for k in ("valid_bits", "status_bits", "build_opt_bits")}
        self.header_to_car = bytes(self.transport["header_pc_to_car"])
        self.header_to_pc = bytes(self.transport["header_car_to_pc"])

    # ------------------------------------------------------------------ names
    def status_name(self, code):
        return self.status_codes.get(code, f"STATUS_{code}")

    def event_name(self, code):
        e = self.event_codes.get(code)
        return e["name"] if e else f"EVENT_{code}"

    def reg_name(self, reg):
        r = self.registers.get(reg)
        return f"{r['name']} (0x{reg:02X})" if r else f"0x{reg:02X}"

    def parse_reg(self, text):
        """'0xC8', '200' or a register name like 'N_MAX'."""
        t = text.strip().upper()
        if t in self.reg_by_name:
            return self.reg_by_name[t]
        return int(text, 0)

    def bit_names(self, table, value):
        return [b["name"] for b in self.bit_tables[table] if value >> b["bit"] & 1]

    # ------------------------------------------------------------------ generic fields
    def _decode_fields(self, fields, buf, base):
        out = {}
        for f in fields:
            pos = base + f["pos"]
            t = f["type"]
            if t == "struct":
                out[f["name"]] = [self._decode_fields(f["fields"], buf, pos + i * f["size"])
                                  for i in range(f["count"])]
            elif ARRAY.match(t):
                out[f["name"]] = bytes(buf[pos:pos + field_size(f)])
            else:
                out[f["name"]] = struct.unpack_from(FMT[t], buf, pos)[0]
        return out

    def _encode_fields(self, fields, buf, base, values):
        for f in fields:
            if f["name"] not in values:
                continue
            v = values[f["name"]]
            pos = base + f["pos"]
            t = f["type"]
            if t == "struct":
                for i, rec in enumerate(v[:f["count"]]):
                    self._encode_fields(f["fields"], buf, pos + i * f["size"], rec)
            elif ARRAY.match(t):
                b = bytes(v)[:field_size(f)]
                buf[pos:pos + len(b)] = b
            else:
                struct.pack_into(FMT[t], buf, pos, v)

    # ------------------------------------------------------------------ telemetry packets
    def check_packet(self, pkt):
        """True if 64 B with sync, version and a good CRC."""
        p = self.pkt
        return (len(pkt) == p["size"] and pkt[0] == p["sync"][0] and pkt[1] == p["sync"][1]
                and pkt[2] >> 4 == p["version"]
                and crc16(pkt[:p["crc_off"]]) == struct.unpack_from("<H", pkt, p["crc_off"])[0])

    def decode_packet(self, pkt, context=None):
        """Decode a 64 B packet. `context` (dict) carries values needed for scaling between
        packets, e.g. 'nmax' from the latest DIAG; it is updated in place when given."""
        p = self.pkt
        pkt = bytes(pkt)
        hdr = self._decode_fields(p["fields"], pkt, 0)
        typ = pkt[2] & 0x0F
        d = {"type": typ, "type_name": self.type_names.get(typ, f"TYPE_{typ}"), "version": pkt[2] >> 4,
             "seq": hdr["seq"], "uptime_ms": hdr["uptime_ms"],
             "crc_ok": crc16(pkt[:p["crc_off"]]) == struct.unpack_from("<H", pkt, p["crc_off"])[0]}
        pl = self.payloads.get(typ)
        if pl is None:
            d["fields"], d["values"] = {}, {}
            return d
        fields = self._decode_fields(pl["fields"], pkt, p["payload_off"])
        d["fields"] = fields
        ctx = context if context is not None else {}
        if d["type_name"] == "DIAG" and fields.get("reg_seen", 0) & 1 and fields.get("nmax"):
            ctx["nmax"] = fields["nmax"]
        d["values"] = self.scaled(pl, fields, ctx)
        return d

    def scaled(self, pl, fields, ctx):
        """Display values: raw * scale + offset; scale_ref fields need ctx; bits as name lists."""
        vals = {}
        for f in pl["fields"]:
            n = f["name"]
            raw = fields.get(n)
            if f.get("bits"):
                vals[n] = self.bit_names(f["bits"], raw)
            elif "scale_ref" in f:
                ref = f["scale_ref"]
                key = ref["field"].split(".")[-1]
                base = ctx.get(key)
                vals[n] = raw / ref["divisor"] * base if base else None
            elif "scale" in f or "offset" in f:
                vals[n] = raw * f.get("scale", 1) + f.get("offset", 0)
        return vals

    def encode_packet(self, type_name, fields, seq=0, uptime_ms=0):
        p = self.pkt
        typ = self.types[type_name]
        buf = bytearray(p["size"])
        buf[0:2] = bytes(p["sync"])
        buf[2] = (p["version"] << 4) | typ
        buf[3] = seq & 0xFF
        struct.pack_into("<I", buf, 4, uptime_ms & 0xFFFFFFFF)
        self._encode_fields(self.payloads[typ]["fields"], buf, p["payload_off"], fields)
        struct.pack_into("<H", buf, p["crc_off"], crc16(buf[:p["crc_off"]]))
        return bytes(buf)

    # ------------------------------------------------------------------ commands
    def encode_command(self, code_name, seq, args=None):
        c = self.cmd
        spec = self.commands[code_name]
        buf = bytearray(c["size"])
        buf[0:2] = bytes(self.pkt["sync"])
        buf[2] = c["ver_type"]
        buf[3] = seq & 0xFF
        buf[4:6] = bytes(c["key"])
        buf[6] = spec["code"]
        self._encode_fields(spec["args"], buf, c["args_off"], args or {})
        struct.pack_into("<H", buf, c["crc_off"], crc16(buf[:c["crc_off"]]))
        return bytes(buf)

    def decode_command(self, b):
        c = self.cmd
        b = bytes(b)
        code = b[6]
        name = self.command_names.get(code, f"CMD_{code}")
        spec = self.commands.get(name)
        return {"seq": b[3], "code": code, "code_name": name, "key": b[4:6],
                "crc_ok": crc16(b[:c["crc_off"]]) == struct.unpack_from("<H", b, c["crc_off"])[0],
                "args": self._decode_fields(spec["args"], b, c["args_off"]) if spec else {}}

    # ------------------------------------------------------------------ Bamocar frames in DIAG_RESP
    def bamo_frame(self, frame):
        """{'reg', 'name', 'value'} for a captured 0x181 frame (B0 = register, LE value:
        16-bit for DLC 4, 32-bit for DLC 6), else None."""
        if frame["id"] != 0x181 or frame["dlc"] < 3:
            return None
        d = frame["data"]
        if frame["dlc"] >= 6:
            val = struct.unpack_from("<i", d, 1)[0]
        else:
            val = struct.unpack_from("<h", d, 1)[0]
        return {"reg": d[0], "name": self.reg_name(d[0]), "value": val}
